#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <FeedChannel.h>

namespace {

TEST(FeedSplit, ShortTextIsOnePartWithoutPrefix) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  ASSERT_EQ(feedSplit("mon on rx=4", 100, out, FEED_PARTS_MAX), 1);
  EXPECT_STREQ(out[0], "mon on rx=4");
}

TEST(FeedSplit, TextExactlyAtTheLimitIsOnePart) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text(100, 'a');
  ASSERT_EQ(feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX), 1);
  EXPECT_EQ(std::string(out[0]), text);
}

TEST(FeedSplit, LongTextSplitsOnWordsWithNumberedPrefixes) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text;
  for (int i = 0; i < 30; i++) text += "word" + std::to_string(i % 10) + " ";   // 180 chars
  text.pop_back();
  int n = feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX);
  ASSERT_EQ(n, 2);
  EXPECT_EQ(std::string(out[0]).substr(0, 4), "1/2 ");
  EXPECT_EQ(std::string(out[1]).substr(0, 4), "2/2 ");
  for (int i = 0; i < n; i++) EXPECT_LE(strlen(out[i]), 100u);
  std::string joined = std::string(out[0]).substr(4) + " " + std::string(out[1]).substr(4);
  EXPECT_EQ(joined, text);
}

TEST(FeedSplit, NewlinePreferredOverSpace) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text = std::string(40, 'a') + "\n" + std::string(30, 'b') + " " + std::string(40, 'c');
  int n = feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX);
  ASSERT_EQ(n, 2);
  EXPECT_EQ(std::string(out[0]), "1/2 " + std::string(40, 'a'));
  EXPECT_EQ(std::string(out[1]), "2/2 " + std::string(30, 'b') + " " + std::string(40, 'c'));
}

TEST(FeedSplit, UnbreakableTextIsCutAtThePayloadWidth) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text(250, 'x');
  int n = feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX);
  ASSERT_EQ(n, 3);
  EXPECT_EQ(strlen(out[0]), 100u);
  EXPECT_EQ(strlen(out[1]), 100u);
  EXPECT_EQ(std::string(out[2]), "3/3 " + std::string(250 - 2 * 96, 'x'));
}

TEST(FeedSplit, CapsAtMaxPartsAndMarksTruncation) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text(2000, 'x');
  ASSERT_EQ(feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX), FEED_PARTS_MAX);
  EXPECT_EQ(std::string(out[0]).substr(0, 4), "1/4 ");
  EXPECT_EQ(std::string(out[3]).substr(0, 4), "4/4 ");
  EXPECT_EQ(std::string(out[3]).substr(strlen(out[3]) - 3), "...");
  EXPECT_LE(strlen(out[3]), 100u);
}

TEST(FeedSplit, ExactlyMaxPartsIsNotMarkedTruncated) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  std::string text(96 * 4, 'x');
  ASSERT_EQ(feedSplit(text.c_str(), 100, out, FEED_PARTS_MAX), 4);
  EXPECT_NE(std::string(out[3]).substr(strlen(out[3]) - 3), "...");
}

TEST(FeedSplit, EmptyTextIsOneEmptyPart) {
  char out[FEED_PARTS_MAX][FEED_PART_CAP];
  ASSERT_EQ(feedSplit("", 100, out, FEED_PARTS_MAX), 1);
  EXPECT_STREQ(out[0], "");
}

TEST(ChannelFrames, MessageLayout) {
  uint8_t buf[FEED_FRAME_CAP];
  size_t n = channelMsgV3Frame(buf, sizeof(buf), 3, 0xFF, -8, -97, 0x11223344, "bench: hi");
  ASSERT_EQ(n, 11u + 9u);
  EXPECT_EQ(buf[0], FEED_RESP_CHANNEL_MSG_RECV_V3);
  EXPECT_EQ((int8_t)buf[1], -8);        // snr, quarter dB
  EXPECT_EQ((int16_t)(buf[2] | buf[3] << 8), -97);   // rssi, little-endian
  EXPECT_EQ(buf[4], 3);                 // channel idx
  EXPECT_EQ(buf[5], 0xFF);              // direct
  EXPECT_EQ(buf[6], 0);                 // plain text
  EXPECT_EQ(buf[7], 0x44);              // timestamp, little-endian
  EXPECT_EQ(buf[10], 0x11);
  EXPECT_EQ(memcmp(&buf[11], "bench: hi", 9), 0);
}

TEST(ChannelFrames, MessageTextIsClampedToTheBuffer) {
  uint8_t buf[20];
  size_t n = channelMsgV3Frame(buf, sizeof(buf), 0, 0xFF, 0, 0, 0, "0123456789abcdefghij");
  EXPECT_EQ(n, sizeof(buf));
}

TEST(FeedSlots, NamesAndSecretsAreDistinct) {
  EXPECT_STRNE(feedDefaultName(FEED_STATUS), feedDefaultName(FEED_CLI));
  EXPECT_STREQ(feedDefaultName(FEED_STATUS), "beebo-status");
  EXPECT_STREQ(feedDefaultName(FEED_CLI), "beebo-cli");
}

TEST(FeedSlots, UnassignedTakesTheHighestFreeSlots) {
  uint8_t a = feedAssignSlot(0xFF, 0xFF, 0, 40);
  EXPECT_EQ(a, 39);
  EXPECT_EQ(feedAssignSlot(0xFF, a, 0, 40), 38);
}

TEST(FeedSlots, ValidSlotIsKept) {
  EXPECT_EQ(feedAssignSlot(7, 8, 0, 40), 7);
}

TEST(FeedSlots, SlotHoldingARealChannelIsReassigned) {
  uint64_t occupied = (1ull << 39) | 1ull;   // a channel at 39 and at 0
  EXPECT_EQ(feedAssignSlot(39, 38, occupied, 40), 37);
}

TEST(FeedSlots, SameSlotAsTheOtherFeedIsReassigned) {
  EXPECT_EQ(feedAssignSlot(5, 5, 0, 40), 39);
}

TEST(FeedSlots, OutOfRangeSlotIsReassigned) {
  EXPECT_EQ(feedAssignSlot(60, 0xFF, 0, 40), 39);
}

TEST(FeedSlots, FullTableHasNoSlot) {
  EXPECT_EQ(feedAssignSlot(0xFF, 0xFF, 0xFull, 4), 0xFF);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
