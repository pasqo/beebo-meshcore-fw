#include <gtest/gtest.h>
#include <helpers/TrialSequence.h>

namespace {

TEST(TrialSequence, NextSwitchFollowsTheRunOrderNotTheBitOrder) {
  EXPECT_EQ(-1, TrialSequence::nextSwitch(0));
  for (int i = 0; i < TrialSequence::NUM_SWITCHES; i++) {
    EXPECT_EQ(TrialSequence::ORDER[i], TrialSequence::nextSwitch(1u << TrialSequence::ORDER[i]));
  }
  uint16_t all = (1u << TrialSequence::NUM_SWITCHES) - 1;
  EXPECT_EQ(TrialSequence::ORDER[0], TrialSequence::nextSwitch(all));
  // the first switch of the order is done: the second is next
  all &= ~(1u << TrialSequence::ORDER[0]);
  EXPECT_EQ(TrialSequence::ORDER[1], TrialSequence::nextSwitch(all));
}

TEST(TrialSequence, ValueRangesPerSwitch) {
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::LNA, 0));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::LNA, 1));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::LNA, 2));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::RX_BOOST, 2));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::CR, 5));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::CR, 8));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::CR, 4));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::CR, 9));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::NUM_SWITCHES, 0));
}

TEST(TrialSequence, DefaultListsHoldOneChallenger) {
  TrialSequence::List l;
  l.reset(TrialSequence::LNA);
  EXPECT_EQ(1, l.n);
  EXPECT_EQ(0, l.current());
  l.reset(TrialSequence::CR);
  EXPECT_EQ(8, l.current());
}

TEST(TrialSequence, SetRejectsEmptyTooLongAndOutOfRange) {
  TrialSequence::List l;
  l.reset(TrialSequence::CR);
  const uint8_t ok[] = {5, 6, 7, 8};
  EXPECT_TRUE(l.set(TrialSequence::CR, ok, 4));
  EXPECT_EQ(4, l.n);
  const uint8_t too_many[] = {5, 6, 7, 8, 5};
  EXPECT_FALSE(l.set(TrialSequence::CR, too_many, 5));
  const uint8_t bad[] = {5, 9};
  EXPECT_FALSE(l.set(TrialSequence::CR, bad, 2));
  EXPECT_FALSE(l.set(TrialSequence::CR, ok, 0));
  EXPECT_EQ(4, l.n);   // a rejected set leaves the list as it was
}

TEST(TrialSequence, AdvanceWalksTheListThenReportsDone) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  EXPECT_EQ(5, l.current());
  EXPECT_TRUE(l.advance());
  EXPECT_EQ(6, l.current());
  EXPECT_TRUE(l.advance());
  EXPECT_EQ(7, l.current());
  EXPECT_FALSE(l.advance());   // last challenger decided
  l.restart();
  EXPECT_EQ(5, l.current());
}

TEST(TrialSequence, ConsecutiveRepeatsCollapse) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 5, 6, 5};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 4));
  EXPECT_EQ(3, l.n);   // 5,6,5
  EXPECT_EQ(5u | (6u << 8) | (5u << 16) | (5u << 24), l.packed());
}

TEST(TrialSequence, SetRestartsAtTheFirstValue) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 2));
  l.advance();
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 2));
  EXPECT_EQ(5, l.current());
}

TEST(TrialSequence, PackedValuesForTheSettingEvent) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  EXPECT_EQ(5u | (6u << 8) | (7u << 16) | (7u << 24), l.packed());   // padded with the last value
  l.reset(TrialSequence::CR);
  EXPECT_EQ(0x08080808u, l.packed());
}

TEST(TrialSequence, ParseAndFormatRoundTrip) {
  uint8_t out[TrialSequence::MAX_VALUES];
  EXPECT_EQ(3, TrialSequence::parse("8,16, 32", out));
  EXPECT_EQ(8, out[0]);
  EXPECT_EQ(16, out[1]);
  EXPECT_EQ(32, out[2]);
  EXPECT_EQ(1, TrialSequence::parse(" 7 ", out));
  EXPECT_EQ(-1, TrialSequence::parse("", out));
  EXPECT_EQ(-1, TrialSequence::parse("8,", out));
  EXPECT_EQ(-1, TrialSequence::parse("8;9", out));
  EXPECT_EQ(-1, TrialSequence::parse("256", out));
  EXPECT_EQ(-1, TrialSequence::parse("1,2,3,4,5", out));
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7, 8};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 4));
  char buf[4 * TrialSequence::MAX_VALUES];
  EXPECT_EQ(7, l.format(buf));
  EXPECT_STREQ("5,6,7,8", buf);
  l.n = 1; l.v[0] = 255;
  l.format(buf);
  EXPECT_STREQ("255", buf);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
