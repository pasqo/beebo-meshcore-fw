#include <gtest/gtest.h>
#include <cstring>
#include <AppSync.h>

namespace {

TEST(AppSync, EmptyContactsStartIsCodeAndZeroCount) {
  uint8_t buf[8];
  memset(buf, 0xEE, sizeof(buf));
  ASSERT_EQ(appSyncContactsStart(buf), 5u);
  const uint8_t want[5] = {APP_SYNC_RESP_CONTACTS_START, 0, 0, 0, 0};
  EXPECT_EQ(memcmp(buf, want, 5), 0);
  EXPECT_EQ(buf[5], 0xEE);   // nothing written past the frame
}

TEST(AppSync, EmptyContactsEndIsCodeAndZeroLastmod) {
  uint8_t buf[8];
  memset(buf, 0xEE, sizeof(buf));
  ASSERT_EQ(appSyncContactsEnd(buf), 5u);
  const uint8_t want[5] = {APP_SYNC_RESP_END_OF_CONTACTS, 0, 0, 0, 0};
  EXPECT_EQ(memcmp(buf, want, 5), 0);
  EXPECT_EQ(buf[5], 0xEE);
}

TEST(AppSync, NoMoreMessagesIsOneByte) {
  uint8_t buf[4];
  memset(buf, 0xEE, sizeof(buf));
  ASSERT_EQ(appSyncNoMoreMessages(buf), 1u);
  EXPECT_EQ(buf[0], APP_SYNC_RESP_NO_MORE_MESSAGES);
  EXPECT_EQ(buf[1], 0xEE);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
