#include <gtest/gtest.h>
#include <cstring>
#include <CliCommand.h>

namespace {

TEST(CliCommand, PlainCommandHasNoPrefix) {
  char text[] = "get name";
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "get name");
  EXPECT_STREQ(prefix, "");
}

TEST(CliCommand, PrefixIsSplitOff) {
  char text[] = "ab|get name";
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "get name");
  EXPECT_STREQ(prefix, "ab|");
}

TEST(CliCommand, LeadingSpacesAreSkippedBeforeThePrefix) {
  char text[] = "  ab|ver";
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "ver");
  EXPECT_STREQ(prefix, "ab|");
}

TEST(CliCommand, ShortTextIsNeverAPrefix) {
  char text[] = "ab|v";   // only 4 characters: not longer than 4
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "ab|v");
  EXPECT_STREQ(prefix, "");
}

TEST(CliCommand, PipeElsewhereIsNotAPrefix) {
  char text[] = "get|name";
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "get|name");
  EXPECT_STREQ(prefix, "");
}

TEST(CliCommand, EmptyText) {
  char text[] = "";
  char prefix[4];
  EXPECT_STREQ(splitCliPrefix(text, prefix), "");
  EXPECT_STREQ(prefix, "");
}

TEST(CliCommand, ReplyLenIsTheStringLengthWhenItFits) {
  EXPECT_EQ(cliReplyLen("OK", 16, 100), 2u);
  EXPECT_EQ(cliReplyLen("", 16, 100), 0u);
}

TEST(CliCommand, ReplyLenIsClampedToTheFrameRoom) {
  EXPECT_EQ(cliReplyLen("0123456789", 16, 4), 4u);
}

TEST(CliCommand, ReplyLenNeverReadsPastTheBuffer) {
  const char unterminated[4] = {'a', 'b', 'c', 'd'};
  EXPECT_EQ(cliReplyLen(unterminated, sizeof(unterminated), 100), 4u);
}

TEST(CliCommand, OnOffAcceptsOnlyThoseTwoWords) {
  bool on = false;
  EXPECT_TRUE(cliOnOff("on", &on));
  EXPECT_TRUE(on);
  EXPECT_TRUE(cliOnOff("off  ", &on));
  EXPECT_FALSE(on);
  on = true;
  EXPECT_FALSE(cliOnOff("", &on));
  EXPECT_FALSE(cliOnOff("onn", &on));
  EXPECT_FALSE(cliOnOff("of", &on));
  EXPECT_FALSE(cliOnOff("1", &on));
  EXPECT_FALSE(cliOnOff("On", &on));
  EXPECT_TRUE(on);   // untouched on a miss
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
