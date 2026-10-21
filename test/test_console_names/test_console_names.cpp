#include <gtest/gtest.h>
#include <cstring>
#include <ConsoleNames.h>

namespace {

TEST(ConsoleNames, CliFormBecomesTheTextCommand) {
  char out[64];
  ASSERT_TRUE(consoleNameRewrite("neighbors forget 0a1b", out, sizeof(out)));
  EXPECT_STREQ("neighbor.remove 0a1b", out);
  ASSERT_TRUE(consoleNameRewrite("region allow pytest_home", out, sizeof(out)));
  EXPECT_STREQ("region allowf pytest_home", out);
  ASSERT_TRUE(consoleNameRewrite("region deny pytest_home", out, sizeof(out)));
  EXPECT_STREQ("region denyf pytest_home", out);
}

TEST(ConsoleNames, WithoutArguments) {
  char out[64];
  ASSERT_TRUE(consoleNameRewrite("neighbors peek", out, sizeof(out)));
  EXPECT_STREQ("neighbors", out);
  ASSERT_TRUE(consoleNameRewrite("region tree", out, sizeof(out)));
  EXPECT_STREQ("region", out);
  ASSERT_TRUE(consoleNameRewrite("neighbors discover", out, sizeof(out)));
  EXPECT_STREQ("discover.neighbors", out);
  ASSERT_TRUE(consoleNameRewrite("advert --flood", out, sizeof(out)));
  EXPECT_STREQ("advert", out);
}

TEST(ConsoleNames, LeadingAndRepeatedSpaces) {
  char out[64];
  ASSERT_TRUE(consoleNameRewrite("  region   allow   x ", out, sizeof(out)));
  EXPECT_STREQ("region allowf x", out);
}

TEST(ConsoleNames, OtherTextIsLeftAlone) {
  char out[64] = "untouched";
  EXPECT_FALSE(consoleNameRewrite("neighbors", out, sizeof(out)));        // the text command itself
  EXPECT_FALSE(consoleNameRewrite("neighbors forgetful", out, sizeof(out)));   // longer word
  EXPECT_FALSE(consoleNameRewrite("region allowf x", out, sizeof(out)));
  EXPECT_FALSE(consoleNameRewrite("advert", out, sizeof(out)));
  EXPECT_FALSE(consoleNameRewrite("", out, sizeof(out)));
  EXPECT_STREQ("untouched", out);
}

TEST(ConsoleNames, ResultThatDoesNotFitIsNotRewritten) {
  char out[8] = "keep";
  EXPECT_FALSE(consoleNameRewrite("neighbors forget 0a1b2c3d4e5f", out, sizeof(out)));
  EXPECT_STREQ("keep", out);
}

TEST(ConsoleNames, TableHasNoDuplicates) {
  for (size_t i = 0; i < CONSOLE_NAME_COUNT; i++)
    for (size_t j = i + 1; j < CONSOLE_NAME_COUNT; j++) EXPECT_STRNE(CONSOLE_NAMES[i].cli, CONSOLE_NAMES[j].cli);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
