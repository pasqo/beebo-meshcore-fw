#include <gtest/gtest.h>
#include <cstring>
#include <ConsoleAlias.h>

namespace {

// `settings <path> [value]`: the node's console takes the CLI's own command.

TEST(ConsoleSettings, PathBecomesGet) {
  char out[64];
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN, consoleSettings("settings board.node.role", out, sizeof(out)));
  EXPECT_STREQ("get node.role", out);
}

TEST(ConsoleSettings, PathWithValueBecomesSet) {
  char out[64];
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN,
            consoleSettings("settings repeater.routing.tuning.trial.profile exhaustive", out, sizeof(out)));
  EXPECT_STREQ("set tuning.trial.profile exhaustive", out);
}

TEST(ConsoleSettings, RolePrefixIsOptional) {
  char a[64], b[64];
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN, consoleSettings("settings routing.tuning.trial.enabled on", a, sizeof(a)));
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN, consoleSettings("settings repeater.routing.tuning.trial.enabled on", b, sizeof(b)));
  EXPECT_STREQ("set tuning.trial.enabled on", a);
  EXPECT_STREQ(a, b);
}

TEST(ConsoleSettings, ExtraSpacesAreTrimmed) {
  char out[64];
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN,
            consoleSettings("  settings   routing.tuning.trial.enabled   on  ", out, sizeof(out)));
  EXPECT_STREQ("set tuning.trial.enabled on", out);
}

TEST(ConsoleSettings, OtherCommandsAreNotSettings) {
  char out[64] = "untouched";
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("get node.role", out, sizeof(out)));
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("ver", out, sizeof(out)));
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("settingsx board.node.role", out, sizeof(out)));
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("", out, sizeof(out)));
  EXPECT_STREQ("untouched", out);
}

TEST(ConsoleSettings, ABarePathIsNoLongerACommand) {
  char out[64] = "untouched";
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("board.node.role", out, sizeof(out)));
  EXPECT_EQ(CONSOLE_NOT_SETTINGS, consoleSettings("radio.txpower 18", out, sizeof(out)));
}

TEST(ConsoleSettings, UnknownPathIsRefused) {
  char out[64] = "untouched";
  EXPECT_EQ(CONSOLE_SETTINGS_UNKNOWN, consoleSettings("settings board.node.rolex", out, sizeof(out)));
  EXPECT_EQ(CONSOLE_SETTINGS_UNKNOWN, consoleSettings("settings nosuch", out, sizeof(out)));
  EXPECT_STREQ("untouched", out);
}

TEST(ConsoleSettings, ResultThatDoesNotFitIsRefused) {
  char out[8] = "keep";
  EXPECT_EQ(CONSOLE_SETTINGS_TOO_LONG, consoleSettings("settings board.node.role repeater", out, sizeof(out)));
  EXPECT_STREQ("keep", out);
}

TEST(ConsoleSettings, RepeaterOnlyIsReported) {
  char out[64];
  bool ro = false;
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN, consoleSettings("settings routing.tuning.trial.profile", out, sizeof(out), &ro));
  EXPECT_TRUE(ro);
  ASSERT_EQ(CONSOLE_SETTINGS_REWRITTEN, consoleSettings("settings board.node.role", out, sizeof(out), &ro));
  EXPECT_FALSE(ro);
}

TEST(ConsoleSettings, NoPathListsTheTopLevel) {
  char out[256];
  ASSERT_EQ(CONSOLE_SETTINGS_LISTED, consoleSettings("settings", out, sizeof(out)));
  EXPECT_NE(nullptr, strstr(out, "board."));
  EXPECT_NE(nullptr, strstr(out, "radio."));
  EXPECT_NE(nullptr, strstr(out, "routing."));
  EXPECT_EQ(nullptr, strstr(out, "tuning"));   // only the first segment
}

TEST(ConsoleSettings, ABranchListsItsChildren) {
  char out[512];
  ASSERT_EQ(CONSOLE_SETTINGS_LISTED, consoleSettings("settings routing.tuning.trial", out, sizeof(out)));
  EXPECT_NE(nullptr, strstr(out, "enabled"));
  EXPECT_NE(nullptr, strstr(out, "profile"));
  EXPECT_EQ(nullptr, strstr(out, "routing"));
  ASSERT_EQ(CONSOLE_SETTINGS_LISTED, consoleSettings("settings repeater.board", out, sizeof(out)));   // role prefix on a branch
}

TEST(ConsoleSettings, EachNameIsListedOnce) {
  char out[512];
  ASSERT_EQ(CONSOLE_SETTINGS_LISTED, consoleSettings("settings routing.tuning.trial", out, sizeof(out)));
  const char* first = strstr(out, "enabled");
  ASSERT_NE(nullptr, first);
  EXPECT_EQ(nullptr, strstr(first + 1, "enabled"));
}

TEST(ConsoleSettings, ABranchTakesNoValue) {
  char out[64] = "untouched";
  EXPECT_EQ(CONSOLE_SETTINGS_UNKNOWN, consoleSettings("settings routing.tuning.trial on", out, sizeof(out)));
}

TEST(ConsoleSettings, ALongListingIsCutWithDots) {
  char out[24];
  ASSERT_EQ(CONSOLE_SETTINGS_LISTED, consoleSettings("settings routing.tuning.trial", out, sizeof(out)));
  size_t n = strlen(out);
  ASSERT_LT(n, sizeof(out));
  EXPECT_STREQ("...", out + n - 3);
}

TEST(ConsoleSettings, TableHasNoDuplicatePaths) {
  for (size_t i = 0; i < CONSOLE_ALIAS_COUNT; i++) {
    for (size_t j = i + 1; j < CONSOLE_ALIAS_COUNT; j++) {
      EXPECT_STRNE(CONSOLE_ALIASES[i].path, CONSOLE_ALIASES[j].path);
    }
  }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
