#include <gtest/gtest.h>
#include <cstring>
#include <ConsoleAlias.h>

namespace {

TEST(ConsoleAlias, BareNameBecomesGet) {
  char out[64];
  ASSERT_TRUE(consoleAliasRewrite("board.node.role", out, sizeof(out)));
  EXPECT_STREQ("get node.role", out);
}

TEST(ConsoleAlias, NameWithValueBecomesSet) {
  char out[64];
  ASSERT_TRUE(consoleAliasRewrite("repeater.routing.tuning.trial.profile exhaustive", out, sizeof(out)));
  EXPECT_STREQ("set tuning.trial.profile exhaustive", out);
}

TEST(ConsoleAlias, RoleprefixIsOptional) {
  char a[64], b[64];
  ASSERT_TRUE(consoleAliasRewrite("routing.tuning.trial.enabled on", a, sizeof(a)));
  ASSERT_TRUE(consoleAliasRewrite("repeater.routing.tuning.trial.enabled on", b, sizeof(b)));
  EXPECT_STREQ("set tuning.trial.enabled on", a);
  EXPECT_STREQ(a, b);
}

TEST(ConsoleAlias, ExtraSpacesAroundTheValueAreTrimmed) {
  char out[64];
  ASSERT_TRUE(consoleAliasRewrite("routing.tuning.trial.enabled   on  ", out, sizeof(out)));
  EXPECT_STREQ("set tuning.trial.enabled on", out);
}

TEST(ConsoleAlias, UnknownNamesAndPlainCommandsAreLeftAlone) {
  char out[64] = "untouched";
  EXPECT_FALSE(consoleAliasRewrite("get node.role", out, sizeof(out)));
  EXPECT_FALSE(consoleAliasRewrite("ver", out, sizeof(out)));
  EXPECT_FALSE(consoleAliasRewrite("board.node.rolex", out, sizeof(out)));   // longer name, no match
  EXPECT_FALSE(consoleAliasRewrite("board.node", out, sizeof(out)));         // a prefix of one
  EXPECT_FALSE(consoleAliasRewrite("", out, sizeof(out)));
  EXPECT_STREQ("untouched", out);
}

TEST(ConsoleAlias, ResultThatDoesNotFitIsNotRewritten) {
  char out[8] = "keep";
  EXPECT_FALSE(consoleAliasRewrite("board.node.role repeater", out, sizeof(out)));
  EXPECT_STREQ("keep", out);
}

TEST(ConsoleAlias, TrialSettingsAreRepeaterOnlyAndRoleIsNot) {
  char out[64];
  bool ro = false;
  ASSERT_TRUE(consoleAliasRewrite("routing.tuning.trial.profile", out, sizeof(out), &ro));
  EXPECT_TRUE(ro);
  ASSERT_TRUE(consoleAliasRewrite("board.node.role", out, sizeof(out), &ro));
  EXPECT_FALSE(ro);
}

TEST(ConsoleAlias, TableHasNoDuplicatePaths) {
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
