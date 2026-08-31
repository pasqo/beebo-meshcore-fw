#include <gtest/gtest.h>
#include <cstring>
#include <AdminSelfCommand.h>

namespace {

TEST(AdminSelfCommand, BareSelfMatchesEmptyCommand) {
  char text[] = "self";
  char* rest = matchAdminSelfCommand(text);
  ASSERT_NE(rest, nullptr);
  EXPECT_STREQ(rest, "");
}

TEST(AdminSelfCommand, SelfWithCommandMatchesRemainder) {
  char text[] = "self get name";
  char* rest = matchAdminSelfCommand(text);
  ASSERT_NE(rest, nullptr);
  EXPECT_STREQ(rest, "get name");
}

TEST(AdminSelfCommand, CaseSensitiveRejectsWrongCase) {
  char text[] = "SeLf get name";
  EXPECT_EQ(matchAdminSelfCommand(text), nullptr);
}

TEST(AdminSelfCommand, PrefixCollisionDoesNotMatch) {
  char text[] = "selfish get name"; // must not match on "self" prefix alone
  EXPECT_EQ(matchAdminSelfCommand(text), nullptr);
}

TEST(AdminSelfCommand, UnrelatedCommandDoesNotMatch) {
  char text[] = "get name";
  EXPECT_EQ(matchAdminSelfCommand(text), nullptr);
}

TEST(AdminSelfCommand, EmptyStringDoesNotMatch) {
  char text[] = "";
  EXPECT_EQ(matchAdminSelfCommand(text), nullptr);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
