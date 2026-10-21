#include <gtest/gtest.h>
#include <cstring>
#include <ConsoleBattery.h>

namespace {

TEST(ConsoleBattery, StateLine) {
  char out[100];
  consoleBatteryState(2, 4190, 4, out, sizeof(out));
  EXPECT_STREQ("present yes  4.190 V  state CHARGED", out);
  consoleBatteryState(1, 5012, 5, out, sizeof(out));
  EXPECT_STREQ("present no  5.012 V  state PLUGGED", out);
  consoleBatteryState(0, 3987, 1, out, sizeof(out));
  EXPECT_STREQ("present unknown  3.987 V  state STABLE", out);
}

TEST(ConsoleBattery, StateNames) {
  EXPECT_STREQ("UNKNOWN", consoleBatteryStateName(0));
  EXPECT_STREQ("STABLE", consoleBatteryStateName(1));
  EXPECT_STREQ("CHARGING", consoleBatteryStateName(2));
  EXPECT_STREQ("DISCHARGING", consoleBatteryStateName(3));
  EXPECT_STREQ("CHARGED", consoleBatteryStateName(4));
  EXPECT_STREQ("PLUGGED", consoleBatteryStateName(5));
  EXPECT_STREQ("UNKNOWN", consoleBatteryStateName(99));
}

TEST(ConsoleBattery, ParseVolts) {
  float v = 0;
  EXPECT_TRUE(consoleParseVolts("4.18", &v));
  EXPECT_FLOAT_EQ(4.18f, v);
  EXPECT_TRUE(consoleParseVolts("  4.2  ", &v));
  EXPECT_FLOAT_EQ(4.2f, v);
  EXPECT_TRUE(consoleParseVolts("4", &v));
  EXPECT_FALSE(consoleParseVolts("", &v));
  EXPECT_FALSE(consoleParseVolts("abc", &v));
  EXPECT_FALSE(consoleParseVolts("4.2V", &v));
  EXPECT_FALSE(consoleParseVolts("4.2 5", &v));
}

TEST(ConsoleBattery, CalibrateScalesTheMultiplier) {
  float out = 0;
  ASSERT_EQ(CONSOLE_CAL_OK, consoleCalibrate(2.0f, 4000, 4.2f, &out));
  EXPECT_NEAR(2.1f, out, 1e-4);   // reported 4.0 V, measured 4.2 V
  ASSERT_EQ(CONSOLE_CAL_OK, consoleCalibrate(2.0f, 4200, 4.2f, &out));
  EXPECT_NEAR(2.0f, out, 1e-4);
}

TEST(ConsoleBattery, CalibrateRefusesBadInput) {
  float out = 123;
  EXPECT_EQ(CONSOLE_CAL_NO_READING, consoleCalibrate(2.0f, 0, 4.2f, &out));
  EXPECT_EQ(CONSOLE_CAL_VOLTS_RANGE, consoleCalibrate(2.0f, 4000, 1.0f, &out));
  EXPECT_EQ(CONSOLE_CAL_VOLTS_RANGE, consoleCalibrate(2.0f, 4000, 5.5f, &out));
  EXPECT_EQ(CONSOLE_CAL_MULT_RANGE, consoleCalibrate(9.5f, 2000, 4.2f, &out));   // x2.1 would pass 10
  EXPECT_EQ(CONSOLE_CAL_MULT_RANGE, consoleCalibrate(0.0f, 4000, 4.2f, &out));   // a zero multiplier cannot scale
  EXPECT_FLOAT_EQ(123, out);
}

TEST(ConsoleBattery, CalibrateReply) {
  char out[100];
  consoleCalibrateReply(2.0f, 2.1f, out, sizeof(out));
  EXPECT_STREQ("mult 2.0000 -> 2.1000", out);
}

TEST(ConsoleBattery, ForceWords) {
  uint8_t st = 99;
  ASSERT_TRUE(consoleBatteryForceWord("unknown", &st));
  EXPECT_EQ(0, st);
  ASSERT_TRUE(consoleBatteryForceWord("charging", &st));
  EXPECT_EQ(2, st);
  ASSERT_TRUE(consoleBatteryForceWord("discharging", &st));
  EXPECT_EQ(3, st);
  ASSERT_TRUE(consoleBatteryForceWord("charged", &st));
  EXPECT_EQ(4, st);
  ASSERT_TRUE(consoleBatteryForceWord("plugged", &st));
  EXPECT_EQ(5, st);
  ASSERT_TRUE(consoleBatteryForceWord("  charged  ", &st));
  EXPECT_EQ(4, st);
}

TEST(ConsoleBattery, ForceWordsRefuseTheRest) {
  uint8_t st = 99;
  EXPECT_FALSE(consoleBatteryForceWord("stable", &st));      // STABLE is a result of the trend, never forced
  EXPECT_FALSE(consoleBatteryForceWord("CHARGED", &st));
  EXPECT_FALSE(consoleBatteryForceWord("charge", &st));
  EXPECT_FALSE(consoleBatteryForceWord("", &st));
  EXPECT_FALSE(consoleBatteryForceWord("charged now", &st));
  EXPECT_EQ(99, st);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
