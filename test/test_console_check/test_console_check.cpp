#include <gtest/gtest.h>
#include <cstring>
#include <ConsoleCheck.h>

namespace {

TEST(ConsoleCheck, QosBands) {
  char out[160];
  consoleCheckQos(9500, 20, out, sizeof(out));
  EXPECT_STREQ("QoS 95.0% OK Excellent.", out);
  consoleCheckQos(9499, 20, out, sizeof(out));
  EXPECT_STREQ("QoS 95.0% INFO Good, not excellent.", out);   // the band uses the exact value, the display rounds
  consoleCheckQos(8000, 500, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "INFO"));
  consoleCheckQos(7999, 500, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "WARN"));
  consoleCheckQos(6000, 500, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "WARN"));
  consoleCheckQos(5999, 500, out, sizeof(out));
  EXPECT_STREQ("QoS 60.0% FAIL Poor -- most sends timing out or dropped.", out);
  consoleCheckQos(0, 500, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "FAIL"));
}

TEST(ConsoleCheck, QosIsPendingBelowTheExposure) {
  char out[160];
  consoleCheckQos(0, 19, out, sizeof(out));
  EXPECT_STREQ("QoS 0.0% PENDING Only 19 attempts so far.", out);
  consoleCheckQos(0, 1, out, sizeof(out));
  EXPECT_STREQ("QoS 0.0% PENDING Only 1 attempt so far.", out);
}

TEST(ConsoleCheck, SohBands) {
  char out[160];
  consoleCheckSoh(10000, out, sizeof(out));
  EXPECT_STREQ("SoH 100.0% OK No meaningful fault rate.", out);
  consoleCheckSoh(9900, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, " OK "));
  consoleCheckSoh(9899, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "INFO"));
  consoleCheckSoh(9500, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "INFO"));
  consoleCheckSoh(9499, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "WARN"));
  consoleCheckSoh(8000, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "WARN"));
  consoleCheckSoh(7999, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "FAIL"));
}

TEST(ConsoleCheck, ValuesRoundToOneDecimal) {
  char out[160];
  consoleCheckSoh(9996, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "SoH 100.0%"));
  consoleCheckSoh(8749, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "SoH 87.5%"));
}

TEST(ConsoleCheck, BothLinesJoined) {
  char out[300];
  consoleCheck(8750, 40, 9990, out, sizeof(out));
  EXPECT_STREQ("QoS 87.5% INFO Good, not excellent.\nSoH 99.9% OK No meaningful fault rate.", out);
}

TEST(ConsoleCheck, ShortBufferIsCut) {
  char out[16];
  consoleCheck(8750, 40, 9990, out, sizeof(out));
  EXPECT_LT(strlen(out), sizeof(out));
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
