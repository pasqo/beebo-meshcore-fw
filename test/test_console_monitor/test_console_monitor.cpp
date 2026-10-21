#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <ConsoleMonitor.h>

namespace {

TEST(ConsoleMonitor, CaptureLineListsEveryKind) {
  char out[200];
  consoleMonitorCapture(0x80 | 0x01 | 0x02 | 0x04 | 0x10 | 0x40, out, sizeof(out));
  EXPECT_STREQ("capture on  rx on  tx on  radio on  env off  batt on  tuning off  event on  path off", out);
}

TEST(ConsoleMonitor, PausedAndPathMode) {
  char out[200];
  consoleMonitorCapture(0x20 | 0x100, out, sizeof(out));
  EXPECT_STREQ("capture off  rx off  tx off  radio off  env off  batt off  tuning on  event off  path on", out);
}

TEST(ConsoleMonitor, ParseShow) {
  uint32_t mask; bool on;
  EXPECT_EQ(CONSOLE_MONITOR_SHOW, consoleMonitorParse("", &mask, &on));
  EXPECT_EQ(CONSOLE_MONITOR_SHOW, consoleMonitorParse("  ", &mask, &on));
}

TEST(ConsoleMonitor, ParseSet) {
  uint32_t mask = 0; bool on = false;
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("rx on", &mask, &on));
  EXPECT_EQ(0x01u, mask);
  EXPECT_TRUE(on);
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("  tuning   off ", &mask, &on));
  EXPECT_EQ(0x20u, mask);
  EXPECT_FALSE(on);
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("path on", &mask, &on));
  EXPECT_EQ(0x100u, mask);
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("event 1", &mask, &on));
  EXPECT_TRUE(on);
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("env 0", &mask, &on));
  EXPECT_FALSE(on);
}

TEST(ConsoleMonitor, ParseRefusesBadInput) {
  uint32_t mask; bool on;
  EXPECT_EQ(CONSOLE_MONITOR_BAD, consoleMonitorParse("nosuch on", &mask, &on));
  EXPECT_EQ(CONSOLE_MONITOR_BAD, consoleMonitorParse("rx", &mask, &on));
  EXPECT_EQ(CONSOLE_MONITOR_BAD, consoleMonitorParse("rx maybe", &mask, &on));
  EXPECT_EQ(CONSOLE_MONITOR_BAD, consoleMonitorParse("rx on extra", &mask, &on));
  EXPECT_EQ(CONSOLE_MONITOR_BAD, consoleMonitorParse("rxx on", &mask, &on));
}

TEST(ConsoleMonitor, ApplyChangesOnlyTheNamedBit) {
  EXPECT_EQ(0x81u | 0x02u, consoleMonitorApply(0x81, 0x02, true));
  EXPECT_EQ(0x80u, consoleMonitorApply(0x81, 0x01, false));
  EXPECT_EQ(0x180u, consoleMonitorApply(0x80, 0x100, true));
}

TEST(ConsoleMonitor, RingLine) {
  char out[100];
  consoleMonitorRing(4120, 65536, out, sizeof(out));
  EXPECT_STREQ("ring 4120/65536", out);
}

TEST(ConsoleMonitor, TrialStatusLines) {
  char out[160];
  consoleTrialLine(false, false, false, nullptr, 0, 0, 0, 0, "main", out, sizeof(out));
  EXPECT_STREQ("trial off  profile main", out);
  consoleTrialLine(true, false, true, nullptr, 0, 0, 0, 0, "main", out, sizeof(out));
  EXPECT_STREQ("trial waiting for traffic  profile main", out);
  consoleTrialLine(true, false, false, nullptr, 0, 0, 0, 0, "exhaustive", out, sizeof(out));
  EXPECT_STREQ("trial on  profile exhaustive", out);
  consoleTrialLine(true, true, false, "lna", 0, 1, 11, 260, "main", out, sizeof(out));
  EXPECT_STREQ("trial running lna a=0 b=1 block 12/260  profile main", out);
}

TEST(ConsoleMonitor, CountersShowOnlyTheNonZero) {
  const ConsoleCounter c[] = {{"rx_pool_full", 0}, {"tx_cad_timeout", 3}, {"rx_start_timeout", 0}, {"tx_pool_full", 12}};
  char out[160];
  consoleCounters("health", c, 4, out, sizeof(out));
  EXPECT_STREQ("health tx_cad_timeout=3 tx_pool_full=12", out);
}

TEST(ConsoleMonitor, CountersAllZero) {
  const ConsoleCounter c[] = {{"a", 0}, {"b", 0}};
  char out[160];
  consoleCounters("health", c, 2, out, sizeof(out));
  EXPECT_STREQ("health all 0", out);
}

TEST(ConsoleMonitor, CountersAreCutWithDotsWhenTheyDoNotFit) {
  const ConsoleCounter c[] = {{"alpha_counter", 1}, {"beta_counter", 2}, {"gamma_counter", 3}};
  char out[40];
  consoleCounters("service", c, 3, out, sizeof(out));
  EXPECT_LT(strlen(out), sizeof(out));
  EXPECT_STREQ("...", out + strlen(out) - 3);
  EXPECT_EQ(0u, strncmp(out, "service alpha_counter=1", 23));
}

TEST(ConsoleMonitor, RatesLine) {
  ConsoleRates r{120, 30, 5, 40, 9405, 41210, 12, 10, 20, 30};
  char out[200];
  consoleRates(r, out, sizeof(out));
  EXPECT_STREQ("rates rx 1.20%  tx 0.30%  lx 0.05%  sys 0.40%  idle 94.05%  loops 41210/s  max_loop 12 ms  "
               "tx_wait_airtime 0.10%  tx_wait_cad 0.20%  rx_wait_relay 0.30%", out);
}

TEST(ConsoleMonitor, CaptureAcceptsTheCliLeafNames) {
  uint32_t mask = 0; bool on = false;
  ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse("enabled off", &mask, &on));
  EXPECT_EQ(0x80u, mask);
  EXPECT_FALSE(on);
  for (const char* alias : {"setting on", "command on", "route on", "debug on"}) {
    ASSERT_EQ(CONSOLE_MONITOR_SET, consoleMonitorParse(alias, &mask, &on)) << alias;
    EXPECT_EQ(0x40u, mask) << alias;   // the same bit as event
  }
}

static const char* const SW[] = {"lna", "rxboost", "cr", "agc", "interference", "rxdelay", "txdelay",
                                 "directtxdelay", "airtime", "cad", "multiacks", "retry"};

TEST(ConsoleMonitor, TrialSwitchesByName) {
  uint16_t mask = 0;
  char bad[24];
  ASSERT_EQ(CONSOLE_SWITCHES_OK, consoleTrialSwitches("lna cad", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ((1u << 0) | (1u << 9), mask);
  ASSERT_EQ(CONSOLE_SWITCHES_OK, consoleTrialSwitches("  retry ", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ(1u << 11, mask);
  ASSERT_EQ(CONSOLE_SWITCHES_OK, consoleTrialSwitches("lna,cad", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ((1u << 0) | (1u << 9), mask);
}

TEST(ConsoleMonitor, TrialSwitchesAll) {
  uint16_t mask = 0;
  char bad[24];
  ASSERT_EQ(CONSOLE_SWITCHES_OK, consoleTrialSwitches("all", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ((1u << 12) - 1, mask);
}

TEST(ConsoleMonitor, TrialSwitchesNoneAndUnknown) {
  uint16_t mask = 7;
  char bad[24];
  EXPECT_EQ(CONSOLE_SWITCHES_NONE, consoleTrialSwitches("", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ(CONSOLE_SWITCHES_NONE, consoleTrialSwitches("   ", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_EQ(CONSOLE_SWITCHES_BAD, consoleTrialSwitches("lna nosuch", SW, 12, &mask, bad, sizeof(bad)));
  EXPECT_STREQ("nosuch", bad);
  EXPECT_EQ(CONSOLE_SWITCHES_BAD, consoleTrialSwitches("ln", SW, 12, &mask, bad, sizeof(bad)));   // a prefix is not a name
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
