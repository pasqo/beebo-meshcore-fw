#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <string>
#include <ConsoleStatus.h>

namespace {

ConsoleStatus sample() {
  ConsoleStatus s{};
  s.name = "bench";
  s.role = "repeater";
  s.version = "1.18.0.beebo.1.0-rc16";
  s.uptime_secs = 2 * 86400 + 3 * 3600 + 11 * 60 + 5;
  s.battery_mv = 4190;
  s.mcu_temp_x10 = 312;
  s.err_flags = 0;
  s.noise_floor = -108;
  s.last_rssi = -91;
  s.last_snr_x4 = 30;   // 7.5 dB
  s.tx_air_secs = 412;
  s.rx_air_secs = 5230;
  s.free_heap = 142 * 1024;
  s.free_psram = 6012 * 1024;
  s.flash_size = 16u * 1024 * 1024;
  s.contacts = 0;
  s.pending_msgs = 0;
  s.ring_enabled = true;
  s.ring_count = 4120;
  s.ring_cap = 65536;
  return s;
}

TEST(ConsoleStatus, NodeLine) {
  char out[200];
  consoleStatusNode(sample(), out, sizeof(out));
  EXPECT_STREQ("bench repeater 1.18.0.beebo.1.0-rc16 up 2d 3h 11m 5s pwr 4.19 V 31.2 C faults none", out);
}

TEST(ConsoleStatus, UptimeSkipsZeroUnits) {
  char out[64];
  EXPECT_EQ(std::string("45s"), (consoleUptime(45, out, sizeof(out)), out));
  EXPECT_EQ(std::string("1m 0s"), (consoleUptime(60, out, sizeof(out)), out));
  EXPECT_EQ(std::string("1d 0s"), (consoleUptime(86400, out, sizeof(out)), out));
  EXPECT_EQ(std::string("1h 1m 1s"), (consoleUptime(3661, out, sizeof(out)), out));
}

TEST(ConsoleStatus, FaultsAreNamed) {
  char out[200];
  ConsoleStatus s = sample();
  s.err_flags = 0x1 | 0x4;
  consoleStatusNode(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "faults pool-full|rx-start-timeout"));
  s.err_flags = 0x2 | 0x40;
  consoleStatusNode(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "faults tx-cad-timeout|0x40"));
}

TEST(ConsoleStatus, NegativeTemperatureAndUnknownName) {
  char out[200];
  ConsoleStatus s = sample();
  s.mcu_temp_x10 = -25;
  s.name = "";
  consoleStatusNode(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "pwr 4.19 V -2.5 C"));
  EXPECT_EQ(0u, strncmp(out, "unknown repeater", 16));
}

TEST(ConsoleStatus, AsleepNodeSaysSo) {
  char out[200];
  ConsoleStatus s = sample();
  s.quiet = true;
  consoleStatusNode(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, " asleep"));
}

TEST(ConsoleStatus, RadioLine) {
  char out[200];
  consoleStatusRadio(sample(), out, sizeof(out));
  EXPECT_STREQ("noise -108 dBm  rssi -91 dBm  snr 7.5 dB  tx 412 s  rx 5230 s", out);
}

TEST(ConsoleStatus, NegativeSnr) {
  char out[200];
  ConsoleStatus s = sample();
  s.last_snr_x4 = -10;   // -2.5 dB
  consoleStatusRadio(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "snr -2.5 dB"));
}

TEST(ConsoleStatus, AsleepRadioSkipsTheLiveReads) {
  char out[200];
  ConsoleStatus s = sample();
  s.quiet = true;
  consoleStatusRadio(s, out, sizeof(out));
  EXPECT_STREQ("asleep  tx 412 s  rx 5230 s", out);
}

TEST(ConsoleStatus, ResourcesLine) {
  char out[200];
  consoleStatusResources(sample(), out, sizeof(out));
  EXPECT_STREQ("heap 142 kB  psram 6012 kB  flash 16 MB  contacts 0  msgs 0  ring 4120/65536 capturing", out);
}

TEST(ConsoleStatus, PausedRing) {
  char out[200];
  ConsoleStatus s = sample();
  s.ring_enabled = false;
  consoleStatusResources(s, out, sizeof(out));
  EXPECT_NE(nullptr, strstr(out, "ring 4120/65536 paused"));
}

TEST(ConsoleStatus, AllThreeAreJoinedByNewlines) {
  char out[600];
  consoleStatusAll(sample(), out, sizeof(out));
  EXPECT_EQ(2, (int)std::count(out, out + strlen(out), '\n'));
  EXPECT_EQ(0u, strncmp(out, "bench repeater", 14));
}

TEST(ConsoleStatus, ShortBufferIsCutNotOverrun) {
  char out[20];
  consoleStatusNode(sample(), out, sizeof(out));
  EXPECT_LT(strlen(out), sizeof(out));
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
