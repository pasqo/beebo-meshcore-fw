#pragma once

// `status`, `status node`, `status radio` and `status resources` on the text console: one
// compact line per section. Arduino-free so fw/test/ can build it; Beebo fills the struct
// from the same state the binary STATS_* replies read.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

struct ConsoleStatus {
  const char* name;
  const char* role;
  const char* version;
  uint32_t uptime_secs;
  uint16_t battery_mv;
  int16_t mcu_temp_x10;
  uint16_t err_flags;
  bool quiet;                  // the radio is asleep (board.state.quiet)
  int16_t noise_floor;
  int8_t last_rssi;
  int8_t last_snr_x4;
  uint32_t tx_air_secs, rx_air_secs;
  uint32_t free_heap, free_psram, flash_size;
  uint16_t contacts, pending_msgs;
  bool ring_enabled;
  uint32_t ring_count, ring_cap;
};

// "2d 3h 11m 5s": a zero day, hour or minute is left out, seconds always show.
inline void consoleUptime(uint32_t secs, char* out, size_t cap) {
  uint32_t d = secs / 86400, h = secs % 86400 / 3600, m = secs % 3600 / 60, s = secs % 60;
  size_t n = 0;
  out[0] = 0;
  if (d) n += snprintf(out + n, cap - n, "%ud ", (unsigned)d);
  if (h && n < cap) n += snprintf(out + n, cap - n, "%uh ", (unsigned)h);
  if (m && n < cap) n += snprintf(out + n, cap - n, "%um ", (unsigned)m);
  if (n < cap) snprintf(out + n, cap - n, "%us", (unsigned)s);
}

// Fault bit names, as the CLI's ERR_FLAGS; a bit this build does not know shows as hex.
inline void consoleFaults(uint16_t flags, char* out, size_t cap) {
  static const char* const names[] = {"pool-full", "tx-cad-timeout", "rx-start-timeout"};
  size_t n = 0;
  out[0] = 0;
  if (!flags) { snprintf(out, cap, "none"); return; }
  for (unsigned b = 0; b < 3; b++) {
    if (flags & (1u << b)) n += snprintf(out + n, n < cap ? cap - n : 0, "%s%s", n ? "|" : "", names[b]);
  }
  uint16_t unknown = flags & ~7u;
  if (unknown && n < cap) snprintf(out + n, cap - n, "%s0x%x", n ? "|" : "", (unsigned)unknown);
}

// Tenths as a signed decimal: -25 -> "-2.5".
inline int consoleTenths(char* out, size_t cap, int tenths) {
  return snprintf(out, cap, "%s%d.%d", tenths < 0 ? "-" : "", (tenths < 0 ? -tenths : tenths) / 10, (tenths < 0 ? -tenths : tenths) % 10);
}

inline void consoleStatusNode(const ConsoleStatus& s, char* out, size_t cap) {
  char up[40], faults[64], temp[16];
  consoleUptime(s.uptime_secs, up, sizeof(up));
  consoleFaults(s.err_flags, faults, sizeof(faults));
  consoleTenths(temp, sizeof(temp), s.mcu_temp_x10);
  snprintf(out, cap, "%s %s %s up %s pwr %u.%02u V %s C faults %s%s", s.name && s.name[0] ? s.name : "unknown",
           s.role, s.version, up, (unsigned)(s.battery_mv / 1000), (unsigned)(s.battery_mv % 1000 / 10), temp, faults,
           s.quiet ? " asleep" : "");
}

inline void consoleStatusRadio(const ConsoleStatus& s, char* out, size_t cap) {
  if (s.quiet) {
    snprintf(out, cap, "asleep  tx %u s  rx %u s", (unsigned)s.tx_air_secs, (unsigned)s.rx_air_secs);
    return;
  }
  char snr[16];
  int t = s.last_snr_x4 * 10 / 4;   // tenths of a dB, truncated toward zero
  consoleTenths(snr, sizeof(snr), t);
  snprintf(out, cap, "noise %d dBm  rssi %d dBm  snr %s dB  tx %u s  rx %u s", (int)s.noise_floor, (int)s.last_rssi, snr,
           (unsigned)s.tx_air_secs, (unsigned)s.rx_air_secs);
}

inline void consoleStatusResources(const ConsoleStatus& s, char* out, size_t cap) {
  snprintf(out, cap, "heap %u kB  psram %u kB  flash %u MB  contacts %u  msgs %u  ring %u/%u %s",
           (unsigned)(s.free_heap / 1024), (unsigned)(s.free_psram / 1024), (unsigned)(s.flash_size / (1024 * 1024)),
           (unsigned)s.contacts, (unsigned)s.pending_msgs, (unsigned)s.ring_count, (unsigned)s.ring_cap,
           s.ring_enabled ? "capturing" : "paused");
}

// The three lines, newline separated.
inline void consoleStatusAll(const ConsoleStatus& s, char* out, size_t cap) {
  size_t n = 0;
  consoleStatusNode(s, out, cap);
  n = strlen(out);
  if (n + 1 < cap) { out[n++] = '\n'; consoleStatusRadio(s, out + n, cap - n); }
  n = strlen(out);
  if (n + 1 < cap) { out[n++] = '\n'; consoleStatusResources(s, out + n, cap - n); }
}
