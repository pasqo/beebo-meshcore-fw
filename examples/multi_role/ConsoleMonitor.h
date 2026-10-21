#pragma once

// `monitor status|start|stop|capture` on the text console. Arduino-free so fw/test/ can build
// it. The kind bits repeat MonRing.h's MON_CAP_* (Beebo.cpp static_asserts they match).

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

struct ConsoleMonitorKind { const char* name; uint32_t bit; };
static const ConsoleMonitorKind CONSOLE_MONITOR_KINDS[] = {
  {"rx", 0x01}, {"tx", 0x02}, {"radio", 0x04}, {"env", 0x08}, {"batt", 0x10},
  {"tuning", 0x20}, {"event", 0x40}, {"path", 0x100},
};
static const uint32_t CONSOLE_MONITOR_ENABLED = 0x80;

enum ConsoleMonitorParse { CONSOLE_MONITOR_SHOW, CONSOLE_MONITOR_SET, CONSOLE_MONITOR_BAD };

// "capture on  rx on  tx on  ..." for a persisted capture config.
inline void consoleMonitorCapture(uint32_t cfg, char* out, size_t cap) {
  size_t n = snprintf(out, cap, "capture %s", (cfg & CONSOLE_MONITOR_ENABLED) ? "on" : "off");
  for (const ConsoleMonitorKind& k : CONSOLE_MONITOR_KINDS) {
    if (n >= cap) return;
    n += snprintf(out + n, cap - n, "  %s %s", k.name, (cfg & k.bit) ? "on" : "off");
  }
}

// Arguments of `monitor capture`: nothing shows the config, "<kind> on|off" sets one bit.
inline ConsoleMonitorParse consoleMonitorParse(const char* args, uint32_t* mask, bool* on) {
  while (*args == ' ') args++;
  if (!*args) return CONSOLE_MONITOR_SHOW;
  const char* kind = args;
  size_t kn = 0;
  while (kind[kn] && kind[kn] != ' ') kn++;
  const char* value = kind + kn;
  while (*value == ' ') value++;
  size_t vn = 0;
  while (value[vn] && value[vn] != ' ') vn++;
  const char* rest = value + vn;
  while (*rest == ' ') rest++;
  if (!vn || *rest) return CONSOLE_MONITOR_BAD;
  if (vn == 2 && strncmp(value, "on", 2) == 0) *on = true;
  else if (vn == 3 && strncmp(value, "off", 3) == 0) *on = false;
  else if (vn == 1 && value[0] == '1') *on = true;
  else if (vn == 1 && value[0] == '0') *on = false;
  else return CONSOLE_MONITOR_BAD;
  for (const ConsoleMonitorKind& k : CONSOLE_MONITOR_KINDS) {
    if (strlen(k.name) == kn && strncmp(k.name, kind, kn) == 0) { *mask = k.bit; return CONSOLE_MONITOR_SET; }
  }
  return CONSOLE_MONITOR_BAD;
}

inline uint32_t consoleMonitorApply(uint32_t cfg, uint32_t bit, bool on) { return on ? (cfg | bit) : (cfg & ~bit); }

// Busy shares are 0-10000 (two decimals of a percent); idle likewise.
inline void consoleMonitorTime(unsigned rx, unsigned tx, unsigned lx, unsigned idle, unsigned loops, char* out, size_t cap) {
  snprintf(out, cap, "time rx %u.%02u%%  tx %u.%02u%%  lx %u.%02u%%  idle %u.%02u%%  loops %u/s", rx / 100, rx % 100,
           tx / 100, tx % 100, lx / 100, lx % 100, idle / 100, idle % 100, loops);
}

inline void consoleMonitorRing(uint32_t count, uint32_t capacity, char* out, size_t cap) {
  snprintf(out, cap, "ring %u/%u", (unsigned)count, (unsigned)capacity);
}

// `monitor tuning trial`: the state in one line. `idx` is the running block's zero-based index.
inline void consoleTrialLine(bool enabled, bool running, bool waiting, const char* sw, unsigned a, unsigned b,
                             unsigned idx, unsigned total, const char* profile, char* out, size_t cap) {
  if (!enabled) snprintf(out, cap, "trial off  profile %s", profile);
  else if (running) snprintf(out, cap, "trial running %s a=%u b=%u block %u/%u  profile %s", sw ? sw : "?", a, b, idx + 1, total, profile);
  else if (waiting) snprintf(out, cap, "trial waiting for traffic  profile %s", profile);
  else snprintf(out, cap, "trial on  profile %s", profile);
}
