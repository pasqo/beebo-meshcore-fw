#pragma once

// `battery state` and `battery calibrate VOLTS` on the text console. Arduino-free so fw/test/
// can build it. The multiplier is exactly linear in the reported voltage, so one voltmeter
// reading gives the new one: new = current * measured / reported.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static const float CONSOLE_CAL_VOLTS_MIN = 2.0f;    // a single LiPo cell
static const float CONSOLE_CAL_VOLTS_MAX = 5.0f;
static const float CONSOLE_CAL_MULT_MAX = 10.0f;    // Beebo::tlvSetAdcMultiplier's constrain()

enum ConsoleCalResult { CONSOLE_CAL_OK, CONSOLE_CAL_NO_READING, CONSOLE_CAL_VOLTS_RANGE, CONSOLE_CAL_MULT_RANGE };

inline const char* consoleBatteryStateName(uint8_t state) {
  static const char* const names[] = {"UNKNOWN", "STABLE", "CHARGING", "DISCHARGING", "CHARGED", "PLUGGED"};
  return state < 6 ? names[state] : "UNKNOWN";
}

// `present` is BattTrend.h's BATT_PRESENT_*: 0 unknown, 1 no, 2 yes.
inline void consoleBatteryState(uint8_t present, uint16_t mv, uint8_t state, char* out, size_t cap) {
  snprintf(out, cap, "present %s  %u.%03u V  state %s", present == 2 ? "yes" : present == 1 ? "no" : "unknown",
           (unsigned)(mv / 1000), (unsigned)(mv % 1000), consoleBatteryStateName(state));
}

// A plain decimal number, surrounded by spaces at most.
inline bool consoleParseVolts(const char* text, float* v) {
  while (*text == ' ') text++;
  char* end = nullptr;
  float x = strtof(text, &end);
  if (end == text) return false;
  while (*end == ' ') end++;
  if (*end) return false;
  *v = x;
  return true;
}

inline ConsoleCalResult consoleCalibrate(float current_mult, uint16_t reported_mv, float measured_v, float* new_mult) {
  if (reported_mv == 0) return CONSOLE_CAL_NO_READING;
  if (measured_v < CONSOLE_CAL_VOLTS_MIN || measured_v > CONSOLE_CAL_VOLTS_MAX) return CONSOLE_CAL_VOLTS_RANGE;
  float m = current_mult * (measured_v / (reported_mv / 1000.0f));
  if (!(m > 0.0f) || m > CONSOLE_CAL_MULT_MAX) return CONSOLE_CAL_MULT_RANGE;
  *new_mult = m;
  return CONSOLE_CAL_OK;
}

inline void consoleCalibrateReply(float old_mult, float new_mult, char* out, size_t cap) {
  snprintf(out, cap, "mult %.4f -> %.4f", (double)old_mult, (double)new_mult);
}
