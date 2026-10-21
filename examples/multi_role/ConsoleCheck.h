#pragma once

// `check` on the text console: the QoS and SoH verdicts of `beebo check`, with the same bands
// and wording (a test in beebo/verif/test/packaging compares the two). Arduino-free so fw/test/
// can build it. Values are 0-10000, two decimals of a percent, as MonRing::computeQos/Soh.

#include <cstddef>
#include <cstdint>
#include <cstdio>

struct ConsoleCheckBand { unsigned min; const char* verdict; const char* text; };   // min: 0-10000

static const unsigned CONSOLE_MIN_QOS_EXPOSURE = 20;   // confirmable attempts before QoS gets a verdict

static const ConsoleCheckBand CONSOLE_QOS_BANDS[] = {
  {9500, "OK", "Excellent."},
  {8000, "INFO", "Good, not excellent."},
  {6000, "WARN", "Degraded -- see `beebo analyze tuning`."},
  {0, "FAIL", "Poor -- most sends timing out or dropped."},
};

static const ConsoleCheckBand CONSOLE_SOH_BANDS[] = {
  {9900, "OK", "No meaningful fault rate."},
  {9500, "INFO", "Small fault rate -- see `beebo monitor status`."},
  {8000, "WARN", "Notable fault rate -- undersized or congested."},
  {0, "FAIL", "High fault rate -- node is struggling to keep up."},
};

inline void consoleCheckLine(const char* name, unsigned value, const ConsoleCheckBand* bands, size_t n, char* out, size_t cap) {
  size_t i = 0;
  while (i + 1 < n && value < bands[i].min) i++;
  unsigned tenths = (value + 5) / 10;   // one decimal, rounded as the CLI's :.1f
  snprintf(out, cap, "%s %u.%u%% %s %s", name, tenths / 10, tenths % 10, bands[i].verdict, bands[i].text);
}

inline void consoleCheckQos(unsigned qos, unsigned exposure, char* out, size_t cap) {
  if (exposure < CONSOLE_MIN_QOS_EXPOSURE) {
    unsigned tenths = (qos + 5) / 10;
    snprintf(out, cap, "QoS %u.%u%% PENDING Only %u attempt%s so far.", tenths / 10, tenths % 10, exposure, exposure != 1 ? "s" : "");
    return;
  }
  consoleCheckLine("QoS", qos, CONSOLE_QOS_BANDS, sizeof(CONSOLE_QOS_BANDS) / sizeof(CONSOLE_QOS_BANDS[0]), out, cap);
}

inline void consoleCheckSoh(unsigned soh, char* out, size_t cap) {
  consoleCheckLine("SoH", soh, CONSOLE_SOH_BANDS, sizeof(CONSOLE_SOH_BANDS) / sizeof(CONSOLE_SOH_BANDS[0]), out, cap);
}

// One line per indicator.
inline void consoleCheck(unsigned qos, unsigned exposure, unsigned soh, char* out, size_t cap) {
  consoleCheckQos(qos, exposure, out, cap);
  size_t n = 0;
  while (out[n]) n++;
  if (n + 1 < cap) { out[n++] = '\n'; consoleCheckSoh(soh, out + n, cap - n); }
}
