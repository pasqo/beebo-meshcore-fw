#pragma once

// Arduino-free formatters for the status feed's one-line messages (plans/ROLE_APP_FEED.md,
// source table), unit-testable from fw/test/. Each returns the line length written to `buf`.

#include <cstddef>
#include <cstdint>
#include <cstdio>

// TrialFSM::Outcome, in enum order; same names as the CLI's TRIAL_OUTCOME_NAMES.
inline const char* feedTrialOutcomeName(uint8_t outcome) {
  static const char* const names[] = {
    "none", "keep_a", "adopt_b", "aborted_changed", "skipped_same", "skipped_no_control",
    "aborted_guardrail", "inconclusive", "alive_a", "alive_b", "aborted_idle", "undecided",
    "skipped_underpowered" };
  return outcome < sizeof(names) / sizeof(names[0]) ? names[outcome] : "outcome?";
}

// A x1000 ratio as a percent with one decimal ("+1.2", "-0.4").
inline int feedFmtGain(char* buf, size_t cap, int32_t x1000) {
  int32_t a = x1000 < 0 ? -x1000 : x1000;
  return snprintf(buf, cap, "%s%d.%d", x1000 < 0 ? "-" : "+", (int)(a / 10), (int)(a % 10));
}

// qos and soh are 0-10000 scaled; `has_qos` false (no confirmable exposure yet) prints "-".
inline int feedFmtMon(char* buf, size_t cap, bool on, uint32_t rx, uint32_t tx, bool has_qos,
                      uint16_t qos, uint16_t soh) {
  char q[8];
  if (has_qos) snprintf(q, sizeof(q), "%u", (unsigned)(qos / 100));
  else snprintf(q, sizeof(q), "-");
  return snprintf(buf, cap, "mon %s rx=%lu tx=%lu qos=%s soh=%u", on ? "on" : "off",
                  (unsigned long)rx, (unsigned long)tx, q, (unsigned)(soh / 100));
}

inline int feedFmtTrialStart(char* buf, size_t cap, const char* sw, const char* profile, uint8_t a,
                             uint8_t b, uint16_t blocks, uint16_t block_s) {
  return snprintf(buf, cap, "trial %s start profile=%s a=%u b=%u blocks=%u block=%us", sw, profile,
                  (unsigned)a, (unsigned)b, (unsigned)blocks, (unsigned)block_s);
}

// `index` is the block that closed; the interval is shown once the trial has one.
inline int feedFmtTrialBlock(char* buf, size_t cap, const char* sw, uint16_t index, uint16_t total,
                             uint32_t pairs, int32_t mean_x1000, bool bounds, int32_t lower_x1000,
                             int32_t upper_x1000) {
  char g[12], lo[12], hi[12];
  feedFmtGain(g, sizeof(g), mean_x1000);
  int n = snprintf(buf, cap, "trial %s blk %u/%u pairs=%lu gain=%s%%", sw, (unsigned)(index + 1),
                   (unsigned)total, (unsigned long)pairs, g);
  if (bounds && n > 0 && (size_t)n < cap) {
    feedFmtGain(lo, sizeof(lo), lower_x1000);
    feedFmtGain(hi, sizeof(hi), upper_x1000);
    n += snprintf(buf + n, cap - n, " [%s,%s]", lo, hi);
  }
  return n;
}

inline int feedFmtTrialEnd(char* buf, size_t cap, const char* sw, uint8_t outcome, uint8_t a,
                           uint8_t b, uint32_t pairs, int32_t mean_x1000, int32_t se_x1000) {
  char g[12], s[12];
  feedFmtGain(g, sizeof(g), mean_x1000);
  feedFmtGain(s, sizeof(s), se_x1000);
  return snprintf(buf, cap, "trial %s %s a=%u b=%u pairs=%lu gain=%s%% se=%s%%", sw,
                  feedTrialOutcomeName(outcome), (unsigned)a, (unsigned)b, (unsigned long)pairs, g, s);
}

// A switch the trial declined to run.
inline int feedFmtTrialSkip(char* buf, size_t cap, const char* sw, uint8_t outcome) {
  return snprintf(buf, cap, "trial %s %s", sw, feedTrialOutcomeName(outcome));
}

// The sequence is enabled but the node counts too little traffic to start a trial.
inline int feedFmtTrialWait(char* buf, size_t cap) {
  return snprintf(buf, cap, "trial waiting for traffic");
}

// Below the PA's saturation onset (16 dBm, see config.py's _check_txpower) the V4.3 is
// underdriven. 0, with `buf` untouched, when the power is fine.
inline int feedFmtTxPowerWarn(char* buf, size_t cap, int dbm) {
  if (dbm >= 16) return 0;
  return snprintf(buf, cap, "warn: txpower=%d dBm underdrives the PA, run: radio.txpower 18", dbm);
}
