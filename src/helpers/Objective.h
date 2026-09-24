#pragma once

#include <math.h>
#include <stdint.h>
#include "EvalWindow.h"

// beebo: the objective both tuners score a closed window with (TrialFSM for
// its paired A/B blocks, TuneController for its step rewards). A weighted
// product of per-window indicators,
//
//   P = prod_i level_i ^ (w_i / 10)          (ln P = sum w_i/10 * ln level_i)
//
// so a weight is an exponent in tenths (10 = 1.0), weight 0 drops the
// indicator, and a negative weight penalizes a cost. Taking the log turns it
// into a plain weighted sum of percentage changes, so unlike indicators
// combine without unit juggling: +10% in any indicator moves ln P by about
// 0.1 * w. The defaults, routed = 1 and confirm = 1, everything else 0, are
// exactly the goodput the tuners always used (routed rate x confirm ratio).
//
// Every level is on one 0-100 scale (percent): the confirm ratio and the two
// cost percentages already are; per-hour volumes are a percent of a fixed
// reference of VOLUME_REFERENCE_PER_HOUR events (100 = one per second), the
// neighbors heard a percent of the neighbor table. One floor then means the
// same thing everywhere: every level is floored at FLOOR (0.01 percent) before
// the log, so a zero level is a finite, ordinary (very large) relative change
// instead of ln(0). The reference is fixed on purpose: a constant scale cancels
// in the trial's B - A difference, while a capacity that followed the radio
// settings would bias a coding-rate trial. Runs on the device, once per closed
// window: a handful of log calls, all in the log domain so extreme weights
// cannot overflow.
class Objective {
public:
  enum Indicator : uint8_t {
    ROUTED,        // confirmed deliveries per hour (ros_rate), % of the reference
    CONFIRM,       // TX confirm ratio, %
    RX_VALID,      // valid packets received per hour, % of the reference
    RX_ERRORS,     // radio-level receive errors per hour, % of the reference
    TX_DISPATCHED, // packets sent per hour, % of the reference
    NBR_HEARD,     // direct neighbors heard, % of the neighbor table
    POOL_BUSY,     // packet pool utilization, %
    CAD_BUSY,      // channel-busy TX wait, %
    NUM_INDICATORS
  };

  static constexpr int8_t WEIGHT_MIN = -50, WEIGHT_MAX = 50;
  static constexpr double FLOOR = 0.01;                    // percent
  static constexpr float VOLUME_REFERENCE_PER_HOUR = 3600.0f;   // = 100 on the scale
  static constexpr float NEIGHBOR_SLOTS = 16.0f;                // Beebo.h MAX_NEIGHBOURS

  int8_t weights[NUM_INDICATORS] = {10, 10, 0, 0, 0, 0, 0, 0};

  // Levels for a window in absolute units (the trial compares two blocks of
  // the same length, so no baseline is needed).
  static void levels(const EvalWindow::Result &r, float x[NUM_INDICATORS]) {
    float per_hour = r.window_ms ? 3600000.0f / (float)r.window_ms : 0.0f;
    const float vol = 100.0f / VOLUME_REFERENCE_PER_HOUR;
    x[ROUTED] = (float)r.ros_rate * vol;
    x[CONFIRM] = (float)r.confirm_ratio / 100.0f;
    x[RX_VALID] = (float)r.rx_valid * per_hour * vol;
    x[RX_ERRORS] = (float)r.rx_errors * per_hour * vol;
    x[TX_DISPATCHED] = (float)r.tx_dispatched * per_hour * vol;
    x[NBR_HEARD] = (float)r.reach_heard * 100.0f / NEIGHBOR_SLOTS;
    x[POOL_BUSY] = (float)r.util_pct;
    x[CAD_BUSY] = (float)r.cad_busy_pct;
  }

  // The trial's per-block score ln P; the trial compares blocks by the plain
  // difference of these (a weighted sum of relative changes).
  double trialLog(const EvalWindow::Result &r) const {
    float x[NUM_INDICATORS];
    levels(r, x);
    return logProduct(x);
  }

  // P itself (tests, diagnostics); may overflow to inf for extreme weights.
  double trialValue(const EvalWindow::Result &r) const { return exp(trialLog(r)); }

  // The adaptive tuner's per-window reward: 10000 * e^(J - J_ref) on the
  // 0-10000 scale goodput always had, capped at `cap`. J_ref is J with every
  // weighted level at its neutral value, 100 (the reference, "as usual"), so
  // the scale does not move with the weights: all indicators at neutral give
  // 10000. Routed enters as 100 * ros_norm (its rate over its rolling
  // baseline) and confirm as a percent, so the defaults give exactly
  // ros_norm * ratio * 1e4. The other volume-like indicators enter as
  // 100 * level / (its own running average, seeded by the first window), so a
  // weight on one of them scores change against recent history. Updates those
  // averages; call once per measured window.
  float banditReward(const EvalWindow::Result &r, float cap = 20000.0f) {
    float x[NUM_INDICATORS];
    levels(r, x);
    float raw[NUM_INDICATORS];
    for (int i = 0; i < NUM_INDICATORS; i++) raw[i] = x[i];
    x[ROUTED] = (float)r.ros_norm / 10.0f;   // ros_norm 1000 = 1.0 -> 100
    for (int i = RX_VALID; i <= NBR_HEARD; i++) {
      x[i] = _ema[i] > 0.0f ? 100.0f * raw[i] / _ema[i] : 100.0f;
      _ema[i] = _ema[i] > 0.0f ? _ema[i] + (raw[i] - _ema[i]) * EMA_ALPHA : raw[i];
    }
    double j = logProduct(x);
    for (int i = 0; i < NUM_INDICATORS; i++) j -= (weights[i] / 10.0) * log(100.0);
    double ln_cap = log((double)cap / 10000.0);
    if (j >= ln_cap) return cap;
    return (float)(10000.0 * exp(j));
  }

  void resetBaselines() {
    for (int i = 0; i < NUM_INDICATORS; i++) _ema[i] = 0.0f;
  }

  // ln of the product of level_i^(w_i/10) over the non-zero weights, every
  // level floored at FLOOR.
  double logProduct(const float x[NUM_INDICATORS]) const {
    double j = 0.0;
    for (int i = 0; i < NUM_INDICATORS; i++) {
      int w = weights[i];
      if (w == 0) continue;
      double level = x[i] < FLOOR ? FLOOR : x[i];
      j += (w / 10.0) * log(level);
    }
    return j;
  }

private:
  static constexpr float EMA_ALPHA = 0.125f;
  float _ema[NUM_INDICATORS] = {0};
};
