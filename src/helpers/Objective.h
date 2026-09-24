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
// into a plain weighted sum of percentage changes, which is why unlike units
// (packets per hour, a 0-1 ratio) combine without scaling: +10% in any
// indicator moves ln P by about 0.1 * w. The defaults, routed = 1 and
// confirm = 1, everything else 0, are exactly the goodput the tuners always
// used (routed rate x confirm ratio).
//
// Levels are positive numbers in each indicator's natural unit, floored at
// FLOOR (0.01) before the log so a zero level is a finite, ordinary (very
// large) relative change instead of ln(0); cost percentages enter as
// 1 + pct/100 so they never get near it. Runs on the device, once per closed
// window: a handful of logf calls, all in the log domain so extreme weights
// cannot overflow.
class Objective {
public:
  enum Indicator : uint8_t {
    ROUTED,        // confirmed deliveries per hour (ros_rate)
    CONFIRM,       // TX confirm ratio, 0-1
    RX_VALID,      // valid packets received per hour
    RX_ERRORS,     // radio-level receive errors per hour
    TX_DISPATCHED, // packets sent per hour
    NBR_HEARD,     // direct neighbors heard in the window
    POOL_BUSY,     // packet pool utilization, as 1 + pct/100
    CAD_BUSY,      // channel-busy TX wait, as 1 + pct/100
    NUM_INDICATORS
  };

  static constexpr int8_t WEIGHT_MIN = -50, WEIGHT_MAX = 50;
  static constexpr double FLOOR = 0.01;

  int8_t weights[NUM_INDICATORS] = {10, 10, 0, 0, 0, 0, 0, 0};

  // Levels for a window in absolute units (the trial compares two blocks of
  // the same length, so no baseline is needed).
  static void levels(const EvalWindow::Result &r, float x[NUM_INDICATORS]) {
    float per_hour = r.window_ms ? 3600000.0f / (float)r.window_ms : 0.0f;
    x[ROUTED] = (float)r.ros_rate;
    x[CONFIRM] = (float)r.confirm_ratio / 10000.0f;
    x[RX_VALID] = (float)r.rx_valid * per_hour;
    x[RX_ERRORS] = (float)r.rx_errors * per_hour;
    x[TX_DISPATCHED] = (float)r.tx_dispatched * per_hour;
    x[NBR_HEARD] = (float)r.reach_heard;
    x[POOL_BUSY] = 1.0f + (float)r.util_pct / 100.0f;
    x[CAD_BUSY] = 1.0f + (float)r.cad_busy_pct / 100.0f;
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

  // The adaptive tuner's per-window reward, on the 0-10000 scale goodput
  // always had (ros_norm x confirm ratio, ros_norm = routed rate over its
  // rolling baseline), capped at `cap`. Routed and confirm keep exactly that
  // form; the other rate-like indicators are divided by their own running
  // average, seeded by the first window, so a weight on one of them scores
  // change against recent history, not raw units. Updates those averages;
  // call once per measured window.
  float banditReward(const EvalWindow::Result &r, float cap = 20000.0f) {
    float x[NUM_INDICATORS];
    levels(r, x);
    float raw[NUM_INDICATORS];
    for (int i = 0; i < NUM_INDICATORS; i++) raw[i] = x[i];
    x[ROUTED] = (float)r.ros_norm / 1000.0f;
    for (int i = RX_VALID; i <= NBR_HEARD; i++) {
      x[i] = _ema[i] > 0.0f ? raw[i] / _ema[i] : 1.0f;
      _ema[i] = _ema[i] > 0.0f ? _ema[i] + (raw[i] - _ema[i]) * EMA_ALPHA : raw[i];
    }
    double j = logProduct(x);
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
