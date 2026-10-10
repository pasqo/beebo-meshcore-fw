#pragma once

#include <math.h>
#include <stdint.h>
#include "EvalWindow.h"

// beebo: the objective both tuners score a closed window with (TrialFSM for
// its paired A/B blocks, AdaptiveController for its step rewards). A weighted
// product of per-window indicators,
//
//   P = prod_i level_i ^ (w_i / 10)          (ln P = sum w_i/10 * ln level_i)
//
// so a weight is an exponent in tenths (10 = 1.0), weight 0 drops the
// indicator, and a negative weight penalizes a cost. Taking the log turns it
// into a plain weighted sum of percentage changes, so unlike indicators
// combine without unit juggling: +10% in any indicator moves ln P by about
// 0.1 * w. The defaults, routed = 1 and delivered = 1, received 0, are the
// confirmed deliveries per hour: the packets routed (confirmable attempts;
// retries are not new ones) times the share of them confirmed.
//
// Every level is on one 0-100 scale (percent): the delivered share already is;
// per-hour volumes are a percent of the channel's usable capacity
// (volumeReference events per hour, see setVolumeReference()). One transform then
// serves every indicator, f(x) = ln(1 + x / C) with C = 0.1 (percent): it is 0
// at x = 0 and continuous, behaves like ln(x / C) once x is well above C (so a
// weight is the weighted relative change) and is linear below it (so a tiny
// count is not blown up). No floors, no special cases. The reference is fixed
// while a trial runs: a constant scale cancels in the trial's B - A difference,
// while one that followed the radio settings would bias a coding-rate trial.
// Runs on the device, once per closed window: a handful of log1p calls, all in
// the log domain so extreme weights cannot overflow.
class Objective {
public:
  enum Indicator : uint8_t {
    ROUTED,        // packets routed (confirmable attempts, a retry is not one) per hour, % of the reference
    DELIVERED,     // share of the routed packets that were confirmed, %
    RECEIVED,      // valid packets received per hour, % of the reference
    NUM_INDICATORS
  };

  static constexpr int8_t WEIGHT_MIN = -50, WEIGHT_MAX = 50;
  static constexpr double C = 0.1;                          // percent, the transform's scale
  static constexpr float DEFAULT_VOLUME_REFERENCE = 2800.0f;   // events/h, see below

  // Events per hour that read as 100 for the volume indicators: the channel's
  // usable capacity, 1 / (2e) = 18.4% of raw airtime (the pure-ALOHA maximum
  // for uncoordinated senders, what a CAD-and-jitter mesh approaches) divided
  // by the airtime of a nominal 64-byte packet at the node's radio settings:
  // 0.184 * 3600 s / airtime_s. About 2800 at SF7, 62.5 kHz, CR 4/5 (0.24 s per 64 bytes).
  float volumeReference = DEFAULT_VOLUME_REFERENCE;
  void setVolumeReference(float per_hour) {
    if (per_hour > 1.0f && fabsf(per_hour - volumeReference) > 0.01f * volumeReference) {
      volumeReference = per_hour;
      resetBaselines();
    }
  }
  static float referenceFor(uint32_t nominal_packet_airtime_ms) {
    return nominal_packet_airtime_ms ? 0.184f * 3600.0f * 1000.0f / (float)nominal_packet_airtime_ms
                                     : DEFAULT_VOLUME_REFERENCE;
  }

  int8_t weights[NUM_INDICATORS] = {10, 10, 0};

  // Levels for a window in absolute units (the trial compares two blocks of
  // the same length, so no baseline is needed).
  void levels(const EvalWindow::Result &r, float x[NUM_INDICATORS]) const {
    float per_hour = r.window_ms ? 3600000.0f / (float)r.window_ms : 0.0f;
    const float vol = 100.0f / volumeReference;
    x[ROUTED] = (float)r.exposure * per_hour * vol;
    x[DELIVERED] = (float)r.confirm_ratio / 100.0f;
    x[RECEIVED] = (float)r.rx_valid * per_hour * vol;
  }

  // The trial's per-block score ln P; the trial compares blocks by the plain
  // difference of these (a weighted sum of relative changes).
  double trialLog(const EvalWindow::Result &r) const {
    float x[NUM_INDICATORS];
    levels(r, x);
    return logProduct(x);
  }

  // Counting noise (variance of ln P) of one block with E routed attempts, C of
  // them confirmed and R packets received: routed is Poisson on E, delivered
  // binomial on E (variance (1 - p) / C) and received Poisson on R, treated as
  // independent, each term scaled by its weight squared. For the default weights
  // this is 1 / C. INFINITY when a weighted indicator has no count.
  double countingNoise(double E, double C, double R) const {
    double wr = weights[ROUTED] / 10.0, wd = weights[DELIVERED] / 10.0, wx = weights[RECEIVED] / 10.0;
    double v = 0.0;
    if (wr != 0.0) { if (E <= 0.0) return INFINITY; v += wr * wr / E; }
    if (wd != 0.0) {
      if (C <= 0.0) return INFINITY;
      double q = E > C ? 1.0 - C / E : 0.0;
      v += wd * wd * q / C;
    }
    if (wx != 0.0) { if (R <= 0.0) return INFINITY; v += wx * wx / R; }
    return v;
  }

  // P itself (tests, diagnostics); may overflow to inf for extreme weights.
  double trialValue(const EvalWindow::Result &r) const { return exp(trialLog(r)); }

  // The adaptive tuner's per-window reward: 10000 * e^(J - J_ref) on the
  // 0-10000 scale goodput always had, capped at `cap`. J_ref is J with every
  // weighted level at its neutral value, 100 (the reference, "as usual"), so
  // the scale does not move with the weights: all indicators at neutral give
  // 10000. Delivered enters as a percent. The two volumes (routed, received)
  // enter as 100 * level / (its own running average, seeded by the first
  // window), so a weight on one of them scores change against recent history:
  // the defaults give delivered ratio * routed / its average * 1e4. Updates
  // those averages; call once per measured window.
  float banditReward(const EvalWindow::Result &r, float cap = 20000.0f) {
    float x[NUM_INDICATORS];
    levels(r, x);
    float raw[NUM_INDICATORS];
    for (int i = 0; i < NUM_INDICATORS; i++) raw[i] = x[i];
    for (int i = 0; i < NUM_INDICATORS; i++) {
      if (i == DELIVERED) continue;   // a share, not a volume
      x[i] = _ema[i] > 0.0f ? 100.0f * raw[i] / _ema[i] : 100.0f;
      _ema[i] = _ema[i] > 0.0f ? _ema[i] + (raw[i] - _ema[i]) * EMA_ALPHA : raw[i];
    }
    double j = logProduct(x);
    const double neutral = log1p(100.0 / C);
    for (int i = 0; i < NUM_INDICATORS; i++) j -= (weights[i] / 10.0) * neutral;
    double ln_cap = log((double)cap / 10000.0);
    if (j >= ln_cap) return cap;
    return (float)(10000.0 * exp(j));
  }

  void resetBaselines() {
    for (int i = 0; i < NUM_INDICATORS; i++) _ema[i] = 0.0f;
  }

  // J = sum (w_i/10) * ln(1 + level_i / C) over the non-zero weights.
  double logProduct(const float x[NUM_INDICATORS]) const {
    double j = 0.0;
    for (int i = 0; i < NUM_INDICATORS; i++) {
      int w = weights[i];
      if (w == 0) continue;
      double level = x[i] > 0.0f ? x[i] : 0.0;
      j += (w / 10.0) * log1p(level / C);
    }
    return j;
  }

private:
  static constexpr float EMA_ALPHA = 0.125f;
  float _ema[NUM_INDICATORS] = {0};
};
