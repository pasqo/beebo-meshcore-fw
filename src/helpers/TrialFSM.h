#pragma once

#include <math.h>
#include <stdint.h>
#include "EvalWindow.h"
#include "Objective.h"

// beebo: on-device A/B trial for one front-end switch (FEM LNA, RX
// boosted gain, coding rate), stage 1 of the two-stage tuner (see plans/
// DYNAMIC_OPTIMIZER_PLAN.md). These switches are high-sensitivity and
// site/environment dependent, so a bandit's -step/stay/+step arms are the
// wrong tool; a paired trial decides them directly.
//
// The FSM is pure logic (no hardware, no records): the caller applies
// Step.value to the switch LIVE ONLY, never persisting it, so a reboot or
// crash mid-trial returns to the stored original. Only a winner
// (Step.final_value on ADOPT_B) is persisted by the caller.
//
// Blocks are fixed duration (the caller runs an EvalWindow with min_s == max_s
// == block_s), so blocks match the time of day. Order is ABBA cyclic
// (A = the original value, B = its alternative, the opposite for a binary
// switch), which cancels drift within a few blocks; each adjacent block pair
// is one A/B pair.
//
// Decision: a sequential test on the paired difference d of ln(objective)
// (Objective.h; goodput by default), i.e. the weighted relative change B vs A
// (B - A, relative to the pair mean), checked after every new pair once
// MIN_LOOK_PAIRS are in, so the trial stops as soon as the evidence settles
// instead of at a fixed pair count. With n pairs, mean m and standard error
// se, the confidence bound half-width is b*se where
//   b = tCrit(n - 1, alpha) * sqrt(N / n)
// (N = pairs in the full schedule). tCrit is the two-sided Student-t critical
// value at the configured error rate, so few pairs get an honest wide interval;
// the sqrt(N / n) factor is an O'Brien-Fleming-style tightening that keeps
// checking after every pair from inflating the false-positive rate (it is 1
// at the last pair of the schedule, so the final look is the plain t-test).
//   keep A   when the upper bound m + b*se is below the minimum worthwhile gain
//            (B cannot beat A by min_gain_pct), i.e. no meaningful difference or B
//            worse (checked first)
//   adopt B  when the lower bound m - b*se > 0 and the confirm ratio is no
//            worse than RATIO_TOLERANCE
//   else keep measuring; the schedule's end is a final look (b without the
//   sqrt term) and anything undecided keeps A.
// A B block whose ratio collapses below A's running mean, or any block with the
// utilization guardrail tripped, aborts and reverts. Unmeasured blocks discard
// their pair only. Reach (neighbors heard / marginal) is averaged per value
// (A, B) for diagnostics; it does not enter the decision. Everything runs on
// the device: no host input, running sums only (no per-pair history).
//
class TrialFSM {
public:
  enum State : uint8_t { IDLE, RUN, DONE };
  // SKIPPED_* are never produced by the FSM: Beebo::startTrial() reports a
  // switch it declined to run (see emitTrialSkip()) in the same trial_result event.
  enum Outcome : uint8_t { NONE, KEEP_A, ADOPT_B, ABORTED, SKIPPED_SAME, SKIPPED_NO_CONTROL };

  static constexpr uint32_t MIN_LOOK_PAIRS = 3;       // fewest pairs a decision may rest on
  static constexpr int16_t  RATIO_TOLERANCE = 200;    // confirm ratio, 0-10000
  static constexpr int16_t  COLLAPSE_THRESHOLD = 1500;

  struct Config {
    uint16_t block_s = 1800;
    uint16_t blocks = 96;
    uint8_t alpha_pct = 5;   // overall error rate: 1, 5 or 10 (other values use 5)
    uint8_t min_gain_pct = 5;    // smallest worthwhile relative gain, percent
    const Objective *objective = nullptr;   // per-block value; null = the default goodput weights
  };

  // Two-sided Student-t critical value for `df` degrees of freedom at
  // alpha_pct 1 / 5 / 10 (else 5). df above 30 uses the last tabulated value at
  // or below it (slightly conservative), df > 120 the normal quantile.
  static float tCrit(uint32_t df, uint8_t alpha_pct) {
    static const float t10[] = {6.314f, 2.920f, 2.353f, 2.132f, 2.015f, 1.943f, 1.895f, 1.860f, 1.833f, 1.812f,
                                1.796f, 1.782f, 1.771f, 1.761f, 1.753f, 1.746f, 1.740f, 1.734f, 1.729f, 1.725f,
                                1.721f, 1.717f, 1.714f, 1.711f, 1.708f, 1.706f, 1.703f, 1.701f, 1.699f, 1.697f};
    static const float t05[] = {12.706f, 4.303f, 3.182f, 2.776f, 2.571f, 2.447f, 2.365f, 2.306f, 2.262f, 2.228f,
                                2.201f, 2.179f, 2.160f, 2.145f, 2.131f, 2.120f, 2.110f, 2.101f, 2.093f, 2.086f,
                                2.080f, 2.074f, 2.069f, 2.064f, 2.060f, 2.056f, 2.052f, 2.048f, 2.045f, 2.042f};
    static const float t01[] = {63.657f, 9.925f, 5.841f, 4.604f, 4.032f, 3.707f, 3.499f, 3.355f, 3.250f, 3.169f,
                                3.106f, 3.055f, 3.012f, 2.977f, 2.947f, 2.921f, 2.898f, 2.878f, 2.861f, 2.845f,
                                2.831f, 2.819f, 2.807f, 2.797f, 2.787f, 2.779f, 2.771f, 2.763f, 2.756f, 2.750f};
    const float *t = alpha_pct == 10 ? t10 : alpha_pct == 1 ? t01 : t05;
    if (df < 1) df = 1;
    if (df <= 30) return t[df - 1];
    float t40 = alpha_pct == 10 ? 1.684f : alpha_pct == 1 ? 2.704f : 2.021f;
    float t60 = alpha_pct == 10 ? 1.671f : alpha_pct == 1 ? 2.660f : 2.000f;
    float t120 = alpha_pct == 10 ? 1.658f : alpha_pct == 1 ? 2.617f : 1.980f;
    float tz = alpha_pct == 10 ? 1.645f : alpha_pct == 1 ? 2.576f : 1.960f;
    return df < 40 ? t[29] : df < 60 ? t40 : df < 120 ? t60 : df == 120 ? t120 : tz;
  }

  struct Step {
    bool set_value;     // apply `value` to the switch (live only) before the next block
    uint8_t value;
    bool finished;
    Outcome outcome;
    uint8_t final_value;  // value to leave (and, if != original, persist) when finished
  };

  struct Stats {
    uint32_t n_pairs;
    int32_t mean_rel_x1000;   // mean relative goodput difference (B - A), x1000
    int32_t se_x1000;
    int32_t mean_ratio_diff;  // mean confirm ratio difference (B - A), 0-10000 scale
    uint32_t reach_heard_a_x10, reach_heard_b_x10;
    uint32_t reach_marginal_a_x10, reach_marginal_b_x10;
  };

  void begin(const Config &cfg) {
    _cfg = cfg;
    uint16_t n = cfg.blocks & ~1u;   // whole pairs
    _total = n < 2 ? 2 : n;
    _state = IDLE;
    resetRun();
  }

  State state() const { return _state; }
  uint16_t blockIndex() const { return _idx; }
  uint16_t totalBlocks() const { return _total; }
  uint16_t blockSeconds() const { return _cfg.block_s; }
  uint8_t original() const { return _orig; }
  uint8_t alternative() const { return _alt; }
  uint8_t currentValue() const { return valueFor(_idx); }

  // `alt` is arm B's value. Omitted (a binary switch), B is the opposite of
  // `original` and `original` is clamped to 0/1; a multi-valued switch (coding
  // rate) passes both explicitly.
  static constexpr uint8_t ALT_OPPOSITE = 0xFF;
  bool start(uint8_t original, uint8_t alt = ALT_OPPOSITE) {
    if (_state == RUN) return false;
    resetRun();
    if (alt == ALT_OPPOSITE) {
      _orig = original ? 1 : 0;
      _alt = !_orig;
    } else {
      _orig = original;
      _alt = alt;
    }
    _state = RUN;
    return true;
  }

  // Revert to the original and stop (setting change, tuner disabled, ...).
  Step abort() {
    Step s = {};
    if (_state != RUN) return s;
    return finish(ABORTED);
  }

  Step onBlock(const EvalWindow::Result &r, uint8_t reach_heard, uint8_t reach_marginal) {
    Step s = {};
    if (_state != RUN) return s;
    bool is_b = armIsB(_idx);
    bool valid = r.measured;

    if (valid) {
      if (r.flags & EVALF_GUARDRAIL) return finish(ABORTED);
      if (is_b && _a_ratio_n > 0 &&
          (int32_t)r.confirm_ratio + COLLAPSE_THRESHOLD < (int32_t)(_a_ratio_sum / _a_ratio_n)) {
        return finish(ABORTED);
      }
      if (!is_b) { _a_ratio_sum += r.confirm_ratio; _a_ratio_n++; }
      if (is_b) { _heard_b += reach_heard; _marg_b += reach_marginal; _reach_b_n++; }
      else      { _heard_a += reach_heard; _marg_a += reach_marginal; _reach_a_n++; }
    }

    double g = valid ? objective().trialLog(r) : 0.0;   // ln of the objective
    bool pair_added = false;
    if (!(_idx & 1)) {
      _prev_valid = valid;
      _prev_is_b = is_b;
      _prev_g = g;
      _prev_ratio = valid ? r.confirm_ratio : 0;
    } else if (valid && _prev_valid) {
      double gb = is_b ? g : _prev_g, ga = is_b ? _prev_g : g;
      int32_t rb = is_b ? r.confirm_ratio : _prev_ratio, ra = is_b ? _prev_ratio : r.confirm_ratio;
      {
        double d = gb - ga;   // difference of ln(objective): the weighted relative change
        _n++;
        _sum_d += d;
        _sum_d2 += d * d;
        _sum_rd += (double)(rb - ra);
        pair_added = true;
      }
    }

    _idx++;
    if (pair_added && _n >= MIN_LOOK_PAIRS) {
      int v = look(false);
      if (v > 0) return finish(ADOPT_B);
      if (v < 0) return finish(KEEP_A);
    }
    if (_idx >= _total) return decide();
    s.set_value = true;
    s.value = valueFor(_idx);
    return s;
  }

  Stats stats() const {
    Stats st = {};
    st.n_pairs = _n;
    if (_n > 0) {
      double mean = _sum_d / _n;
      st.mean_rel_x1000 = (int32_t)lround(mean * 1000.0);
      st.mean_ratio_diff = (int32_t)lround(_sum_rd / _n);
      st.se_x1000 = (int32_t)lround(stdErr() * 1000.0);
    }
    if (_reach_a_n) { st.reach_heard_a_x10 = _heard_a * 10 / _reach_a_n; st.reach_marginal_a_x10 = _marg_a * 10 / _reach_a_n; }
    if (_reach_b_n) { st.reach_heard_b_x10 = _heard_b * 10 / _reach_b_n; st.reach_marginal_b_x10 = _marg_b * 10 / _reach_b_n; }
    return st;
  }

private:
  static bool armIsB(uint16_t idx) { uint8_t m = idx & 3; return m == 1 || m == 2; }   // A B B A
  uint8_t valueFor(uint16_t idx) const { return armIsB(idx) ? _alt : _orig; }

  void resetRun() {
    _idx = 0;
    _orig = 0;
    _alt = 1;
    _prev_valid = false; _prev_is_b = false; _prev_g = 0; _prev_ratio = 0;
    _n = 0; _sum_d = _sum_d2 = _sum_rd = 0.0;
    _a_ratio_sum = 0; _a_ratio_n = 0;
    _heard_a = _heard_b = _marg_a = _marg_b = 0;
    _reach_a_n = _reach_b_n = 0;
  }

  double stdErr() const {
    if (_n < 2) return 0.0;
    double mean = _sum_d / _n;
    double var = (_sum_d2 - _n * mean * mean) / (_n - 1);
    if (var < 0.0) var = 0.0;
    return sqrt(var / _n);
  }

  Step finish(Outcome o) {
    Step s = {};
    _state = DONE;
    s.finished = true;
    s.outcome = o;
    s.final_value = (o == ADOPT_B) ? _alt : _orig;
    s.set_value = true;
    s.value = s.final_value;
    return s;
  }

  const Objective &objective() const {
    static const Objective defaults;
    return _cfg.objective ? *_cfg.objective : defaults;
  }

  // +1: B wins, -1: keep A (no worthwhile gain), 0: keep measuring. `final`
  // is the last look of the schedule, where the plain t bound applies.
  int look(bool final) const {
    double n = (double)_n;
    double mean = _sum_d / n;
    double se = stdErr();
    double b = tCrit(_n - 1, _cfg.alpha_pct);
    if (!final) b *= sqrt((double)(_total / 2) / n);
    double lower = mean - b * se, upper = mean + b * se;
    bool ratio_ok = (_sum_rd / n) >= -(double)RATIO_TOLERANCE;
    if (upper < log(1.0 + _cfg.min_gain_pct / 100.0)) return -1;   // even the best case is below the minimum gain
    if (lower > 0.0 && ratio_ok) return 1;
    return 0;
  }

  Step decide() {
    if (_n < MIN_LOOK_PAIRS) return finish(KEEP_A);
    return finish(look(true) > 0 ? ADOPT_B : KEEP_A);
  }

  Config _cfg;
  State _state = IDLE;
  uint16_t _total = 96;
  uint16_t _idx = 0;
  uint8_t _orig = 0;
  uint8_t _alt = 1;
  bool _prev_valid = false, _prev_is_b = false;
  double _prev_g = 0;
  int32_t _prev_ratio = 0;
  uint32_t _n = 0;
  double _sum_d = 0, _sum_d2 = 0, _sum_rd = 0;
  uint32_t _a_ratio_sum = 0, _a_ratio_n = 0;
  uint32_t _heard_a = 0, _heard_b = 0, _marg_a = 0, _marg_b = 0;
  uint32_t _reach_a_n = 0, _reach_b_n = 0;
};
