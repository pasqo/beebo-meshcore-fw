#pragma once

#include <math.h>
#include <stdint.h>
#include "EvalWindow.h"

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
// switch), which cancels drift within a few
// blocks; each adjacent block pair is one A/B pair. Decision, deliberately
// conservative: adopt B only with >= MIN_PAIRS valid pairs, a mean relative
// goodput gain above ADOPT_Z standard errors, and a confirm ratio not worse
// than RATIO_TOLERANCE; otherwise keep A. A B block whose ratio collapses
// below A's running mean, or any block with the utilization guardrail
// tripped, aborts and reverts. Unmeasured blocks discard their pair only.
// Reach (neighbors heard / marginal) is averaged per value (A, B) for
// diagnostics; it does not enter the decision. All constants are first picks.
//
// Pragmatic early stop: checked after every new pair once
// EARLY_STOP_MIN_PAIRS are in, in EITHER direction (B clearly better ->
// ADOPT_B, B clearly worse -> KEEP_A -- no reason to keep running degraded
// service once that's obvious either). EARLY_STOP_Z is deliberately much
// stricter than ADOPT_Z: checking every pair, instead of once at the end,
// inflates the false-positive rate a single ADOPT_Z=2 check controls for,
// so the bar has to be high enough to absorb that. This is not a real
// group-sequential/alpha-spending design (no per-checkpoint schedule,
// no formal error-rate guarantee) -- a deliberately simple stopgap; revisit
// if it turns out to fire on noise early in a trial's span of hours.
class TrialFSM {
public:
  enum State : uint8_t { IDLE, RUN, DONE };
  // SKIPPED_* are never produced by the FSM: Beebo::startTrial() reports a
  // switch it declined to run (see emitTrialSkip()) in the same trial_result event.
  enum Outcome : uint8_t { NONE, KEEP_A, ADOPT_B, ABORTED, SKIPPED_SAME, SKIPPED_NO_CONTROL };

  static constexpr uint8_t  MIN_PAIRS = 20;
  static constexpr float    ADOPT_Z = 2.0f;
  static constexpr int16_t  RATIO_TOLERANCE = 200;    // confirm ratio, 0-10000
  static constexpr int16_t  COLLAPSE_THRESHOLD = 1500;
  static constexpr uint32_t EARLY_STOP_MIN_PAIRS = 5;
  static constexpr float    EARLY_STOP_Z = 4.0f;

  struct Config {
    uint16_t block_s = 1800;
    uint16_t blocks = 96;
  };

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

    double g = valid ? (double)r.ros_rate * r.confirm_ratio / 10000.0 : 0.0;
    bool pair_added = false;
    if (!(_idx & 1)) {
      _prev_valid = valid;
      _prev_is_b = is_b;
      _prev_g = g;
      _prev_ratio = valid ? r.confirm_ratio : 0;
    } else if (valid && _prev_valid) {
      double gb = is_b ? g : _prev_g, ga = is_b ? _prev_g : g;
      int32_t rb = is_b ? r.confirm_ratio : _prev_ratio, ra = is_b ? _prev_ratio : r.confirm_ratio;
      if (ga + gb > 0.0) {
        double d = (gb - ga) / ((ga + gb) / 2.0);
        _n++;
        _sum_d += d;
        _sum_d2 += d * d;
        _sum_rd += (double)(rb - ra);
        pair_added = true;
      }
    }

    _idx++;
    if (pair_added && _n >= EARLY_STOP_MIN_PAIRS) {
      double mean = _sum_d / _n;
      double se = stdErr();
      bool ratio_ok = (_sum_rd / _n) >= -(double)RATIO_TOLERANCE;
      bool winning = mean > 0.0 && (se > 0.0 ? mean / se >= EARLY_STOP_Z : true);
      bool losing  = mean < 0.0 && (se > 0.0 ? mean / se <= -EARLY_STOP_Z : true);
      if (winning && ratio_ok) return finish(ADOPT_B);
      if (losing) return finish(KEEP_A);
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

  Step decide() {
    if (_n < MIN_PAIRS) return finish(KEEP_A);
    double mean = _sum_d / _n;
    double se = stdErr();
    bool significant = mean > 0.0 && (se > 0.0 ? mean / se >= ADOPT_Z : true);
    bool ratio_ok = (_sum_rd / _n) >= -(double)RATIO_TOLERANCE;
    return finish(significant && ratio_ok ? ADOPT_B : KEEP_A);
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
