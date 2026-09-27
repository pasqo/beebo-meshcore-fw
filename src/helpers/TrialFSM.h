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
//            worse than RATIO_TOLERANCE, unless B's confirmed deliveries grew
//            (more forwarding lowers the ratio without losing anything)
//   else keep measuring; the schedule's end is a final look (b without the
//   sqrt term) and anything undecided there ends UNDECIDED, keeping A.
// A block with the utilization guardrail tripped aborts and reverts. An undecided trial whose bounds have stopped moving (stable())
// ends as INCONCLUSIVE and keeps A; a schedule that ends with fewer than
// MIN_LOOK_PAIRS pairs ends UNDECIDED and keeps A. Unmeasured blocks discard
// their pair from the paired test only.
//
// Before the paired test, after every pair: a dead arm. A block that heard
// nothing is unmeasured, so a setting that deafens the radio never forms a
// pair and the paired test alone would keep A by default. Each arm's RX Valid
// count (capped at ALIVE_CAP per block, against bursts) is summed over the
// pairs with no invalidated block; ABBA gives both arms equal time, so with
// no effect the split is 50/50. When the weaker arm has at most 1/ALIVE_RATIO
// of the stronger's count, the stronger heard something in at least
// ALIVE_MIN_BLOCKS blocks, and the two-sided binomial tail is below alpha /
// (pairs in the schedule), the stronger arm wins: ALIVE_A keeps A, ALIVE_B
// adopts B.
//
// After every A B B A cycle: idle. Fewer than Config.idle_min_rx packets in the whole
// cycle (both arms, no invalidated block) ends the trial ABORTED_IDLE: the
// channel is too quiet to decide anything. The caller retries the switch later.
//
// Reach (neighbors heard / marginal) is averaged per value (A, B) for
// diagnostics; it does not enter the decision. Everything runs on the device:
// no host input, running sums only (no per-pair history).
//
class TrialFSM {
public:
  enum State : uint8_t { IDLE, RUN, DONE };
  // SKIPPED_* are never produced by the FSM: Beebo::startTrial() reports a
  // switch it declined to run (see emitTrialSkip()) in a MON_TRIAL end record.
  enum Outcome : uint8_t { NONE, KEEP_A, ADOPT_B, ABORTED, SKIPPED_SAME, SKIPPED_NO_CONTROL,
                   ABORTED_GUARDRAIL, INCONCLUSIVE, ALIVE_A, ALIVE_B, ABORTED_IDLE, UNDECIDED };
                   // ABORTED: setting changed / tuner disabled

  static constexpr uint32_t MIN_LOOK_PAIRS = 3;       // fewest pairs a decision may rest on
  static constexpr int16_t  RATIO_TOLERANCE = 200;    // confirm ratio, 0-10000
  static constexpr uint32_t STABLE_PAIRS = 4;         // looks the bounds must stay put over to end an undecided trial
  static constexpr uint32_t ALIVE_CAP = 8;            // RX Valid counted per block by the dead-arm test
  static constexpr uint32_t ALIVE_RATIO = 4;          // the weaker arm at most 1/ALIVE_RATIO of the stronger
  static constexpr uint32_t ALIVE_MIN_BLOCKS = 2;     // blocks the stronger arm heard something in
  static constexpr uint8_t  IDLE_MIN_RX = 4;          // default Config.idle_min_rx

  struct Config {
    uint16_t block_s = 1800;
    uint16_t blocks = 96;
    uint8_t alpha_pct = 5;   // overall error rate: 1, 5 or 10 (other values use 5)
    uint8_t min_gain_pct = 5;    // smallest worthwhile relative gain, percent
    uint8_t idle_min_rx = IDLE_MIN_RX;   // packets per A B B A cycle below which the trial ends idle; 0 = never
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

  // Two-sided binomial tail P(X <= k) * 2 for n fair coin flips, capped at 1:
  // how likely a split this uneven (k of n on the weaker side) is with no effect.
  static double binomialTwoSided(uint32_t n, uint32_t k) {
    if (n == 0) return 1.0;
    double lt = -(double)n * log(2.0);   // ln C(n, 0) / 2^n
    double sum = 0.0;
    for (uint32_t i = 0; i <= k && i <= n; i++) {
      sum += exp(lt);
      lt += log((double)(n - i) / (double)(i + 1));
    }
    double p = 2.0 * sum;
    return p > 1.0 ? 1.0 : p;
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

  // The pairs so far and, from MIN_LOOK_PAIRS pairs on, the bounds the last look
  // used (ln units) -- what a per-block trace records.
  struct Progress {
    uint32_t n_pairs;
    double mean;
    bool bounds;
    double lower, upper;
  };
  Progress progress() const {
    Progress p = {};
    p.n_pairs = _n;
    if (_n > 0) p.mean = _sum_d / _n;
    if (_n >= MIN_LOOK_PAIRS) {
      p.bounds = true;
      p.lower = _lo[_n % HIST];
      p.upper = _hi[_n % HIST];
    }
    return p;
  }

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
      if (r.flags & EVALF_GUARDRAIL) return finish(ABORTED_GUARDRAIL);
      if (is_b) { _heard_b += reach_heard; _marg_b += reach_marginal; _reach_b_n++; }
      else      { _heard_a += reach_heard; _marg_a += reach_marginal; _reach_a_n++; }
    }

    double g = valid ? objective().trialLog(r) : 0.0;   // ln of the objective
    // heard or silent, but not invalidated: counts toward the dead-arm and idle tests
    bool usable = valid || r.outcome == EVAL_INSUFFICIENT_DATA;
    uint32_t rx = usable ? r.rx_valid : 0;
    if ((_idx & 3) == 0) { _cycle_rx = 0; _cycle_usable = true; }
    _cycle_rx += rx;
    _cycle_usable = _cycle_usable && usable;
    bool pair_added = false;
    bool pair_done = false;
    if (!(_idx & 1)) {
      _prev_valid = valid;
      _prev_is_b = is_b;
      _prev_g = g;
      _prev_ratio = valid ? r.confirm_ratio : 0;
      _prev_ros = valid ? r.ros_rate : 0;
      _prev_usable = usable;
      _prev_rx = rx;
    } else {
      pair_done = true;
      if (usable && _prev_usable) {
        uint32_t rb = is_b ? rx : _prev_rx, ra = is_b ? _prev_rx : rx;
        _live_a += ra < ALIVE_CAP ? ra : ALIVE_CAP;
        _live_b += rb < ALIVE_CAP ? rb : ALIVE_CAP;
        if (ra) _live_a_blocks++;
        if (rb) _live_b_blocks++;
      }
    }
    if (pair_done && valid && _prev_valid) {
      double gb = is_b ? g : _prev_g, ga = is_b ? _prev_g : g;
      int32_t rb = is_b ? r.confirm_ratio : _prev_ratio, ra = is_b ? _prev_ratio : r.confirm_ratio;
      {
        double d = gb - ga;   // difference of ln(objective): the weighted relative change
        _n++;
        _sum_d += d;
        _sum_d2 += d * d;
        _sum_rd += (double)(rb - ra);
        _sum_ros_d += (double)(is_b ? r.ros_rate : _prev_ros) - (double)(is_b ? _prev_ros : r.ros_rate);
        pair_added = true;
      }
    }

    _idx++;
    if (pair_done) {
      int alive = aliveArm();
      if (alive > 0) return finish(ALIVE_B);
      if (alive < 0) return finish(ALIVE_A);
    }
    if (pair_added && _n >= MIN_LOOK_PAIRS) {
      int v = look(false);
      if (v > 0) return finish(ADOPT_B);
      if (v < 0) return finish(KEEP_A);
      if (stable()) return finish(INCONCLUSIVE);
    }
    if ((_idx & 3) == 0 && _cycle_usable && _cycle_rx < _cfg.idle_min_rx) return finish(ABORTED_IDLE);
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
    _prev_usable = false; _prev_rx = 0; _prev_ros = 0; _sum_ros_d = 0.0;
    _live_a = _live_b = _live_a_blocks = _live_b_blocks = 0;
    _cycle_rx = 0; _cycle_usable = true;
    _n = 0; _sum_d = _sum_d2 = _sum_rd = 0.0;
    for (uint32_t i = 0; i < HIST; i++) _lo[i] = _hi[i] = 0.0;
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
    s.final_value = (o == ADOPT_B || o == ALIVE_B) ? _alt : _orig;
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
  int look(bool final) {
    double n = (double)_n;
    double mean = _sum_d / n;
    double se = stdErr();
    double b = tCrit(_n - 1, _cfg.alpha_pct);
    if (!final) b *= sqrt((double)(_total / 2) / n);
    double lower = mean - b * se, upper = mean + b * se;
    if (!final) { _lo[_n % HIST] = lower; _hi[_n % HIST] = upper; }
    bool ratio_ok = (_sum_rd / n) >= -(double)RATIO_TOLERANCE || _sum_ros_d > 0.0;
    if (upper < log(1.0 + _cfg.min_gain_pct / 100.0)) return -1;   // even the best case is below the minimum gain
    if (lower > 0.0 && ratio_ok) return 1;
    return 0;
  }

  // +1: B alive against a dead A, -1: the reverse, 0: neither arm dead.
  int aliveArm() const {
    uint32_t hi = _live_b > _live_a ? _live_b : _live_a;
    uint32_t lo = _live_b > _live_a ? _live_a : _live_b;
    uint32_t hi_blocks = _live_b > _live_a ? _live_b_blocks : _live_a_blocks;
    if (hi_blocks < ALIVE_MIN_BLOCKS || lo * ALIVE_RATIO > hi) return 0;
    double alpha = _cfg.alpha_pct == 10 ? 0.10 : _cfg.alpha_pct == 1 ? 0.01 : 0.05;
    if (binomialTwoSided(hi + lo, lo) >= alpha / (double)(_total / 2)) return 0;
    return _live_b > _live_a ? 1 : -1;
  }

  // Undecided and stable: over the last STABLE_PAIRS looks (one more than that many
  // values) neither bound moved by more than a quarter of the minimum worthwhile
  // gain, so more pairs would not change the outcome. Ends the trial keeping A.
  bool stable() const {
    if (_n < MIN_LOOK_PAIRS + STABLE_PAIRS) return false;
    double eps = log(1.0 + _cfg.min_gain_pct / 100.0) / 4.0;
    double lo_min = _lo[0], lo_max = _lo[0], hi_min = _hi[0], hi_max = _hi[0];
    for (uint32_t i = 1; i < HIST; i++) {
      if (_lo[i] < lo_min) lo_min = _lo[i];
      if (_lo[i] > lo_max) lo_max = _lo[i];
      if (_hi[i] < hi_min) hi_min = _hi[i];
      if (_hi[i] > hi_max) hi_max = _hi[i];
    }
    return lo_max - lo_min < eps && hi_max - hi_min < eps;
  }

  Step decide() {
    if (_n < MIN_LOOK_PAIRS) return finish(UNDECIDED);
    int v = look(true);
    return finish(v > 0 ? ADOPT_B : v < 0 ? KEEP_A : UNDECIDED);
  }

  static constexpr uint32_t HIST = STABLE_PAIRS + 1;
  double _lo[HIST] = {}, _hi[HIST] = {};   // bounds at the last looks, by pair count modulo HIST
  Config _cfg;
  State _state = IDLE;
  uint16_t _total = 96;
  uint16_t _idx = 0;
  uint8_t _orig = 0;
  uint8_t _alt = 1;
  bool _prev_valid = false, _prev_is_b = false;
  double _prev_g = 0;
  int32_t _prev_ratio = 0;
  bool _prev_usable = false;
  uint32_t _prev_rx = 0;
  uint32_t _prev_ros = 0;
  double _sum_ros_d = 0;   // confirmed deliveries per hour, B - A, summed over pairs
  uint32_t _live_a = 0, _live_b = 0, _live_a_blocks = 0, _live_b_blocks = 0;   // dead-arm test sums
  uint32_t _cycle_rx = 0;       // RX Valid in the current A B B A cycle
  bool _cycle_usable = true;    // no invalidated block in it so far
  uint32_t _n = 0;
  double _sum_d = 0, _sum_d2 = 0, _sum_rd = 0;
  uint32_t _heard_a = 0, _heard_b = 0, _marg_a = 0, _marg_b = 0;
  uint32_t _reach_a_n = 0, _reach_b_n = 0;
};
