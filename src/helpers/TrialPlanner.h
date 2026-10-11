#pragma once

#include <math.h>
#include <stdint.h>
#include "Objective.h"
#include "TrialFSM.h"

// beebo: the rate-adaptive schedule of an on-device ABBA trial (plans/
// TRIAL_AUTO_SCHEDULE.md). Pure logic, no hardware, no allocation.
//
// ExposureTracker: what the node's traffic looks like, from the same counters
// EvalWindow reads, so a plan can start at any time.
//
// TrialPlanner: from those rates and the objective's weights, the block length
// and the number of blocks a trial needs so its confidence bound can reach
// min_gain_pct. A block's variance is the counting noise v of its confirmed
// deliveries plus a block spread c the length does not average away; a pair
// of blocks differs with variance 2 (v + c), so the pairs needed are
//   P = 2 t^2 (v + c) / g^2        (t = Student-t at the confidence, g = ln(1 + min gain))
// A node whose rates cannot get there inside MAX_TRIAL_S is underpowered.

// The node's recent traffic, from the counters EvalWindow reads: counts and
// elapsed time decayed with the time constant TAU_S, so the rates follow the
// traffic of the last few hours and a node that just booted has them as soon as
// it has seen MIN_CONFIRMED confirmed deliveries over MIN_S.
class ExposureTracker {
public:
  static constexpr double TAU_S = 7200.0;
  static constexpr double MIN_S = 30.0;
  static constexpr double MIN_CONFIRMED = 5.0;
  static constexpr uint32_t STEP_MS = 1000;   // counters are folded in once a second

  // Feed the lifetime counters every loop tick. A counter that went backwards
  // (device-side reset) starts over from these values.
  void poll(uint32_t now_ms, uint32_t exposure, uint32_t confirmed, uint32_t rx) {
    if (!_started) {
      _started = true;
      _t = now_ms; _e = exposure; _c = confirmed; _r = rx;
      return;
    }
    uint32_t d_e = exposure - _e, d_c = confirmed - _c, d_r = rx - _r;
    if (d_e > 0x80000000u || d_c > 0x80000000u || d_r > 0x80000000u) {
      reset();
      poll(now_ms, exposure, confirmed, rx);
      return;
    }
    uint32_t dt = now_ms - _t;
    if (dt < STEP_MS) return;
    double s = dt / 1000.0, k = exp(-s / TAU_S);
    _sec = _sec * k + s;
    _exposure = _exposure * k + d_e;
    _confirmed = _confirmed * k + d_c;
    _rx = _rx * k + d_r;
    _t = now_ms; _e = exposure; _c = confirmed; _r = rx;
  }

  void reset() { _started = false; _sec = _exposure = _confirmed = _rx = 0; }
  bool ready() const { return _sec >= MIN_S && _confirmed >= MIN_CONFIRMED; }

  // Per-second rates; zeros until ready().
  TrialRates rates() const {
    TrialRates r;
    if (!ready()) return r;
    r.exposure = _exposure / _sec;
    r.confirmed = _confirmed / _sec;
    r.rx = _rx / _sec;
    return r;
  }

private:
  bool _started = false;
  uint32_t _t = 0, _e = 0, _c = 0, _r = 0;       // counters at the last fold
  double _sec = 0, _exposure = 0, _confirmed = 0, _rx = 0;   // decayed sums
};

class TrialPlanner {
public:
  static constexpr uint32_t MIN_BLOCK_N = 50;       // about 1 / c: counts at which a longer block stops helping
  static constexpr uint32_t MIN_BLOCK_S = 60;
  static constexpr uint32_t MAX_BLOCK_S = 3600;
  static constexpr uint32_t SETTLE_S = 10;          // a switching block opens its window this late
  static constexpr uint32_t SETTLE_MIN_BLOCK_S = 60;   // shorter blocks (bench tests) are not guarded
  static constexpr uint32_t MAX_TRIAL_S = 72u * 3600u;
  static constexpr uint32_t MAX_BLOCKS = 1000;
  static constexpr uint32_t MIN_PAIRS = 4;          // whole ABBA cycle, above TrialFSM::MIN_LOOK_PAIRS

  struct Input {
    TrialRates rates;
    const Objective *objective = nullptr;   // null = default weights
    uint8_t alpha_pct = 5;                  // error rate, 1 / 5 / 10
    uint8_t min_gain_pct = 5;
    uint8_t spread_pct = 17;                // relative std dev of a block the length does not average away
    double spread_c = -1;                   // >= 0: the block spread c itself (a running trial's estimate), over spread_pct
    uint16_t block_s = 0;                   // 0 = plan it
    uint16_t blocks = 0;                    // 0 = plan it
    uint32_t elapsed_s = 0;                 // a running trial: seconds and blocks already spent,
    uint32_t blocks_run = 0;                //   so the cap on its length counts them
  };

  enum Status : uint8_t { OK, NO_RATE, UNDERPOWERED };

  struct Plan {
    Status status = NO_RATE;
    uint16_t block_s = 0;
    uint16_t blocks = 0;
    uint8_t settle_s = 0;
    bool futility = false;          // the planner chose the pairs: project the final precision
    uint8_t spread_pct = 0;         // the block spread the plan used, sqrt(c) in percent (saturated)
    uint32_t pairs = 0;             // the pairs the counting noise and spread call for
    double v = 0;                   // counting noise of one block at the plan's rates
    uint32_t needed_confirmed_h = 0;   // UNDERPOWERED: confirmed deliveries per hour that would fit
  };

  // `c` for a block spread in percent.
  static double spreadC(uint8_t pct) { double s = pct / 100.0; return s * s; }
  static double spreadOf(const Input &in) { return in.spread_c >= 0 ? in.spread_c : spreadC(in.spread_pct); }
  static uint8_t spreadPct(double c) { double p = sqrt(c) * 100.0 + 0.5; return p > 255.0 ? 255 : (uint8_t)p; }

  // A running trial's block spread: the setting's `prior_pct` as the starting guess,
  // moved toward what the trial's own pairs show. c = var(d) / 2 - v (the variance of
  // a block less its counting noise) from MIN_ESTIMATE_PAIRS pairs on, weighted by
  // its pairs against PRIOR_PAIRS for the prior. The variance of d is taken around
  // its mean, so the estimate does not depend on the effect under test. Returns c.
  static constexpr uint32_t MIN_ESTIMATE_PAIRS = 4;
  static constexpr uint32_t PRIOR_PAIRS = 10;
  static double blendSpread(uint8_t prior_pct, const TrialFSM::Noise &z) {
    double prior = spreadC(prior_pct);
    if (z.pairs < MIN_ESTIMATE_PAIRS || z.blocks == 0) return prior;
    double seen = z.var_d / 2.0 - z.mean_v;
    if (seen < 0) seen = 0;
    return (PRIOR_PAIRS * prior + z.pairs * seen) / (PRIOR_PAIRS + z.pairs);
  }

  static Plan plan(const Input &in) {
    static const Objective defaults;
    const Objective &obj = in.objective ? *in.objective : defaults;
    Plan p;
    bool fixed_block = in.block_s != 0, fixed_blocks = in.blocks != 0;
    if (fixed_block && fixed_blocks) {   // today's fixed schedule, nothing planned
      p.status = OK;
      p.block_s = in.block_s;
      p.blocks = in.blocks;
      return p;
    }
    const TrialRates &r = in.rates;
    if (r.exposure <= 0 || r.confirmed <= 0) return p;   // NO_RATE
    // v for a block of T seconds is a / T.
    double a = obj.countingNoise(r.exposure, r.confirmed, r.rx);
    if (!isfinite(a) || a <= 0) return p;
    uint32_t T = in.block_s;
    if (!fixed_block) {
      double t = ceil(MIN_BLOCK_N * a);
      T = t < MIN_BLOCK_S ? MIN_BLOCK_S : t > MAX_BLOCK_S ? MAX_BLOCK_S : (uint32_t)t;
    }
    p.block_s = (uint16_t)T;
    p.settle_s = T >= SETTLE_MIN_BLOCK_S ? SETTLE_S : 0;
    p.v = a / T;
    p.spread_pct = spreadPct(spreadOf(in));
    p.pairs = pairsFor(p.v, in);
    uint32_t span = T + p.settle_s;
    uint32_t left_s = in.elapsed_s >= MAX_TRIAL_S ? 0 : MAX_TRIAL_S - in.elapsed_s;
    uint32_t cap = in.blocks_run + left_s / span;
    if (cap > MAX_BLOCKS) cap = MAX_BLOCKS;
    if (fixed_blocks) {   // the user chose the length: no feasibility check
      p.status = OK;
      p.blocks = in.blocks;
      return p;
    }
    p.futility = true;
    uint32_t blocks = 2 * p.pairs;
    if (blocks > cap) {
      p.status = UNDERPOWERED;
      p.blocks = (uint16_t)(cap & ~3u);
      p.needed_confirmed_h = neededConfirmedPerHour(r, p.v, cap / 2, in);
      return p;
    }
    p.status = OK;
    p.blocks = (uint16_t)blocks;
    return p;
  }

  // The schedule of a running trial from the traffic rates of its latest A blocks
  // (TrialFSM::aRates()): the blocks it needs now, up to the cap (UNDERPOWERED
  // then, with the capped blocks). `adapt`: the block length follows the rates too,
  // so a block keeps about MIN_BLOCK_N counts as the traffic changes; it moves only
  // when the length for these rates is STEP_PCT or more away from the current one,
  // so a noisy rate does not change it every cycle. false when the A blocks show no
  // confirmed deliveries, which says nothing to plan from.
  static constexpr uint32_t STEP_PCT = 15;
  static bool replan(Input in, const TrialRates &a_rates, uint16_t block_s, bool adapt, Plan &out) {
    in.rates = a_rates;
    in.blocks = 0;
    in.block_s = adapt ? 0 : block_s;
    out = plan(in);
    if (adapt && out.status != NO_RATE) {
      uint32_t d = out.block_s > block_s ? out.block_s - block_s : block_s - out.block_s;
      if (d * 100 < (uint32_t)block_s * STEP_PCT) {   // close enough: keep the length
        in.block_s = block_s;
        out = plan(in);
      }
    }
    return out.status != NO_RATE;
  }

  // Whole ABBA cycles of pairs for one block's counting noise v.
  static uint32_t pairsFor(double v, const Input &in) {
    double g = log(1.0 + in.min_gain_pct / 100.0);
    double var = v + spreadOf(in);
    double t = TrialFSM::tCrit(120, in.alpha_pct);
    uint32_t pairs = MIN_PAIRS;
    for (int i = 0; i < 4; i++) {   // t depends on the pairs it gives
      double need = ceil(2.0 * t * t * var / (g * g));
      pairs = need < MIN_PAIRS ? MIN_PAIRS : need > 1e6 ? 1000000u : (uint32_t)need;
      t = TrialFSM::tCrit(pairs - 1, in.alpha_pct);
    }
    return (pairs + 1) & ~1u;
  }

private:
  // Confirmed deliveries per hour that would let `pairs_max` pairs decide: the
  // rates scale together, so v scales as their inverse.
  static uint32_t neededConfirmedPerHour(const TrialRates &r, double v, uint32_t pairs_max, const Input &in) {
    double g = log(1.0 + in.min_gain_pct / 100.0);
    double t = TrialFSM::tCrit(pairs_max > 1 ? pairs_max - 1 : 1, in.alpha_pct);
    double v_ok = pairs_max * g * g / (2.0 * t * t) - spreadOf(in);
    if (v_ok <= 0) return UINT32_MAX;   // the block spread alone is too much
    double need = r.confirmed * 3600.0 * v / v_ok;
    return need > 4e9 ? UINT32_MAX : (uint32_t)ceil(need);
  }
};
