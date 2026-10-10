#pragma once

#include <math.h>
#include <stdint.h>
#include "Objective.h"
#include "TrialFSM.h"

// beebo: the rate-adaptive schedule of an on-device ABBA trial (plans/
// TRIAL_AUTO_SCHEDULE.md). Pure logic, no hardware, no allocation.
//
// ExposureTracker: what the node's traffic looks like, from the same counters
// EvalWindow reads, kept in a static ring of 30 minute slots so a plan can start
// at any time and a quiet hour does not set the rates.
//
// TrialPlanner: from those rates and the objective's weights, the block length
// and the number of blocks a trial needs so its confidence bound can reach
// min_gain_pct. A block's variance is the counting noise v of its confirmed
// deliveries plus a block spread c the length does not average away; a pair
// of blocks differs with variance 2 (v + c), so the pairs needed are
//   P = 2 t^2 (v + c) / g^2        (t = Student-t at the confidence, g = ln(1 + min gain))
// A node whose rates cannot get there inside MAX_TRIAL_S is underpowered.

// Traffic counters, per second.
struct TrialRates {
  double exposure = 0;    // routed attempts
  double confirmed = 0;   // confirmed deliveries
  double rx = 0;          // valid packets received
};

class ExposureTracker {
public:
  static constexpr uint32_t SLOT_S = 1800;
  static constexpr uint8_t SLOTS = 48;        // 24 h
  static constexpr uint8_t MIN_SLOTS = 2;     // slots before rates() is trusted
  static constexpr uint8_t QUANTILE_PCT = 25; // the slot ranked this far up by confirmed count

  // Feed the lifetime counters every loop tick. A counter that went backwards
  // (device-side reset) clears the ring.
  void poll(uint32_t now_ms, uint32_t exposure, uint32_t confirmed, uint32_t rx) {
    if (!_started) { rebase(now_ms, exposure, confirmed, rx); return; }
    uint32_t d_e = exposure - _e0, d_c = confirmed - _c0, d_r = rx - _r0;
    if (d_e > 0x80000000u || d_c > 0x80000000u || d_r > 0x80000000u) {
      reset();
      rebase(now_ms, exposure, confirmed, rx);
      return;
    }
    if (now_ms - _t0 < SLOT_S * 1000u) return;
    Slot &s = _slots[_next];
    s.exposure = sat16(d_e);
    s.confirmed = sat16(d_c);
    s.rx = sat16(d_r);
    _next = (uint8_t)((_next + 1) % SLOTS);
    if (_filled < SLOTS) _filled++;
    rebase(now_ms, exposure, confirmed, rx);
  }

  void reset() { _started = false; _next = 0; _filled = 0; }
  uint8_t filled() const { return _filled; }
  bool ready() const { return _filled >= MIN_SLOTS; }

  // The slot at the low quantile of the confirmed count, as per-second rates:
  // a quiet part of the day, with its own exposure and rx so the confirm ratio
  // stays consistent. Zeros until ready().
  TrialRates rates() const {
    TrialRates r;
    if (!ready()) return r;
    uint8_t idx[SLOTS];
    for (uint8_t i = 0; i < _filled; i++) idx[i] = i;
    for (uint8_t i = 1; i < _filled; i++) {   // insertion sort by confirmed count
      uint8_t k = idx[i];
      int j = i - 1;
      while (j >= 0 && _slots[idx[j]].confirmed > _slots[k].confirmed) { idx[j + 1] = idx[j]; j--; }
      idx[j + 1] = k;
    }
    const Slot &s = _slots[idx[(uint32_t)(_filled - 1) * QUANTILE_PCT / 100]];
    r.exposure = (double)s.exposure / SLOT_S;
    r.confirmed = (double)s.confirmed / SLOT_S;
    r.rx = (double)s.rx / SLOT_S;
    return r;
  }

private:
  struct Slot { uint16_t exposure = 0, confirmed = 0, rx = 0; };
  static uint16_t sat16(uint32_t v) { return (uint16_t)(v > 0xFFFF ? 0xFFFF : v); }
  void rebase(uint32_t now_ms, uint32_t e, uint32_t c, uint32_t r) {
    _started = true; _t0 = now_ms; _e0 = e; _c0 = c; _r0 = r;
  }
  Slot _slots[SLOTS];
  uint8_t _next = 0, _filled = 0;
  bool _started = false;
  uint32_t _t0 = 0, _e0 = 0, _c0 = 0, _r0 = 0;
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
    uint16_t block_s = 0;                   // 0 = plan it
    uint16_t blocks = 0;                    // 0 = plan it
  };

  enum Status : uint8_t { OK, NO_RATE, UNDERPOWERED };

  struct Plan {
    Status status = NO_RATE;
    uint16_t block_s = 0;
    uint16_t blocks = 0;
    uint8_t settle_s = 0;
    bool futility = false;          // the planner chose the pairs: project the final precision
    uint32_t pairs = 0;             // the pairs the counting noise and spread call for
    double v = 0;                   // counting noise of one block at the plan's rates
    uint32_t needed_confirmed_h = 0;   // UNDERPOWERED: confirmed deliveries per hour that would fit
  };

  // `c` for a block spread in percent.
  static double spreadC(uint8_t pct) { double s = pct / 100.0; return s * s; }

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
    p.pairs = pairsFor(p.v, in);
    uint32_t span = T + p.settle_s;
    uint32_t cap = MAX_TRIAL_S / span;
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

  // Whole ABBA cycles of pairs for one block's counting noise v.
  static uint32_t pairsFor(double v, const Input &in) {
    double g = log(1.0 + in.min_gain_pct / 100.0);
    double var = v + spreadC(in.spread_pct);
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
    double v_ok = pairs_max * g * g / (2.0 * t * t) - spreadC(in.spread_pct);
    if (v_ok <= 0) return UINT32_MAX;   // the block spread alone is too much
    double need = r.confirmed * 3600.0 * v / v_ok;
    return need > 4e9 ? UINT32_MAX : (uint32_t)ceil(need);
  }
};
