#pragma once

#include <math.h>
#include "MonRing.h"

// beebo: on-device, observe-only dynamic-tuning controller (Phase A of
// beebo/plans/DYNAMIC_OPTIMIZER_PLAN.md).
//
// Runs one small multi-armed bandit (UCB1) per tunable repeater parameter,
// over a fixed 3-arm neighbourhood {-step, 0 (stay), +step} around whatever
// the parameter's live value currently is. Only one parameter's bandit
// advances per tick (round-robin across TUNE_* -- see MonRing.h), so a
// reward is never confounded by two knobs moving at once; that's Phase B
// (SPSA joint perturbation)'s job, not this one's.
//
// The reward is txConfirmReward() -- a "TX reception confirmation" rate,
// replacing the original delivery-proxy reward (forwarded / (forwarded +
// no_forward + path_full)), which turned out to be structurally insensitive
// to nearly every one of these parameters: no_forward is an admin policy
// choice, path_full is a topology/hop-count ceiling, and "forwarded" itself
// only means a retransmission was *scheduled*, not that it was received by
// anyone. See beebo/plans/DYNAMIC_OPTIMIZER_PLAN.md item 9 for the full
// derivation. txConfirmReward() instead measures whether this node's own
// transmissions (self-originated or forwarded) actually got a confirmed
// response from the mesh -- a flood echo (SimpleMeshTables::getEchoSuccessCount())
// or a DM ACK (BaseChatMesh::getAckSuccessCount()/getAckTimeoutCount()) --
// blended into one rate:
//
//   reward = (ack_success + rpt) /
//            (ack_success + ack_timeout + echo_flood)
//
// This is a pure link-confirmation-quality RATIO ("confirm ratio" --
// DYNAMIC_OPTIMIZER_PLAN.md's "goodput" reward redesign, 2026-08-24), no
// longer widened by rx_drop (pool-exhausted/parse-error/queue-full) the way
// an earlier revision did -- that capacity signal is SoH's job (see
// MonRing::SohStats), and folding it into this ratio's denominator hid the
// effect of any parameter (radio txpower, RX/FEM gain) whose main effect is
// on routing VOLUME rather than this ratio: this reward alone is still
// structurally blind to a node routing 1 packet/hour at 100% vs. 1000/hour
// at 100%. MonRing::computeRos() is the companion raw-volume half;
// a combined goodput-style reward (ros_count, baseline-normalized,
// times this ratio) is the target shape once the decision-window gate's
// per-window deltas exist. This class still only computes the ratio half
// for now -- see the plan's Step 1 progress list for what's staged next.
//
// Both halves are proper 0/1-per-attempt indicators, not just any count that
// happened to be lying around:
// - ack_success/ack_timeout: exactly one outcome per DM attempt (onAckRecv()'s
//   match / txt_send_timeout's firing are mutually exclusive per send).
// - rpt: SimpleMeshTables tracks each flood-type self-transmission in a
//   small ring with a per-slot "already confirmed" flag, so no matter how
//   many neighbors independently echo the *same* original transmission,
//   it contributes at most 1 to echo_success_count -- an earlier draft of this
//   reward incremented echo_success_count once per echo *arrival* instead of
//   once per transmission, which made it an unbounded "mean echoes per TX"
//   that could exceed 100% and wasn't a valid probability to combine with
//   ack_success_rate at all. Fixed at the source (SimpleMeshTables.h), not
//   worked around here.
// - echo_flood: only flood-type self-transmissions (SimpleMeshTables::
//   getEchoAttemptCount()) -- direct/addressed sends are deliberately
//   excluded, since a direct packet structurally can't come back to us as a
//   flood echo and would otherwise inflate this denominator with zero
//   chance of ever moving the rpt half of the numerator, while still
//   being fully (and correctly, exactly once) covered by ack_success/
//   ack_timeout. An earlier draft pooled ALL self-transmissions here,
//   silently penalizing direct-only DM traffic relative to flood-routed
//   traffic for no reason connected to actual link quality.
//
// Both counters are lifetime totals (same "cumulative since boot" shape the
// original delivery-proxy histogram had), so the same caveat applies: early
// history dominates and the rate gets less responsive to recent conditions
// as the denominator grows over uptime.
//
// Every decision is logged as a MON_TUNE record. This class never touches
// NodePrefs/ComPrefs itself (no board/prefs access at all) -- it only
// returns a Decision{param_id, value, should_apply} for the caller
// (Beebo.cpp) to act on. A param only ever gets should_apply=true when its
// bit is set in the `applied_mask` passed into tick() (all off by default,
// see Beebo::_tune_applied_mask) -- Phase A ships with every param
// observe-only until individually promoted. `interference_threshold` is
// permanently excluded from should_apply regardless of its mask bit (see
// specFor()'s comment) since Beebo::getInterferenceThreshold() ignores the
// pref entirely today -- applying it would be a silent no-op.
//
// Rollback: after a live change on a param, its NEXT visit compares the
// freshly-read reward to the reward recorded just before that change; a
// drop past ROLLBACK_THRESHOLD reverts to the last known-good value
// (should_apply=true, proposed_value=last_good) instead of trying the
// bandit's next arm, and skips that arm's stats update for the tick (treated
// as neutral, neither reward nor visit-count evidence for the arm that
// caused the regression).
class TuneController {
public:
  static const int NUM_PARAMS = 6;
  static const int NUM_ARMS = 3;   // arm 0 = -step, 1 = stay, 2 = +step

  struct ParamSpec {
    uint8_t param_id;    // TUNE_* (MonRing.h)
    int16_t step;        // arm spacing, in the param's own fixed-point scale
    int16_t min_value;
    int16_t max_value;
  };

  // Fixed-point scales: rx_delay_base/tx_delay_factor/direct_tx_delay_factor/
  // airtime_factor are float NodePrefs fields, encoded here as value*100
  // (matches TuneRecord's int16_t fields); agc_reset_interval/
  // interference_threshold are already raw bytes on the wire (ComPrefs), no
  // scaling. Ranges mirror CommonCLI.cpp's own constrain()/CLI-enforced
  // bounds where one exists; agc_reset_interval/interference_threshold have
  // none documented upstream, so the full uint8_t range is used.
  //
  // A plain function building a local array (rather than a static class
  // member) sidesteps ODR-use/out-of-line-definition rules entirely for a
  // header-only class -- returned by value, six small structs, called once
  // per tick.
  static ParamSpec specFor(int p) {
    static const ParamSpec table[NUM_PARAMS] = {
      { TUNE_RX_DELAY_BASE,           100, 0, 2000 },  // 0.00 .. 20.00, step 1.00
      { TUNE_TX_DELAY_FACTOR,          20, 0,  200 },  // 0.00 ..  2.00, step 0.20
      { TUNE_DIRECT_TX_DELAY_FACTOR,   20, 0,  200 },  // 0.00 ..  2.00, step 0.20
      { TUNE_AGC_RESET_INTERVAL,        1, 0,  255 },  // raw byte (*4 = seconds)
      { TUNE_INTERFERENCE_THRESHOLD,    1, 0,    9 },  // raw byte -- range is 0-9,
                                                        // not the full uint8_t range:
                                                        // Beebo::tlvSetInterferenceThreshold
                                                        // (BeeboRepeater.cpp) silently
                                                        // clamps any raw > 9. NOTE:
                                                        // Beebo::getInterferenceThreshold()
                                                        // is separately hardcoded to
                                                        // return 0 regardless of this
                                                        // pref (Beebo.cpp) -- proposals
                                                        // for this param are logged but
                                                        // permanently excluded from
                                                        // should_apply (see isAppliable())
                                                        // until that's fixed upstream.
      { TUNE_AIRTIME_FACTOR,           50, 0,  900 },  // 0.00 ..  9.00, step 0.50
    };
    return table[p];
  }

  // interference_threshold's live path is currently dead (see specFor()'s
  // comment) -- excluded from actuation regardless of applied_mask, so a
  // caller can't accidentally "apply" a change that has zero real effect
  // and silently think it did something.
  static bool isAppliable(uint8_t param_id) {
    return param_id != TUNE_INTERFERENCE_THRESHOLD;
  }

  // Reward drop (0-10000 scale) past which a live-applied param reverts to
  // its last known-good value instead of trying the bandit's next arm.
  // 1500 = 15 percentage points -- a coarse, deliberately conservative
  // threshold (no data yet on real reward noise/variance for this mesh);
  // revisit once enough live-actuation history exists to tune it properly.
  static const int ROLLBACK_THRESHOLD = 1500;

  struct Decision {
    uint8_t param_id;
    int16_t value;         // value to write if should_apply, else undefined
    bool should_apply;
  };

  // Lifetime counters the caller reads fresh every tick (companion
  // BaseChatMesh::getAckSuccessCount()/getAckTimeoutCount(), mesh-wide
  // SimpleMeshTables::getEchoSuccessCount()/getEchoAttemptCount()) -- see
  // txConfirmReward() below for how they combine into the confirm-ratio
  // reward. echo_attempt_count deliberately excludes direct-routed self-tx
  // (see SimpleMeshTables::markSelfTx()) -- those are fully covered by
  // ack_success_count/ack_timeout_count instead.
  // No rx_drop_count field here anymore (removed in the goodput reward
  // redesign, DYNAMIC_OPTIMIZER_PLAN.md, 2026-08-24) -- capacity drops
  // (pool-exhausted/parse-error/queue-full) are SoH's job, not this ratio's;
  // folding them in here also hid the effect of volume-sensitive parameters
  // (radio txpower, RX/FEM gain) this ratio can't see regardless. Same
  // shape as MonRing::QosStats by construction (see txConfirmReward()).
  struct TxConfirmStats {
    uint32_t ack_success_count;
    uint32_t ack_timeout_count;
    uint32_t echo_attempt_count;
    uint32_t echo_success_count;
  };

  void begin() {
    for (int p = 0; p < NUM_PARAMS; p++) {
      _state[p] = ParamState{};
    }
    _next_param = 0;
  }

  // Called periodically (e.g. every few minutes) from the firmware's main
  // loop, repeater role only. `current_values[i]` must hold TUNE_*
  // (SPECS[i].param_id)'s live value, in that param's fixed-point scale --
  // the caller reads it from wherever that param actually lives (companion
  // NodePrefs, RAM-cached ComPrefs fields, or readComPrefsField()) since
  // that varies per parameter and this class has no board/prefs access.
  // `applied_mask` bit i = TUNE_* i (specFor(i).param_id) is promoted to
  // live actuation; 0 (all bits clear) reproduces the original fully
  // observe-only behaviour exactly. Returns the decision so the caller can
  // perform the actual write when should_apply is true -- this class never
  // does so itself.
  Decision tick(MonRing &ring, uint32_t now, const int16_t current_values[NUM_PARAMS],
                const TxConfirmStats &stats, uint8_t applied_mask = 0) {
    int p = _next_param;
    ParamSpec spec = specFor(p);
    ParamState &ps = _state[p];
    int16_t current = current_values[p];
    bool param_applied_enabled = isAppliable(spec.param_id) && (applied_mask & (1 << p)) != 0;

    uint16_t reward = txConfirmReward(stats);

    // Regression check: only meaningful if this param actually had a live
    // change applied on a previous visit (has_pending_live_change) -- an
    // observe-only-only history has nothing to roll back.
    bool rollback = ps.has_pending_live_change &&
                    (uint32_t)reward + ROLLBACK_THRESHOLD < (uint32_t)ps.reward_at_last_change;

    if (!rollback && ps.pending_arm >= 0) {
      ArmState &arm = ps.arms[ps.pending_arm];
      arm.pulls++;
      arm.reward_sum += reward;
    }

    int16_t proposed;
    bool should_apply = false;

    if (rollback) {
      proposed = ps.last_good_value;
      should_apply = true;
      ps.has_pending_live_change = false;
      ps.pending_arm = -1;   // reverted -- no arm-stats update from this tick
    } else {
      int chosen = chooseArm(ps);
      ps.total_pulls++;
      ps.pending_arm = chosen;

      int16_t offset = (int16_t)(chosen - 1) * spec.step;  // arm 0/1/2 -> -step/0/+step
      proposed = current + offset;
      if (proposed < spec.min_value) proposed = spec.min_value;
      if (proposed > spec.max_value) proposed = spec.max_value;

      if (param_applied_enabled && proposed != current) {
        should_apply = true;
        // Always the value right before THIS change, not just the first-ever
        // one: `current` at this point is either the pre-trial value (no
        // prior live change) or a value already confirmed stable (rollback
        // above would have fired instead of reaching here if the previous
        // change had regressed) -- so a later rollback reverts to the most
        // recent known-good value, not all the way back to where the param
        // started.
        ps.last_good_value = current;
        ps.has_last_good = true;
        ps.reward_at_last_change = reward;
        ps.has_pending_live_change = true;
      }
    }

    TuneRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.param_id = spec.param_id;
    rec.applied = should_apply ? 1 : 0;
    rec.old_value = current;
    rec.proposed_value = proposed;
    rec.reward_before = reward;
    ring.appendTune(rec, now);

    _next_param = (uint8_t)((p + 1) % NUM_PARAMS);

    Decision d;
    d.param_id = spec.param_id;
    d.value = proposed;
    d.should_apply = should_apply;
    return d;
  }

private:
  struct ArmState {
    uint32_t pulls = 0;
    float reward_sum = 0.0f;
  };
  struct ParamState {
    ArmState arms[NUM_ARMS];
    uint32_t total_pulls = 0;
    int8_t pending_arm = -1;   // arm proposed on this param's previous visit, -1 = none yet
    bool has_last_good = false;
    int16_t last_good_value = 0;        // value before the first-ever live change to this param
    uint16_t reward_at_last_change = 0;  // reward recorded just before the most recent live change
    bool has_pending_live_change = false; // true while a live change hasn't yet been confirmed/rolled back
  };

  ParamState _state[NUM_PARAMS];
  uint8_t _next_param = 0;

  // TX reception confirmation reward, scaled to the TuneRecord.reward_before
  // wire range (0-10000 = 0-100%) -- see the class-level comment above for
  // the derivation and known limitations. This IS the QoS objective function
  // (DYNAMIC_OPTIMIZER_PLAN.md item 10) -- delegates to MonRing::computeQos()
  // so there is exactly one implementation of this formula, not two that
  // could silently drift apart (TxConfirmStats/MonRing::QosStats are the
  // same shape by construction).
  static uint16_t txConfirmReward(const TxConfirmStats &s) {
    MonRing::QosStats qs{s.ack_success_count, s.ack_timeout_count,
                          s.echo_attempt_count, s.echo_success_count};
    return MonRing::computeQos(qs);
  }

  // UCB1: try every never-pulled arm first, then argmax(mean + sqrt(2 ln(N)/n)).
  static int chooseArm(const ParamState &ps) {
    for (int a = 0; a < NUM_ARMS; a++) {
      if (ps.arms[a].pulls == 0) return a;
    }
    int best = 0;
    float best_score = -1.0f;
    float logN = logf((float)(ps.total_pulls + 1));
    for (int a = 0; a < NUM_ARMS; a++) {
      const ArmState &arm = ps.arms[a];
      float mean = arm.reward_sum / (float)arm.pulls;
      float bonus = sqrtf(2.0f * logN / (float)arm.pulls);
      float score = mean + bonus;
      if (score > best_score) {
        best_score = score;
        best = a;
      }
    }
    return best;
  }
};
