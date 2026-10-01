#pragma once

#include <math.h>
#include "MonRing.h"
#include "EvalWindow.h"
#include "Objective.h"

// On-device, observe-only dynamic-tuning controller.
//
// Runs one small multi-choice bandit (UCB1) per tunable repeater parameter,
// over a fixed 3-step neighborhood {-step, 0 (stay), +step} around whatever
// the parameter's live value currently is. Only one parameter's bandit
// advances per tick (round-robin across TUNING_* -- see MonRing.h), so a
// reward is never confounded by two knobs moving at once; that's Phase B
// (SPSA joint perturbation)'s job, not this one's.
//
// One decision is evaluated per closed EvalWindow (see EvalWindow.h): tick()
// receives that window's Result, so a change is judged only by the window
// that immediately follows it, never by a lifetime total or by a revisit
// several ticks later.
//
// Per window, in order:
//  1. Unmeasured (insufficient data, invalidated, baseline still filling):
//     emit the MON_EVAL record and hold -- no step update, no rollback, no
//     proposal, round-robin does not advance.
//  2. Measured: judge the pending decision. A live change rolls back if the
//     window's confirm ratio fell past ROLLBACK_THRESHOLD below the ratio of
//     the window before the change, or the utilization guardrail tripped.
//     Otherwise it is accepted and the step learns the window's goodput
//     (ros_norm x confirm_ratio, capped) -- volume only ever enters here,
//     accumulated across windows, never a single window's rollback call.
//  3. Emit the MON_EVAL record (accepted/rollback), then either revert (the
//     rollback tick proposes nothing new) or propose the next param.
//
// Every proposal is a MON_ADAPTIVE record carrying `iteration`, the id of the
// window that will evaluate it. This class never touches NodePrefs/ComPrefs
// itself -- it only returns a Decision for the caller (Beebo.cpp) to act on.
// A param only gets should_apply=true when its bit is set in `applied_mask`
// (all off by default, see Beebo::_adaptive_applied_mask); every current param
// is applicable (isApplicable()).
//
// The bandit's steps are UCB1 over goodput in the 0-20000 range, so the
// exploration bonus (order 1) is negligible next to reward differences: the
// selection is effectively greedy after each step's first pull.
class AdaptiveController {
public:
  static const int NUM_PARAMS = 6;
  static const int NUM_STEPS = 3;   // step 0 = -step, 1 = stay, 2 = +step

  struct ParamSpec {
    uint8_t param_id;    // TUNING_* (MonRing.h)
    int16_t step;        // spacing between adjacent steps, in the param's own fixed-point scale
    int16_t min_value;
    int16_t max_value;
  };

  // Fixed-point scales: rx_delay_base/tx_delay_factor/direct_tx_delay_factor/
  // airtime_factor are float NodePrefs fields, encoded here as value*100
  // (matches AdaptiveRecord's int16_t fields); agc_reset_interval/
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
      { TUNING_RX_DELAY_BASE,           100, 0, 2000 },  // 0.00 .. 20.00, step 1.00
      { TUNING_TX_DELAY_FACTOR,          20, 0,  200 },  // 0.00 ..  2.00, step 0.20
      { TUNING_DIRECT_TX_DELAY_FACTOR,   20, 0,  200 },  // 0.00 ..  2.00, step 0.20
      { TUNING_AGC_RESET_INTERVAL,        1, 0,  255 },  // raw byte (*4 = seconds)
      { TUNING_INTERFERENCE_THRESHOLD,    1, 0,    9 },  // raw byte -- range is 0-9,
                                                        // not the full uint8_t range:
                                                        // Beebo::tlvSetInterferenceThreshold
                                                        // (BeeboRepeater.cpp) silently
                                                        // clamps any raw > 9. Live-appliable
                                                        // since Beebo::getInterferenceThreshold()
                                                        // reads the real pref (was
                                                        // permanently hardcoded to 0; fixed
                                                        // upstream, meshcore-dev/MeshCore@
                                                        // 5d82ed35, applied directly here).
      { TUNING_AIRTIME_FACTOR,           50, 0,  900 },  // 0.00 ..  9.00, step 0.50
    };
    return table[p];
  }

  // Every current param can now take live effect (see specFor()'s
  // interference_threshold comment for the one that couldn't until
  // recently). Kept as a real function, not `return true;`, so a future
  // param that genuinely can't be applied yet has an obvious place to add
  // its exclusion, the way interference_threshold's used to live here.
  static bool isApplicable(uint8_t param_id) {
    (void)param_id;
    return true;
  }

  // Reward drop (0-10000 scale) past which a live-applied param reverts to
  // its last known-good value instead of trying the bandit's next step.
  // 1500 = 15 percentage points -- a coarse, deliberately conservative
  // threshold (no data yet on real reward noise/variance for this mesh);
  // revisit once enough live-actuation history exists to tune it properly.
  static const int ROLLBACK_THRESHOLD = 1500;

  struct Decision {
    uint8_t param_id;
    int16_t value;         // value to write if should_apply, else undefined
    bool should_apply;
    bool hold;             // unmeasured window: nothing proposed, param_id/value undefined
    uint16_t window_id;    // id to open the next EvalWindow with
  };

  // Step reward is capped so one burst window cannot dominate a step's mean.
  static const uint32_t GOODPUT_CAP = 20000;

  void begin() {
    for (int p = 0; p < NUM_PARAMS; p++) {
      _state[p] = ParamState{};
    }
    _next_param = 0;
    _pending_param = -1;
    _window_id = 1;
  }

  // True when no decision is awaiting its window's verdict (safe to run a
  // TrialFSM trial without confounding a pending live change).
  bool idle() const { return _pending_param < 0; }

  // Score windows with the shared objective instead of plain goodput; the
  // controller keeps the pointer (the caller owns it and its weights).
  void setObjective(Objective *o) { _objective = o; }

  // Id of the window currently open (or about to be opened).
  uint16_t windowId() const { return _window_id; }
  uint32_t stepPulls(int p, int s) const { return _state[p].steps[s].pulls; }
  float stepRewardSum(int p, int s) const { return _state[p].steps[s].reward_sum; }

  // Called once per closed EvalWindow, repeater role only. `current_values[i]`
  // must hold TUNING_* (specFor(i).param_id)'s live value, in that param's
  // fixed-point scale -- the caller reads it from wherever that param actually
  // lives (companion NodePrefs, RAM-cached ComPrefs fields, or
  // readComPrefsField()) since that varies per parameter and this class has no
  // board/prefs access. `applied_mask` bit i promotes TUNING_* i to live
  // actuation; 0 reproduces fully observe-only behavior exactly. Returns the
  // decision so the caller can perform the actual write when should_apply is
  // true and open the next window with `window_id`.
  Decision tick(MonRing &ring, uint64_t now, const int16_t current_values[NUM_PARAMS],
                const EvalWindow::Result &window, uint8_t applied_mask = 0,
                bool propose = true) {
    uint16_t closing_id = _window_id;
    _window_id++;

    Decision d = {};
    d.window_id = _window_id;

    if (!window.measured) {
      emitEval(ring, now, window, closing_id, window.outcome);
      d.hold = true;
      return d;
    }

    // Judge the decision the previous tick left pending, if any.
    bool rollback = false;
    int q = _pending_param;
    if (q >= 0) {
      ParamState &qs = _state[q];
      if (qs.has_pending_live_change) {
        rollback = (uint32_t)window.confirm_ratio + ROLLBACK_THRESHOLD <
                       (uint32_t)qs.reward_at_last_change ||
                   (window.flags & EVALF_GUARDRAIL);
      }
      if (!rollback && qs.pending_step >= 0) {
        StepState &st = qs.steps[qs.pending_step];
        st.pulls++;
        st.reward_sum += reward(window);
      }
      qs.pending_step = -1;
      qs.has_pending_live_change = false;
      _pending_param = -1;
    }

    emitEval(ring, now, window, closing_id, rollback ? EVAL_ROLLBACK : EVAL_ACCEPTED);

    if (rollback) {
      ParamSpec spec = specFor(q);
      d.param_id = spec.param_id;
      d.value = _state[q].last_good_value;
      d.should_apply = true;
      appendAdaptiveRecord(ring, now, spec.param_id, true, current_values[q], d.value,
                       window.confirm_ratio);
      return d;
    }

    // `propose` false: judge the pending decision above but leave nothing
    // pending, so idle() turns true and the caller can start a TrialFSM trial.
    if (!propose) {
      d.hold = true;
      return d;
    }

    int p = _next_param;
    ParamSpec spec = specFor(p);
    ParamState &ps = _state[p];
    int16_t current = current_values[p];
    bool param_applied_enabled = isApplicable(spec.param_id) && (applied_mask & (1 << p)) != 0;

    int chosen = chooseStep(ps);
    ps.total_pulls++;
    ps.pending_step = chosen;

    int16_t offset = (int16_t)(chosen - 1) * spec.step;  // step 0/1/2 -> -step/0/+step
    int16_t proposed = current + offset;
    if (proposed < spec.min_value) proposed = spec.min_value;
    if (proposed > spec.max_value) proposed = spec.max_value;

    bool should_apply = false;
    if (param_applied_enabled && proposed != current) {
      should_apply = true;
      // `current` is either the pre-trial value or one already accepted as
      // stable (a regression would have rolled back above), so a later
      // rollback reverts to the most recent known-good value.
      ps.last_good_value = current;
      ps.has_last_good = true;
      ps.reward_at_last_change = window.confirm_ratio;
      ps.has_pending_live_change = true;
    }
    _pending_param = (int8_t)p;
    appendAdaptiveRecord(ring, now, spec.param_id, should_apply, current, proposed,
                     window.confirm_ratio);

    _next_param = (uint8_t)((p + 1) % NUM_PARAMS);

    d.param_id = spec.param_id;
    d.value = proposed;
    d.should_apply = should_apply;
    return d;
  }

private:
  struct StepState {
    uint32_t pulls = 0;
    float reward_sum = 0.0f;
  };
  struct ParamState {
    StepState steps[NUM_STEPS];
    uint32_t total_pulls = 0;
    int8_t pending_step = -1;   // step proposed on this param's previous visit, -1 = none yet
    bool has_last_good = false;
    int16_t last_good_value = 0;        // value before the first-ever live change to this param
    uint16_t reward_at_last_change = 0;  // reward recorded just before the most recent live change
    bool has_pending_live_change = false; // true while a live change hasn't yet been confirmed/rolled back
  };

  ParamState _state[NUM_PARAMS];
  Objective *_objective = nullptr;   // null = plain goodput
  uint8_t _next_param = 0;
  int8_t _pending_param = -1;   // param proposed by the previous tick, awaiting its window's verdict
  uint16_t _window_id = 1;

  // ros_norm (1000 = baseline volume) x confirm ratio (0-10000), capped.
  // A window's step reward: the shared objective (Objective.h) when one is
  // attached, else plain goodput; capped either way.
  float reward(const EvalWindow::Result &w) {
    if (!_objective) return goodput(w);
    return _objective->banditReward(w, (float)GOODPUT_CAP);
  }

  static float goodput(const EvalWindow::Result &w) {
    uint64_t g = (uint64_t)w.ros_norm * w.confirm_ratio / 1000;
    return (float)(g > GOODPUT_CAP ? GOODPUT_CAP : g);
  }

  static void emitEval(MonRing &ring, uint64_t now, const EvalWindow::Result &w,
                       uint16_t window_id, uint8_t outcome) {
    EvalRecordA a;
    EvalRecordB b;
    EvalRecordC c;
    EvalWindow::toRecords(w, window_id, outcome, a, b, c);
    ring.appendEval(a, b, c, now);
  }

  void appendAdaptiveRecord(MonRing &ring, uint64_t now, uint8_t param_id, bool applied,
                        int16_t old_value, int16_t proposed, uint16_t ratio) const {
    AdaptiveRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.param_id = param_id;
    rec.applied = applied ? 1 : 0;
    rec.old_value = old_value;
    rec.proposed_value = proposed;
    rec.reward_before = ratio;
    rec.iteration = _window_id;   // already advanced: the window that will evaluate this
    ring.appendAdaptive(rec, now);
  }

  // UCB1: try every never-pulled step first, then argmax(mean + sqrt(2 ln(N)/n)).
  static int chooseStep(const ParamState &ps) {
    for (int s = 0; s < NUM_STEPS; s++) {
      if (ps.steps[s].pulls == 0) return s;
    }
    int best = 0;
    float best_score = -1.0f;
    float logN = logf((float)(ps.total_pulls + 1));
    for (int s = 0; s < NUM_STEPS; s++) {
      const StepState &st = ps.steps[s];
      float mean = st.reward_sum / (float)st.pulls;
      float bonus = sqrtf(2.0f * logN / (float)st.pulls);
      float score = mean + bonus;
      if (score > best_score) {
        best_score = score;
        best = s;
      }
    }
    return best;
  }
};
