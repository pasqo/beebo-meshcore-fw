#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <vector>
#include <helpers/AdaptiveController.h>

namespace {

template <uint32_t N>
struct RingFixture {
  MonRecord buf[N];
  MonRing ring;
  uint32_t seed;
  RingFixture(uint32_t now = 1000) : seed(now) {
    ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), now, now, RadioRecord{}, EnvRecord{});
    ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
  }

  // beebo: AdaptiveController::tick()'s `now` (like every MonRing append*())
  // takes an already-resolved epoch-ms instant directly now, not a raw
  // millis() reading -- see test_monring.cpp's own RingFixture::ms() for
  // the full rationale; identical helper, duplicated here since this
  // suite has its own small fixture rather than sharing one.
  uint64_t ms(uint32_t millis_like) const {
    return (uint64_t)seed * 1000 + (uint64_t)(millis_like - seed);
  }
};

// A closed EvalWindow::Result as AdaptiveController::tick() receives it. `ratio`
// is the window's confirm ratio (0-10000), `ros_norm` its volume against the
// baseline (1000 = baseline); a real EvalWindow computes both from deltas.
EvalWindow::Result measured(uint16_t ratio, uint16_t ros_norm = 1000, uint8_t flags = 0) {
  EvalWindow::Result r = {};
  r.measured = true;
  r.outcome = EVAL_ACCEPTED;
  r.confirm_ratio = ratio;
  r.ros_norm = ros_norm;
  r.flags = flags;
  r.exposure = 50;
  r.window_ms = 300000;
  return r;
}

EvalWindow::Result unmeasured(uint8_t outcome, uint8_t flags = 0) {
  EvalWindow::Result r = {};
  r.measured = false;
  r.outcome = outcome;
  r.flags = flags;
  return r;
}

std::vector<MonRecord> readAll(MonRing &ring) {
  std::vector<MonRecord> out(ring.count());
  uint32_t returned = 0;
  ring.serialize(reinterpret_cast<uint8_t *>(out.data()), out.size() * sizeof(MonRecord), 0, &returned);
  out.resize(returned);
  return out;
}

std::vector<MonRecord> ofKind(MonRing &ring, uint8_t kind) {
  std::vector<MonRecord> v;
  for (const MonRecord &r : readAll(ring)) {
    if ((r.kind & RLOG_KIND_MASK) == kind) v.push_back(r);
  }
  return v;
}

const int16_t kZeros[AdaptiveController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};

}  // namespace

TEST(AdaptiveController, TicksThroughAllParamsRoundRobinOneAdaptiveRecordEach) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();

  for (int i = 0; i < AdaptiveController::NUM_PARAMS; i++) {
    tc.tick(f.ring, f.ms(1000 + i), kZeros, measured(9000));
  }

  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_EQ((size_t)AdaptiveController::NUM_PARAMS, tunes.size());
  for (int i = 0; i < AdaptiveController::NUM_PARAMS; i++) {
    EXPECT_EQ((uint8_t)i, tunes[i].adaptive.param_id);
    EXPECT_EQ(0, tunes[i].adaptive.applied);  // observe-only -- never set
  }
}

TEST(AdaptiveController, ProposedValueStaysWithinSpecRangeAtLowerBound) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();

  // rx_delay_base (param 0) at its floor: every step must clamp to [0, 2000].
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));

  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_GE(tunes.size(), 1u);
  EXPECT_EQ(TUNING_RX_DELAY_BASE, tunes[0].adaptive.param_id);
  EXPECT_GE(tunes[0].adaptive.proposed_value, 0);
  EXPECT_LE(tunes[0].adaptive.proposed_value, 2000);
}

TEST(AdaptiveController, NeverSetsAppliedRegardlessOfRewardHistory) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();

  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 20, 200};
  for (int i = 0; i < 30; i++) {
    tc.tick(f.ring, f.ms(2000 + i), current, measured(10000));
  }
  for (const MonRecord &r : ofKind(f.ring, MON_ADAPTIVE)) EXPECT_EQ(0, r.adaptive.applied);
}

TEST(AdaptiveController, AdaptiveRecordRewardBeforeIsTheClosedWindowsConfirmRatio) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();

  tc.tick(f.ring, f.ms(1010), kZeros, measured(7500));

  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(7500, tunes[0].adaptive.reward_before);
}

TEST(AdaptiveController, EmitsEvalRunThenAdaptiveRecordSharingWindowIds) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();
  uint16_t closing = tc.windowId();

  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));

  auto all = readAll(f.ring);
  ASSERT_EQ(4u, all.size());
  EXPECT_EQ(MON_EVAL | RLOG_CONT_BIT, all[0].kind);
  EXPECT_EQ(MON_EVAL | RLOG_CONT_BIT, all[1].kind);
  EXPECT_EQ(MON_EVAL, all[2].kind);
  EXPECT_EQ(MON_ADAPTIVE, all[3].kind);
  EXPECT_EQ(closing, all[0].eval_a.window_id);
  EXPECT_EQ(EVAL_ACCEPTED, all[0].eval_a.outcome);
  EXPECT_EQ(9000, all[0].eval_a.confirm_ratio);
  // the new decision is evaluated by the NEXT window
  EXPECT_EQ((uint16_t)(closing + 1), d.window_id);
  EXPECT_EQ(d.window_id, all[3].adaptive.iteration);
  EXPECT_EQ(d.window_id, tc.windowId());
}

TEST(AdaptiveController, UnmeasuredWindowEmitsEvalOnlyAndHolds) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();
  uint16_t closing = tc.windowId();

  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1000), kZeros,
                                       unmeasured(EVAL_INSUFFICIENT_DATA), 0xFF);

  EXPECT_TRUE(d.hold);
  EXPECT_FALSE(d.should_apply);
  EXPECT_EQ((uint16_t)(closing + 1), d.window_id);
  auto all = readAll(f.ring);
  ASSERT_EQ(3u, all.size());  // MON_EVAL run (3 slots) only, no MON_ADAPTIVE
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, all[0].eval_a.outcome);
  EXPECT_EQ(0u, ofKind(f.ring, MON_ADAPTIVE).size());
}

TEST(AdaptiveController, UnmeasuredWindowDoesNotAdvanceRoundRobinOrStepStats) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();
  tc.tick(f.ring, f.ms(1000), kZeros, unmeasured(EVAL_INSUFFICIENT_DATA));
  tc.tick(f.ring, f.ms(1001), kZeros, unmeasured(EVAL_INVALIDATED, EVALF_REBOOT));

  tc.tick(f.ring, f.ms(1002), kZeros, measured(9000));
  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(TUNING_RX_DELAY_BASE, tunes[0].adaptive.param_id);  // still the first param
  for (int a = 0; a < AdaptiveController::NUM_STEPS; a++) {
    EXPECT_EQ(0u, tc.stepPulls(0, a));
  }
}

TEST(AdaptiveController, AppliedMaskBitEnablesLiveActuationDecision) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();

  int16_t current[AdaptiveController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 0;  // TUNING_RX_DELAY_BASE only

  // First-ever visit to param 0: chooseStep() picks the first never-pulled
  // step (index 0 = -step), so the proposal is deterministic.
  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1000), current, measured(9000), mask);
  EXPECT_EQ(TUNING_RX_DELAY_BASE, d.param_id);
  EXPECT_EQ(900, d.value);          // 1000 - step(100)
  EXPECT_TRUE(d.should_apply);
  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(1, tunes[0].adaptive.applied);
}

TEST(AdaptiveController, AppliedMaskBitClearLeavesDecisionObserveOnly) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();

  int16_t current[AdaptiveController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1000), current, measured(9000));
  EXPECT_EQ(900, d.value);
  EXPECT_FALSE(d.should_apply);
}

TEST(AdaptiveController, InterferenceThresholdAppliesWhenMasked) {
  // Beebo::getInterferenceThreshold() used to be permanently hardcoded to 0
  // (upstream bug, fixed directly here), so this param's proposals could
  // never take live effect regardless of applied_mask; that exclusion is
  // gone now that the getter reads the real pref -- isApplicable() no
  // longer treats any param specially.
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();

  int16_t current[AdaptiveController::NUM_PARAMS] = {0, 0, 0, 0, 5, 0};
  uint8_t mask = 0xFF;  // every bit set, including TUNING_INTERFERENCE_THRESHOLD's

  AdaptiveController::Decision d;
  for (int i = 0; i < 5; i++) {   // params 0..4 visited in order; index 4 = interference_threshold
    d = tc.tick(f.ring, f.ms(1000 + i), current, measured(9000), mask);
  }
  EXPECT_EQ(TUNING_INTERFERENCE_THRESHOLD, d.param_id);
  EXPECT_TRUE(d.should_apply);
  EXPECT_TRUE(AdaptiveController::isApplicable(TUNING_INTERFERENCE_THRESHOLD));
}

// Params are visited in order, so the tx_delay_factor (param 1) proposal is
// the SECOND tick's decision. Helper: apply param 0 unmasked (no live change),
// then the masked live change on param 1; returns that decision.
static AdaptiveController::Decision liveChangeOnParam1(RingFixture<256> &f, AdaptiveController &tc,
                                                   int16_t current[AdaptiveController::NUM_PARAMS],
                                                   uint8_t mask, uint16_t ratio_before) {
  tc.tick(f.ring, f.ms(1010), current, measured(ratio_before), mask);          // param 0
  return tc.tick(f.ring, f.ms(1011), current, measured(ratio_before), mask);   // param 1
}

TEST(AdaptiveController, RegressionInTheNextWindowRollsBackImmediately) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;  // TUNING_TX_DELAY_FACTOR only

  AdaptiveController::Decision d1 = liveChangeOnParam1(f, tc, current, mask, 10000);
  ASSERT_EQ(TUNING_TX_DELAY_FACTOR, d1.param_id);
  ASSERT_EQ(30, d1.value);
  ASSERT_TRUE(d1.should_apply);

  // The very next window (not six ticks later) shows a collapse: 10000 -> 1428.
  AdaptiveController::Decision d2 = tc.tick(f.ring, f.ms(1020), current, measured(1428), mask);
  EXPECT_EQ(TUNING_TX_DELAY_FACTOR, d2.param_id);
  EXPECT_EQ(50, d2.value);   // reverted to the pre-change value
  EXPECT_TRUE(d2.should_apply);

  auto evals = ofKind(f.ring, MON_EVAL);
  ASSERT_GE(evals.size(), 3u);
  const MonRecord &last_a = evals[evals.size() - 3];
  EXPECT_EQ(EVAL_ROLLBACK, last_a.eval_a.outcome);
}

TEST(AdaptiveController, GuardrailTripInTheNextWindowRollsBackEvenWithHealthyRatio) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  ASSERT_TRUE(liveChangeOnParam1(f, tc, current, mask, 10000).should_apply);
  AdaptiveController::Decision d2 = tc.tick(f.ring, f.ms(1020), current,
                                        measured(10000, 1000, EVALF_GUARDRAIL), mask);
  EXPECT_EQ(50, d2.value);
  EXPECT_TRUE(d2.should_apply);
}

TEST(AdaptiveController, RollbackTickProposesNothingNew) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  liveChangeOnParam1(f, tc, current, mask, 10000);
  size_t tunes_before = ofKind(f.ring, MON_ADAPTIVE).size();
  tc.tick(f.ring, f.ms(1020), current, measured(1428), mask);   // rollback
  auto tunes = ofKind(f.ring, MON_ADAPTIVE);
  ASSERT_EQ(tunes_before + 1, tunes.size());                    // only the revert record
  EXPECT_EQ(1, tunes.back().adaptive.applied);
  EXPECT_EQ(50, tunes.back().adaptive.proposed_value);

  // the next measured window resumes the round-robin at param 2
  AdaptiveController::Decision d3 = tc.tick(f.ring, f.ms(1030), current, measured(9000), mask);
  EXPECT_EQ(TUNING_DIRECT_TX_DELAY_FACTOR, d3.param_id);
}

TEST(AdaptiveController, IdleOnlyWhileNoDecisionAwaitsItsWindow) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  EXPECT_TRUE(tc.idle());
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));   // proposes param 0
  EXPECT_FALSE(tc.idle());
  tc.tick(f.ring, f.ms(1001), kZeros, unmeasured(EVAL_INSUFFICIENT_DATA));   // still pending
  EXPECT_FALSE(tc.idle());
}

TEST(AdaptiveController, StableWindowAfterLiveChangeIsAcceptedAndUpdatesStepWithGoodput) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  AdaptiveController::Decision d1 = liveChangeOnParam1(f, tc, current, mask, 10000);
  ASSERT_TRUE(d1.should_apply);

  // Next window: ratio 9000 (within ROLLBACK_THRESHOLD of 10000) at 1.5x
  // baseline volume -> goodput 13500.
  AdaptiveController::Decision d2 = tc.tick(f.ring, f.ms(1020), current, measured(9000, 1500), mask);
  EXPECT_NE(TUNING_TX_DELAY_FACTOR, d2.param_id);  // moved on, no rollback
  EXPECT_EQ(1u, tc.stepPulls(1, 0));              // the -step step that was applied
  EXPECT_FLOAT_EQ(13500.0f, tc.stepRewardSum(1, 0));
}

TEST(AdaptiveController, GoodputStepRewardIsCappedAtTwentyThousand) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  tc.tick(f.ring, f.ms(1000), current, measured(10000));           // param 0 proposed (observe)
  tc.tick(f.ring, f.ms(1001), current, measured(10000, 60000));    // evaluates it
  EXPECT_FLOAT_EQ(20000.0f, tc.stepRewardSum(0, 0));
}

TEST(AdaptiveController, UnmeasuredWindowWhileLiveChangePendingKeepsItPending) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  int16_t current[AdaptiveController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  liveChangeOnParam1(f, tc, current, mask, 10000);
  // an insufficient window neither confirms nor rolls back...
  AdaptiveController::Decision hold = tc.tick(f.ring, f.ms(1020), current,
                                          unmeasured(EVAL_INSUFFICIENT_DATA), mask);
  EXPECT_TRUE(hold.hold);
  EXPECT_FALSE(hold.should_apply);
  // ...and the next measured window still evaluates that change.
  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1030), current, measured(1428), mask);
  EXPECT_EQ(TUNING_TX_DELAY_FACTOR, d.param_id);
  EXPECT_EQ(50, d.value);
  EXPECT_TRUE(d.should_apply);
}

TEST(AdaptiveController, RollbackRevertsToMostRecentGoodValueNotTheOriginal) {
  // last_good_value must track the value from just before the MOST RECENT
  // live change, not the first one ever made.
  RingFixture<512> f;
  AdaptiveController tc;
  tc.begin();

  int16_t current[AdaptiveController::NUM_PARAMS] = {1000, 0, 0, 0, 0, 0};
  uint8_t mask = 1 << 0;  // TUNING_RX_DELAY_BASE only
  const EvalWindow::Result healthy = measured(10000);
  int t = 2000;

  // Visit 1: first-ever visit picks step 0 (-step) -> 900, a real change.
  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  ASSERT_EQ(900, d.value);
  ASSERT_TRUE(d.should_apply);
  current[0] = d.value;   // firmware re-reads the live value every tick

  // Evaluating window (accepted) also proposes param 1; then 4 more to
  // come back around to param 0.
  for (int i = 0; i < 6; i++) d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  // Visit 2 of param 0 happened on the 6th call above: step 1 (stay) -> 900.
  ASSERT_EQ(TUNING_RX_DELAY_BASE, d.param_id);
  ASSERT_EQ(900, d.value);
  ASSERT_FALSE(d.should_apply);
  current[0] = d.value;

  for (int i = 0; i < 6; i++) d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  // Visit 3: step 2 (+step) -> 1000, a second real change; last_good must be 900.
  ASSERT_EQ(TUNING_RX_DELAY_BASE, d.param_id);
  ASSERT_EQ(1000, d.value);
  ASSERT_TRUE(d.should_apply);
  current[0] = d.value;

  // The very next window collapses.
  d = tc.tick(f.ring, f.ms(t++), current, measured(1000), mask);
  EXPECT_EQ(TUNING_RX_DELAY_BASE, d.param_id);
  EXPECT_EQ(900, d.value);
  EXPECT_TRUE(d.should_apply);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(AdaptiveController, ProposeFalseJudgesPendingButLeavesControllerIdle) {
  RingFixture<128> f;
  AdaptiveController tc;
  tc.begin();
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));
  EXPECT_FALSE(tc.idle());   // a proposal is awaiting its window's verdict
  AdaptiveController::Decision d = tc.tick(f.ring, f.ms(1300), kZeros, measured(9000), 0, false);
  EXPECT_TRUE(tc.idle());
  EXPECT_TRUE(d.hold);
  EXPECT_FALSE(d.should_apply);
  // and the normal path resumes afterwards
  tc.tick(f.ring, f.ms(1600), kZeros, measured(9000));
  EXPECT_FALSE(tc.idle());
}

TEST(AdaptiveController, AttachedObjectiveScoresStepsInsteadOfGoodput) {
  RingFixture<256> f;
  AdaptiveController tc;
  tc.begin();
  Objective o;
  o.weights[Objective::ROUTED] = 0;   // ignore volume: reward is the ratio alone
  tc.setObjective(&o);
  // two identical-ratio windows with very different volume must reward the same
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000, 500));
  tc.tick(f.ring, f.ms(1010), kZeros, measured(9000, 2000));
  EXPECT_NEAR(9000.0f, tc.stepRewardSum(TUNING_RX_DELAY_BASE, 0), 150.0f);
}
