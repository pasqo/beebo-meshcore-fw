#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <vector>
#include <helpers/TuneController.h>

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

  // beebo: TuneController::tick()'s `now` (like every MonRing append*())
  // takes an already-resolved epoch-ms instant directly now, not a raw
  // millis() reading -- see test_monring.cpp's own RingFixture::ms() for
  // the full rationale; identical helper, duplicated here since this
  // suite has its own small fixture rather than sharing one.
  uint64_t ms(uint32_t millis_like) const {
    return (uint64_t)seed * 1000 + (uint64_t)(millis_like - seed);
  }
};

// A closed EvalWindow::Result as TuneController::tick() receives it. `ratio`
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

const int16_t kZeros[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};

}  // namespace

TEST(TuneController, TicksThroughAllParamsRoundRobinOneTuneRecordEach) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  for (int i = 0; i < TuneController::NUM_PARAMS; i++) {
    tc.tick(f.ring, f.ms(1000 + i), kZeros, measured(9000));
  }

  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_EQ((size_t)TuneController::NUM_PARAMS, tunes.size());
  for (int i = 0; i < TuneController::NUM_PARAMS; i++) {
    EXPECT_EQ((uint8_t)i, tunes[i].tune.param_id);
    EXPECT_EQ(0, tunes[i].tune.applied);  // observe-only -- never set
  }
}

TEST(TuneController, ProposedValueStaysWithinSpecRangeAtLowerBound) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  // rx_delay_base (param 0) at its floor: every arm must clamp to [0, 2000].
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));

  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_GE(tunes.size(), 1u);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, tunes[0].tune.param_id);
  EXPECT_GE(tunes[0].tune.proposed_value, 0);
  EXPECT_LE(tunes[0].tune.proposed_value, 2000);
}

TEST(TuneController, NeverSetsAppliedRegardlessOfRewardHistory) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 20, 200};
  for (int i = 0; i < 30; i++) {
    tc.tick(f.ring, f.ms(2000 + i), current, measured(10000));
  }
  for (const MonRecord &r : ofKind(f.ring, MON_TUNE)) EXPECT_EQ(0, r.tune.applied);
}

TEST(TuneController, TuneRecordRewardBeforeIsTheClosedWindowsConfirmRatio) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  tc.tick(f.ring, f.ms(1010), kZeros, measured(7500));

  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(7500, tunes[0].tune.reward_before);
}

TEST(TuneController, EmitsEvalRunThenTuneRecordSharingWindowIds) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();
  uint16_t closing = tc.windowId();

  TuneController::Decision d = tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));

  auto all = readAll(f.ring);
  ASSERT_EQ(3u, all.size());
  EXPECT_EQ(MON_EVAL | RLOG_CONT_BIT, all[0].kind);
  EXPECT_EQ(MON_EVAL, all[1].kind);
  EXPECT_EQ(MON_TUNE, all[2].kind);
  EXPECT_EQ(closing, all[0].eval_a.window_id);
  EXPECT_EQ(EVAL_ACCEPTED, all[0].eval_a.outcome);
  EXPECT_EQ(9000, all[0].eval_a.confirm_ratio);
  // the new decision is evaluated by the NEXT window
  EXPECT_EQ((uint16_t)(closing + 1), d.window_id);
  EXPECT_EQ(d.window_id, all[2].tune.iteration);
  EXPECT_EQ(d.window_id, tc.windowId());
}

TEST(TuneController, UnmeasuredWindowEmitsEvalOnlyAndHolds) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();
  uint16_t closing = tc.windowId();

  TuneController::Decision d = tc.tick(f.ring, f.ms(1000), kZeros,
                                       unmeasured(EVAL_INSUFFICIENT_DATA), 0xFF);

  EXPECT_TRUE(d.hold);
  EXPECT_FALSE(d.should_apply);
  EXPECT_EQ((uint16_t)(closing + 1), d.window_id);
  auto all = readAll(f.ring);
  ASSERT_EQ(2u, all.size());  // MON_EVAL run only, no MON_TUNE
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, all[0].eval_a.outcome);
  EXPECT_EQ(0u, ofKind(f.ring, MON_TUNE).size());
}

TEST(TuneController, UnmeasuredWindowDoesNotAdvanceRoundRobinOrArmStats) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();
  tc.tick(f.ring, f.ms(1000), kZeros, unmeasured(EVAL_INSUFFICIENT_DATA));
  tc.tick(f.ring, f.ms(1001), kZeros, unmeasured(EVAL_INVALIDATED, EVALF_REBOOT));

  tc.tick(f.ring, f.ms(1002), kZeros, measured(9000));
  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(TUNE_RX_DELAY_BASE, tunes[0].tune.param_id);  // still the first param
  for (int a = 0; a < TuneController::NUM_ARMS; a++) {
    EXPECT_EQ(0u, tc.armPulls(0, a));
  }
}

TEST(TuneController, AppliedMaskBitEnablesLiveActuationDecision) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 0;  // TUNE_RX_DELAY_BASE only

  // First-ever visit to param 0: chooseArm() picks the first never-pulled
  // arm (index 0 = -step), so the proposal is deterministic.
  TuneController::Decision d = tc.tick(f.ring, f.ms(1000), current, measured(9000), mask);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, d.param_id);
  EXPECT_EQ(900, d.value);          // 1000 - step(100)
  EXPECT_TRUE(d.should_apply);
  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_EQ(1u, tunes.size());
  EXPECT_EQ(1, tunes[0].tune.applied);
}

TEST(TuneController, AppliedMaskBitClearLeavesDecisionObserveOnly) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  TuneController::Decision d = tc.tick(f.ring, f.ms(1000), current, measured(9000));
  EXPECT_EQ(900, d.value);
  EXPECT_FALSE(d.should_apply);
}

TEST(TuneController, InterferenceThresholdNeverAppliesEvenWhenMasked) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 5, 0};
  uint8_t mask = 0xFF;  // every bit set, including TUNE_INTERFERENCE_THRESHOLD's

  TuneController::Decision d;
  for (int i = 0; i < 5; i++) {   // params 0..4 visited in order; index 4 = interference_threshold
    d = tc.tick(f.ring, f.ms(1000 + i), current, measured(9000), mask);
  }
  EXPECT_EQ(TUNE_INTERFERENCE_THRESHOLD, d.param_id);
  EXPECT_FALSE(d.should_apply);  // isApplicable() excludes it regardless of the mask
}

// Params are visited in order, so the tx_delay_factor (param 1) proposal is
// the SECOND tick's decision. Helper: apply param 0 unmasked (no live change),
// then the masked live change on param 1; returns that decision.
static TuneController::Decision liveChangeOnParam1(RingFixture<256> &f, TuneController &tc,
                                                   int16_t current[TuneController::NUM_PARAMS],
                                                   uint8_t mask, uint16_t ratio_before) {
  tc.tick(f.ring, f.ms(1010), current, measured(ratio_before), mask);          // param 0
  return tc.tick(f.ring, f.ms(1011), current, measured(ratio_before), mask);   // param 1
}

TEST(TuneController, RegressionInTheNextWindowRollsBackImmediately) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;  // TUNE_TX_DELAY_FACTOR only

  TuneController::Decision d1 = liveChangeOnParam1(f, tc, current, mask, 10000);
  ASSERT_EQ(TUNE_TX_DELAY_FACTOR, d1.param_id);
  ASSERT_EQ(30, d1.value);
  ASSERT_TRUE(d1.should_apply);

  // The very next window (not six ticks later) shows a collapse: 10000 -> 1428.
  TuneController::Decision d2 = tc.tick(f.ring, f.ms(1020), current, measured(1428), mask);
  EXPECT_EQ(TUNE_TX_DELAY_FACTOR, d2.param_id);
  EXPECT_EQ(50, d2.value);   // reverted to the pre-change value
  EXPECT_TRUE(d2.should_apply);

  auto evals = ofKind(f.ring, MON_EVAL);
  ASSERT_GE(evals.size(), 2u);
  const MonRecord &last_a = evals[evals.size() - 2];
  EXPECT_EQ(EVAL_ROLLBACK, last_a.eval_a.outcome);
}

TEST(TuneController, GuardrailTripInTheNextWindowRollsBackEvenWithHealthyRatio) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  ASSERT_TRUE(liveChangeOnParam1(f, tc, current, mask, 10000).should_apply);
  TuneController::Decision d2 = tc.tick(f.ring, f.ms(1020), current,
                                        measured(10000, 1000, EVALF_GUARDRAIL), mask);
  EXPECT_EQ(50, d2.value);
  EXPECT_TRUE(d2.should_apply);
}

TEST(TuneController, RollbackTickProposesNothingNew) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  liveChangeOnParam1(f, tc, current, mask, 10000);
  size_t tunes_before = ofKind(f.ring, MON_TUNE).size();
  tc.tick(f.ring, f.ms(1020), current, measured(1428), mask);   // rollback
  auto tunes = ofKind(f.ring, MON_TUNE);
  ASSERT_EQ(tunes_before + 1, tunes.size());                    // only the revert record
  EXPECT_EQ(1, tunes.back().tune.applied);
  EXPECT_EQ(50, tunes.back().tune.proposed_value);

  // the next measured window resumes the round-robin at param 2
  TuneController::Decision d3 = tc.tick(f.ring, f.ms(1030), current, measured(9000), mask);
  EXPECT_EQ(TUNE_DIRECT_TX_DELAY_FACTOR, d3.param_id);
}

TEST(TuneController, IdleOnlyWhileNoDecisionAwaitsItsWindow) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  EXPECT_TRUE(tc.idle());
  tc.tick(f.ring, f.ms(1000), kZeros, measured(9000));   // proposes param 0
  EXPECT_FALSE(tc.idle());
  tc.tick(f.ring, f.ms(1001), kZeros, unmeasured(EVAL_INSUFFICIENT_DATA));   // still pending
  EXPECT_FALSE(tc.idle());
}

TEST(TuneController, StableWindowAfterLiveChangeIsAcceptedAndUpdatesArmWithGoodput) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  TuneController::Decision d1 = liveChangeOnParam1(f, tc, current, mask, 10000);
  ASSERT_TRUE(d1.should_apply);

  // Next window: ratio 9000 (within ROLLBACK_THRESHOLD of 10000) at 1.5x
  // baseline volume -> goodput 13500.
  TuneController::Decision d2 = tc.tick(f.ring, f.ms(1020), current, measured(9000, 1500), mask);
  EXPECT_NE(TUNE_TX_DELAY_FACTOR, d2.param_id);  // moved on, no rollback
  EXPECT_EQ(1u, tc.armPulls(1, 0));              // the -step arm that was applied
  EXPECT_FLOAT_EQ(13500.0f, tc.armRewardSum(1, 0));
}

TEST(TuneController, GoodputArmRewardIsCappedAtTwentyThousand) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  tc.tick(f.ring, f.ms(1000), current, measured(10000));           // param 0 proposed (observe)
  tc.tick(f.ring, f.ms(1001), current, measured(10000, 60000));    // evaluates it
  EXPECT_FLOAT_EQ(20000.0f, tc.armRewardSum(0, 0));
}

TEST(TuneController, UnmeasuredWindowWhileLiveChangePendingKeepsItPending) {
  RingFixture<256> f;
  TuneController tc;
  tc.begin();
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;

  liveChangeOnParam1(f, tc, current, mask, 10000);
  // an insufficient window neither confirms nor rolls back...
  TuneController::Decision hold = tc.tick(f.ring, f.ms(1020), current,
                                          unmeasured(EVAL_INSUFFICIENT_DATA), mask);
  EXPECT_TRUE(hold.hold);
  EXPECT_FALSE(hold.should_apply);
  // ...and the next measured window still evaluates that change.
  TuneController::Decision d = tc.tick(f.ring, f.ms(1030), current, measured(1428), mask);
  EXPECT_EQ(TUNE_TX_DELAY_FACTOR, d.param_id);
  EXPECT_EQ(50, d.value);
  EXPECT_TRUE(d.should_apply);
}

TEST(TuneController, RollbackRevertsToMostRecentGoodValueNotTheOriginal) {
  // last_good_value must track the value from just before the MOST RECENT
  // live change, not the first one ever made.
  RingFixture<512> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 0, 0, 0, 0, 0};
  uint8_t mask = 1 << 0;  // TUNE_RX_DELAY_BASE only
  const EvalWindow::Result healthy = measured(10000);
  int t = 2000;

  // Visit 1: first-ever visit picks arm 0 (-step) -> 900, a real change.
  TuneController::Decision d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  ASSERT_EQ(900, d.value);
  ASSERT_TRUE(d.should_apply);
  current[0] = d.value;   // firmware re-reads the live value every tick

  // Evaluating window (accepted) also proposes param 1; then 4 more to
  // come back around to param 0.
  for (int i = 0; i < 6; i++) d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  // Visit 2 of param 0 happened on the 6th call above: arm 1 (stay) -> 900.
  ASSERT_EQ(TUNE_RX_DELAY_BASE, d.param_id);
  ASSERT_EQ(900, d.value);
  ASSERT_FALSE(d.should_apply);
  current[0] = d.value;

  for (int i = 0; i < 6; i++) d = tc.tick(f.ring, f.ms(t++), current, healthy, mask);
  // Visit 3: arm 2 (+step) -> 1000, a second real change; last_good must be 900.
  ASSERT_EQ(TUNE_RX_DELAY_BASE, d.param_id);
  ASSERT_EQ(1000, d.value);
  ASSERT_TRUE(d.should_apply);
  current[0] = d.value;

  // The very next window collapses.
  d = tc.tick(f.ring, f.ms(t++), current, measured(1000), mask);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, d.param_id);
  EXPECT_EQ(900, d.value);
  EXPECT_TRUE(d.should_apply);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
