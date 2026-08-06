#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <helpers/TuneController.h>

namespace {

template <uint32_t N>
struct RingFixture {
  MonRecord buf[N];
  MonRing ring;
  RingFixture(uint32_t now = 1000) {
    ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), now, RadioRecord{}, EnvRecord{});
    ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
  }
};

// Lifetime TxConfirmStats snapshot -- ack_success/ack_timeout/echo_attempt/
// echo_success, same cumulative-since-boot shape the real counters have
// (BaseChatMesh::getAckSuccessCount()/getAckTimeoutCount(), SimpleMeshTables::
// getEchoAttemptCount()/getEchoSuccessCount()). rx_drop_count defaults to 0
// (no rx-side degradation) -- zero-initialize the whole struct first so
// adding a field to TxConfirmStats can never again leave a member here as
// uninitialized stack garbage.
TuneController::TxConfirmStats stats(uint32_t ack_success, uint32_t ack_timeout,
                                     uint32_t echo_attempt, uint32_t echo_success,
                                     uint32_t rx_drop = 0) {
  TuneController::TxConfirmStats s = {};
  s.ack_success_count = ack_success;
  s.ack_timeout_count = ack_timeout;
  s.echo_attempt_count = echo_attempt;
  s.echo_success_count = echo_success;
  s.rx_drop_count = rx_drop;
  return s;
}

const TuneController::TxConfirmStats kNoExposure = stats(0, 0, 0, 0);

}  // namespace

TEST(TuneController, TicksThroughAllParamsRoundRobinOneTuneRecordEach) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  for (int i = 0; i < TuneController::NUM_PARAMS; i++) {
    int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};
    tc.tick(f.ring, 1000 + i, current, kNoExposure);
  }

  EXPECT_EQ((uint32_t)TuneController::NUM_PARAMS, f.ring.tuneCount());

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ((uint32_t)TuneController::NUM_PARAMS, returned);
  // One record per param_id, in round-robin (TUNE_*) order.
  for (int i = 0; i < TuneController::NUM_PARAMS; i++) {
    EXPECT_EQ(MON_TUNE, out[i].kind);
    EXPECT_EQ((uint8_t)i, out[i].tune.param_id);
    EXPECT_EQ(0, out[i].tune.applied);  // observe-only -- never set
  }
}

TEST(TuneController, ProposedValueStaysWithinSpecRangeAtLowerBound) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  // rx_delay_base (param 0) at its floor (0): every arm (-step/stay/+step)
  // must clamp to [0, 2000], never go negative.
  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 3; i++) {   // enough ticks to visit param 0 at least once per arm across restarts
    tc.tick(f.ring, 1000 + i, current, kNoExposure);
  }

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_GE(returned, 1u);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, out[0].tune.param_id);
  EXPECT_GE(out[0].tune.proposed_value, 0);
  EXPECT_LE(out[0].tune.proposed_value, 2000);
}

TEST(TuneController, NeverSetsAppliedRegardlessOfRewardHistory) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  // Feed in a long, clearly-healthy TX-confirmation history, then tick many
  // times -- observe-only must hold no matter how confident the bandit gets.
  TuneController::TxConfirmStats healthy = stats(/*ack_success=*/20, /*ack_timeout=*/0,
                                                 /*echo_attempt=*/0, /*echo_success=*/0);
  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 20, 200};
  for (int i = 0; i < 30; i++) {
    tc.tick(f.ring, 2000 + i, current, healthy);
  }

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  for (uint32_t i = 0; i < returned; i++) {
    if (out[i].kind == MON_TUNE) {
      EXPECT_EQ(0, out[i].tune.applied);
    }
  }
}

TEST(TuneController, RewardBeforeReflectsTxConfirmRate) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  // 3 ack successes, 1 ack timeout, no flood exposure => reward = 3/4 = 7500
  // (of 10000).
  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};
  TuneController::Decision d = tc.tick(f.ring, 1010, current, stats(3, 1, 0, 0));
  (void)d;

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_GE(returned, 1u);
  EXPECT_EQ(MON_TUNE, out[returned - 1].kind);
  EXPECT_EQ(7500, out[returned - 1].tune.reward_before);
}

TEST(TuneController, RewardBlendsFloodEchoAndAckIntoOneWeightedRate) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  // 2 ack successes + 3 rpt = 5 confirmed; denominator = ack_success(2)
  // + ack_timeout(1) + echo_attempt(7) = 10 -> reward = 5000.
  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};
  tc.tick(f.ring, 1000, current, stats(/*ack_success=*/2, /*ack_timeout=*/1,
                                       /*echo_attempt=*/7, /*echo_success=*/3));

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_GE(returned, 1u);
  EXPECT_EQ(5000, out[returned - 1].tune.reward_before);
}

TEST(TuneController, RewardIsZeroWithNoExposureYet) {
  RingFixture<64> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 0, 0};
  tc.tick(f.ring, 1000, current, kNoExposure);

  MonRecord out[64];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(0, out[0].tune.reward_before);
}

TEST(TuneController, AppliedMaskBitEnablesLiveActuationDecision) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 0;  // TUNE_RX_DELAY_BASE only

  // First-ever visit to param 0: chooseArm() picks the first never-pulled
  // arm (index 0 = -step), so the proposal is deterministic.
  TuneController::Decision d = tc.tick(f.ring, 1000, current, kNoExposure, mask);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, d.param_id);
  EXPECT_EQ(900, d.value);          // 1000 - step(100)
  EXPECT_TRUE(d.should_apply);

  MonRecord out[128];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(1, out[0].tune.applied);
}

TEST(TuneController, AppliedMaskBitClearLeavesDecisionObserveOnly) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 50, 50, 10, 5, 200};
  // mask = 0 (default): identical proposal, but never applied.
  TuneController::Decision d = tc.tick(f.ring, 1000, current, kNoExposure);
  EXPECT_EQ(900, d.value);
  EXPECT_FALSE(d.should_apply);
}

TEST(TuneController, InterferenceThresholdNeverAppliesEvenWhenMasked) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {0, 0, 0, 0, 5, 0};
  uint8_t mask = 0xFF;  // every bit set, including TUNE_INTERFERENCE_THRESHOLD's

  TuneController::Decision d;
  for (int i = 0; i < 5; i++) {   // params 0..4 visited in order; index 4 = interference_threshold
    d = tc.tick(f.ring, 1000 + i, current, kNoExposure, mask);
  }
  EXPECT_EQ(TUNE_INTERFERENCE_THRESHOLD, d.param_id);
  EXPECT_FALSE(d.should_apply);  // isAppliable() excludes it regardless of the mask
}

TEST(TuneController, RegressionAfterLiveChangeTriggersRollbackToLastGood) {
  RingFixture<128> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {500, 50, 50, 10, 5, 200};
  uint8_t mask = 1 << 1;  // TUNE_TX_DELAY_FACTOR only

  // Healthy TX-confirmation rate (10 ack successes, 0 timeouts -> 10000)
  // throughout the two calls that establish state.
  TuneController::TxConfirmStats healthy = stats(10, 0, 0, 0);

  // Call 1: visits param 0 (not masked) -- establishes no relevant state.
  tc.tick(f.ring, 1010, current, healthy, mask);
  // Call 2: first visit to param 1 (masked) -- reward is still 10000 here.
  // First-ever visit picks arm 0 (-step), proposing 50-20=30 != 50, so this
  // live-applies and records last_good_value=50, reward_at_last_change=10000.
  TuneController::Decision d1 = tc.tick(f.ring, 1011, current, healthy, mask);
  EXPECT_EQ(TUNE_TX_DELAY_FACTOR, d1.param_id);
  EXPECT_EQ(30, d1.value);
  EXPECT_TRUE(d1.should_apply);

  // Push the TX-confirmation rate down hard: 10 successes vs 60 timeouts ->
  // 10/70 * 10000 = 1428, a drop of 8572 (well past ROLLBACK_THRESHOLD=1500).
  TuneController::TxConfirmStats degraded = stats(10, 60, 0, 0);

  // Calls 3-7: visit params 2, 3, 4, 5, 0 (not masked) to complete the
  // round-robin back around to param 1.
  for (int i = 0; i < 5; i++) {
    tc.tick(f.ring, 1050 + i, current, degraded, mask);
  }
  // Call 8: second visit to param 1 -- must roll back instead of trying the
  // bandit's next arm.
  TuneController::Decision d2 = tc.tick(f.ring, 1060, current, degraded, mask);
  EXPECT_EQ(TUNE_TX_DELAY_FACTOR, d2.param_id);
  EXPECT_EQ(50, d2.value);   // reverted to the pre-change value
  EXPECT_TRUE(d2.should_apply);
}

TEST(TuneController, RollbackRevertsToMostRecentGoodValueNotTheOriginal) {
  // Regression coverage: last_good_value must track the value from just
  // before the MOST RECENT live change, not the very first one ever made --
  // otherwise a rollback after a second successful change would overshoot
  // all the way back past a perfectly good intermediate value.
  RingFixture<256> f;
  TuneController tc;
  tc.begin();

  int16_t current[TuneController::NUM_PARAMS] = {1000, 0, 0, 0, 0, 0};
  uint8_t mask = 1 << 0;  // TUNE_RX_DELAY_BASE only

  // Healthy TX-confirmation rate (10 ack successes, 0 timeouts -> 10000)
  // throughout the two successful changes below.
  TuneController::TxConfirmStats healthy = stats(10, 0, 0, 0);

  // Visit 1 (call 1): first-ever visit picks arm 0 (-step) -> 1000-100=900,
  // a real change. last_good_value becomes 1000 (pre-first-change). Update
  // current[0] to 900 afterward -- the real firmware re-reads the actual
  // (now-changed) prefs value on every subsequent tick, so the test must
  // mirror that instead of feeding tick() a stale snapshot.
  TuneController::Decision d1 = tc.tick(f.ring, 2000, current, healthy, mask);
  ASSERT_EQ(900, d1.value);
  ASSERT_TRUE(d1.should_apply);
  current[0] = d1.value;

  // 5 calls for params 1-5, back around to param 0.
  for (int i = 0; i < 5; i++) tc.tick(f.ring, 2010 + i, current, healthy, mask);

  // Visit 2 (call 7): arm 1 (stay) -> 900+0=900, no change (should_apply
  // false) -- doesn't touch last_good_value.
  TuneController::Decision d2 = tc.tick(f.ring, 2020, current, healthy, mask);
  ASSERT_EQ(900, d2.value);
  ASSERT_FALSE(d2.should_apply);
  current[0] = d2.value;

  for (int i = 0; i < 5; i++) tc.tick(f.ring, 2030 + i, current, healthy, mask);

  // Visit 3 (call 13): arm 2 (+step) -> 900+100=1000, a second real change.
  // last_good_value must now become 900 (the just-confirmed-stable value),
  // not stay at the original 1000.
  TuneController::Decision d3 = tc.tick(f.ring, 2040, current, healthy, mask);
  ASSERT_EQ(1000, d3.value);
  ASSERT_TRUE(d3.should_apply);
  current[0] = d3.value;

  // Now push the TX-confirmation rate down hard, well past ROLLBACK_THRESHOLD.
  TuneController::TxConfirmStats degraded = stats(10, 60, 0, 0);

  for (int i = 0; i < 5; i++) tc.tick(f.ring, 2200 + i, current, degraded, mask);

  // Visit 4 (call 19): must roll back to 900 (the second change's baseline),
  // not 1000 (the very first-ever value) -- that's the fix under test.
  TuneController::Decision d4 = tc.tick(f.ring, 2210, current, degraded, mask);
  EXPECT_EQ(TUNE_RX_DELAY_BASE, d4.param_id);
  EXPECT_EQ(900, d4.value);
  EXPECT_TRUE(d4.should_apply);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
