#include <gtest/gtest.h>
#include <helpers/TrialPlanner.h>

namespace {

constexpr uint32_t SLOT_MS = ExposureTracker::SLOT_S * 1000u;

// Feed `n` slots of traffic: per slot `e` exposure, `c` confirmed, `r` rx.
void feed(ExposureTracker &t, uint32_t &now, uint32_t &E, uint32_t &C, uint32_t &R,
          int n, uint32_t e, uint32_t c, uint32_t r) {
  for (int i = 0; i < n; i++) {
    now += SLOT_MS;
    E += e; C += c; R += r;
    t.poll(now, E, C, R);
  }
}

TEST(ExposureTracker, FirstPollOnlyBasesTheCounters) {
  ExposureTracker t;
  t.poll(1000, 500, 200, 900);
  EXPECT_EQ(0, t.filled());
  EXPECT_FALSE(t.ready());
}

TEST(ExposureTracker, ClosesASlotEveryThirtyMinutes) {
  ExposureTracker t;
  uint32_t now = 1000, E = 500, C = 200, R = 900;
  t.poll(now, E, C, R);
  now += SLOT_MS - 1;
  t.poll(now, E + 10, C + 5, R + 20);
  EXPECT_EQ(0, t.filled());
  now += 1;
  t.poll(now, E + 10, C + 5, R + 20);
  EXPECT_EQ(1, t.filled());
}

TEST(ExposureTracker, NotReadyUntilMinSlots) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  feed(t, now, E, C, R, ExposureTracker::MIN_SLOTS - 1, 100, 40, 300);
  EXPECT_FALSE(t.ready());
  EXPECT_EQ(0.0, t.rates().confirmed);
  feed(t, now, E, C, R, 1, 100, 40, 300);
  EXPECT_TRUE(t.ready());
}

TEST(ExposureTracker, RatesAreThePerSecondCountsOfTheLowQuantileSlot) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  // 8 slots, confirmed 10, 20, ..., 80; the 25th percentile rank is (8-1)*25/100 = 1: the 20 slot
  for (int i = 1; i <= 8; i++) feed(t, now, E, C, R, 1, 100 * i, 10 * i, 1000 + i);
  TrialRates r = t.rates();
  EXPECT_DOUBLE_EQ(20.0 / 1800, r.confirmed);
  EXPECT_DOUBLE_EQ(200.0 / 1800, r.exposure);   // the same slot's exposure
  EXPECT_DOUBLE_EQ(1002.0 / 1800, r.rx);
}

TEST(ExposureTracker, RingKeepsTheLastDay) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  feed(t, now, E, C, R, ExposureTracker::SLOTS, 10, 1, 10);   // old, quiet
  feed(t, now, E, C, R, ExposureTracker::SLOTS, 100, 50, 100);   // a day of busier slots replaces them
  EXPECT_EQ(ExposureTracker::SLOTS, t.filled());
  EXPECT_DOUBLE_EQ(50.0 / 1800, t.rates().confirmed);
}

TEST(ExposureTracker, CounterGoingBackwardsClears) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  feed(t, now, E, C, R, 5, 100, 40, 300);
  ASSERT_TRUE(t.ready());
  now += SLOT_MS;
  t.poll(now, 3, 1, 2);   // device-side reset
  EXPECT_EQ(0, t.filled());
  EXPECT_FALSE(t.ready());
  E = 3; C = 1; R = 2;
  feed(t, now, E, C, R, 2, 100, 40, 300);   // slots count again from the new base
  EXPECT_TRUE(t.ready());
}

TEST(ExposureTracker, SlotCountsSaturate) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  feed(t, now, E, C, R, 2, 200000, 100000, 300000);
  EXPECT_DOUBLE_EQ(65535.0 / 1800, t.rates().confirmed);
}

TEST(ExposureTracker, ResetForgetsEverything) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  feed(t, now, E, C, R, 4, 100, 40, 300);
  t.reset();
  EXPECT_EQ(0, t.filled());
}

// A node with a given confirmed rate (per second) and a confirm ratio p, rx = 1.5 x exposure.
TrialPlanner::Input input(double conf_rate, double p = 0.5) {
  TrialPlanner::Input in;
  in.rates.confirmed = conf_rate;
  in.rates.exposure = conf_rate / p;
  in.rates.rx = 1.5 * conf_rate / p;
  return in;
}

TEST(TrialPlanner, BothFixedIsToday) {
  TrialPlanner::Input in;   // no rates at all
  in.block_s = 900;
  in.blocks = 16;
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_EQ(TrialPlanner::OK, p.status);
  EXPECT_EQ(900, p.block_s);
  EXPECT_EQ(16, p.blocks);
  EXPECT_EQ(0, p.settle_s);
  EXPECT_FALSE(p.futility);
}

TEST(TrialPlanner, NoRateWhenNothingWasConfirmed) {
  TrialPlanner::Input in = input(0.0);
  EXPECT_EQ(TrialPlanner::NO_RATE, TrialPlanner::plan(in).status);
  in = input(0.01);
  in.rates.exposure = 0;
  EXPECT_EQ(TrialPlanner::NO_RATE, TrialPlanner::plan(in).status);
}

TEST(TrialPlanner, BlockHoldsMinBlockNConfirmedDeliveries) {
  // default weights: v = 1 / C, so a block of 50 confirmed deliveries at 0.1 per second is 500 s
  TrialPlanner::Plan p = TrialPlanner::plan(input(0.1));
  EXPECT_EQ(TrialPlanner::OK, p.status);
  EXPECT_EQ(500, p.block_s);
  EXPECT_NEAR(1.0 / 50, p.v, 1e-9);
  EXPECT_EQ(TrialPlanner::SETTLE_S, p.settle_s);
  EXPECT_TRUE(p.futility);
}

TEST(TrialPlanner, BlockLengthIsClamped) {
  EXPECT_EQ(TrialPlanner::MIN_BLOCK_S, TrialPlanner::plan(input(10.0)).block_s);
  TrialPlanner::Plan slow = TrialPlanner::plan(input(0.001));   // 50000 s wanted
  EXPECT_EQ(TrialPlanner::MAX_BLOCK_S, slow.block_s);
}

TEST(TrialPlanner, PairsFollowTheNoiseModel) {
  // Oracle: P = 2 t^2 (v + c) / g^2 with the t of that many pairs, rounded up to a whole cycle.
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  double g = log(1.05), c = 0.17 * 0.17;
  double v = 1.0 / 50;
  uint32_t P = p.pairs;
  double t = TrialFSM::tCrit(P - 1, 5);
  double need = 2.0 * t * t * (v + c) / (g * g);
  EXPECT_GE(P, need - 1e-9);
  EXPECT_LT(P - 2, need);   // not more than one cycle above
  EXPECT_EQ(0u, P % 2);
  EXPECT_EQ(2 * P, p.blocks);
}

TEST(TrialPlanner, LargerGainOrLowerConfidenceNeedsFewerPairs) {
  TrialPlanner::Input in = input(0.1);
  uint32_t base = TrialPlanner::plan(in).pairs;
  in.min_gain_pct = 10;
  uint32_t gain10 = TrialPlanner::plan(in).pairs;
  EXPECT_LT(gain10, base);
  EXPECT_NEAR((double)base / gain10, 4.0, 1.0);   // pairs scale with 1 / g^2 (about)
  in = input(0.1);
  in.alpha_pct = 10;
  EXPECT_LT(TrialPlanner::plan(in).pairs, base);
}

TEST(TrialPlanner, BlockSpreadSetsThePairs) {
  TrialPlanner::Input in = input(0.1);
  in.spread_pct = 0;
  uint32_t none = TrialPlanner::plan(in).pairs;
  in.spread_pct = 10;
  uint32_t ten = TrialPlanner::plan(in).pairs;
  in.spread_pct = 17;
  uint32_t seventeen = TrialPlanner::plan(in).pairs;
  EXPECT_LT(none, ten);
  EXPECT_LT(ten, seventeen);
}

TEST(TrialPlanner, AtLeastOneCycleOfPairsAboveTheLookMinimum) {
  TrialPlanner::Input in = input(0.1);
  in.min_gain_pct = 50;
  in.spread_pct = 0;
  in.alpha_pct = 10;
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_GE(p.pairs, TrialPlanner::MIN_PAIRS);
  EXPECT_GT(p.pairs, TrialFSM::MIN_LOOK_PAIRS);
  EXPECT_EQ(0u, p.blocks % 4);
}

TEST(TrialPlanner, SlowNodeIsUnderpowered) {
  TrialPlanner::Input in = input(0.002);   // 7 confirmed an hour
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_EQ(TrialPlanner::UNDERPOWERED, p.status);
  EXPECT_GT(p.needed_confirmed_h, 7u);
  // the rate it names would fit: re-plan at that rate
  TrialPlanner::Input need = input(p.needed_confirmed_h / 3600.0);
  EXPECT_EQ(TrialPlanner::OK, TrialPlanner::plan(need).status);
}

TEST(TrialPlanner, BlockSpreadAloneCanMakeAnyRateUnderpowered) {
  TrialPlanner::Input in = input(0.1);
  in.spread_pct = 70;
  in.min_gain_pct = 1;
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_EQ(TrialPlanner::UNDERPOWERED, p.status);
  EXPECT_EQ(UINT32_MAX, p.needed_confirmed_h);
}

TEST(TrialPlanner, ScheduleStaysInsideTheTrialCap) {
  for (double rate : {0.005, 0.01, 0.05, 0.2, 1.0}) {
    TrialPlanner::Plan p = TrialPlanner::plan(input(rate));
    if (p.status != TrialPlanner::OK) continue;
    EXPECT_LE((uint64_t)p.blocks * (p.block_s + p.settle_s), TrialPlanner::MAX_TRIAL_S);
    EXPECT_LE(p.blocks, TrialPlanner::MAX_BLOCKS);
  }
}

TEST(TrialPlanner, FixedBlockPlansTheBlocksFromTheRate) {
  TrialPlanner::Input in = input(0.1);
  in.block_s = 300;   // 30 confirmed per block
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_EQ(300, p.block_s);
  EXPECT_NEAR(1.0 / 30, p.v, 1e-9);
  EXPECT_EQ(TrialPlanner::OK, p.status);
  EXPECT_GT(p.blocks, 0);
  EXPECT_TRUE(p.futility);
}

TEST(TrialPlanner, FixedBlocksTakeTheBlockFromTheRateAndSkipFeasibility) {
  TrialPlanner::Input in = input(0.002);   // would be underpowered in auto
  in.blocks = 16;
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  EXPECT_EQ(TrialPlanner::OK, p.status);
  EXPECT_EQ(16, p.blocks);
  EXPECT_EQ(TrialPlanner::MAX_BLOCK_S, p.block_s);
  EXPECT_FALSE(p.futility);   // the user's schedule, not the planner's
}

TEST(TrialPlanner, ShortUserBlocksAreNotSettleGuarded) {
  TrialPlanner::Input in = input(1.0);
  in.block_s = 3;
  EXPECT_EQ(0, TrialPlanner::plan(in).settle_s);
}

TEST(TrialPlanner, WeightsEnterThroughTheCountingNoise) {
  Objective o;
  o.weights[Objective::ROUTED] = 0;   // delivered ratio only: v = (1 - p) / C
  o.weights[Objective::DELIVERED] = 10;
  TrialPlanner::Input in = input(0.1, 0.5);
  in.objective = &o;
  TrialPlanner::Plan p = TrialPlanner::plan(in);
  // a (seconds) = (1 - 0.5) / 0.1 = 5, block = 50 * 5 = 250 s
  EXPECT_EQ(250, p.block_s);
  EXPECT_NEAR(5.0 / 250, p.v, 1e-9);
  // a RECEIVED weight adds its Poisson term on rx
  o.weights[Objective::RECEIVED] = 10;
  in.objective = &o;
  TrialPlanner::Plan q = TrialPlanner::plan(in);
  double rx = 1.5 * 0.1 / 0.5;
  EXPECT_NEAR(5.0 + 1.0 / rx, q.v * q.block_s, 1e-6);
}

TEST(Objective, CountingNoiseOfTheDefaultWeightsIsOneOverConfirmed) {
  Objective o;
  EXPECT_NEAR(1.0 / 40, o.countingNoise(100, 40, 250), 1e-12);
  EXPECT_NEAR(1.0 / 40, o.countingNoise(40, 40, 0), 1e-12);   // p = 1
  EXPECT_TRUE(std::isinf(o.countingNoise(100, 0, 10)));
  EXPECT_TRUE(std::isinf(o.countingNoise(0, 0, 10)));
  o.weights[Objective::RECEIVED] = 10;
  EXPECT_TRUE(std::isinf(o.countingNoise(100, 40, 0)));
  EXPECT_NEAR(1.0 / 40 + 1.0 / 250, o.countingNoise(100, 40, 250), 1e-12);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
