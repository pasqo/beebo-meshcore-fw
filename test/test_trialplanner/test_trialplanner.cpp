#include <gtest/gtest.h>
#include <helpers/TrialPlanner.h>

namespace {

// Steady traffic: `e`, `c`, `r` events per second for `secs` seconds, polled every second.
void run(ExposureTracker &t, uint32_t &now, uint32_t &E, uint32_t &C, uint32_t &R,
         int secs, double e, double c, double r) {
  for (int i = 0; i < secs; i++) {
    now += 1000;
    // whole events: carry the fraction in the counters by rounding the running total
    E = (uint32_t)((now / 1000.0) * e); C = (uint32_t)((now / 1000.0) * c); R = (uint32_t)((now / 1000.0) * r);
    t.poll(now, E, C, R);
  }
}

TEST(ExposureTracker, NotReadyAtFirst) {
  ExposureTracker t;
  t.poll(1000, 500, 200, 900);
  EXPECT_FALSE(t.ready());
  EXPECT_EQ(0.0, t.rates().confirmed);
}

TEST(ExposureTracker, NeedsTimeAndConfirmedDeliveries) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  run(t, now, E, C, R, 20, 2.0, 1.0, 3.0);   // plenty heard, too early
  EXPECT_FALSE(t.ready());
  ExposureTracker u;
  now = 0; E = C = R = 0;
  u.poll(now, 0, 0, 0);
  run(u, now, E, C, R, 120, 0.02, 0.01, 0.03);   // long enough, one confirmed delivery
  EXPECT_FALSE(u.ready());
  run(t, now, E, C, R, 40, 2.0, 1.0, 3.0);
  EXPECT_TRUE(t.ready());   // 60 s, 60 confirmed
}

TEST(ExposureTracker, RatesAreTheCountsPerSecond) {
  ExposureTracker t;
  uint32_t now = 5000, E = 1000, C = 400, R = 3000;   // counters do not start at zero
  t.poll(now, E, C, R);
  for (int i = 0; i < 600; i++) {
    now += 1000; E += 2; C += 1; R += 5;
    t.poll(now, E, C, R);
  }
  ASSERT_TRUE(t.ready());
  TrialRates r = t.rates();
  EXPECT_NEAR(2.0, r.exposure, 1e-6);
  EXPECT_NEAR(1.0, r.confirmed, 1e-6);
  EXPECT_NEAR(5.0, r.rx, 1e-6);
}

TEST(ExposureTracker, RatesFollowTheTrafficOverHours) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  for (int i = 0; i < 6 * 3600; i++) {   // six hours busy, then three quiet
    now += 1000; E += 2; C += 1; R += 3;
    t.poll(now, E, C, R);
  }
  EXPECT_NEAR(1.0, t.rates().confirmed, 0.01);
  for (int i = 0; i < 3 * 3600; i++) {
    now += 1000; E += 1; C += 0 + (i % 10 == 0); R += 1;
    t.poll(now, E, C, R);
  }
  double c = t.rates().confirmed;
  EXPECT_LT(c, 0.45);   // moved most of the way to the quiet 0.1/s
  EXPECT_GT(c, 0.1);    // but is not a snapshot
}

TEST(ExposureTracker, FoldsOncePerSecond) {
  ExposureTracker t;
  t.poll(0, 0, 0, 0);
  for (uint32_t ms = 10; ms < 60000; ms += 10) t.poll(ms, ms / 10, ms / 20, ms / 5);
  t.poll(60000, 6000, 3000, 12000);
  EXPECT_TRUE(t.ready());
  EXPECT_NEAR(50.0, t.rates().confirmed, 1.0);   // 3000 in 60 s
}

TEST(ExposureTracker, CounterGoingBackwardsStartsOver) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  run(t, now, E, C, R, 300, 2.0, 1.0, 3.0);
  ASSERT_TRUE(t.ready());
  now += 1000;
  t.poll(now, 3, 1, 2);   // device-side reset
  EXPECT_FALSE(t.ready());
  E = 3; C = 1; R = 2;
  for (int i = 0; i < 120; i++) { now += 1000; E += 2; C += 1; R += 3; t.poll(now, E, C, R); }
  EXPECT_TRUE(t.ready());
  EXPECT_NEAR(1.0, t.rates().confirmed, 1e-6);
}

TEST(ExposureTracker, ResetForgetsEverything) {
  ExposureTracker t;
  uint32_t now = 0, E = 0, C = 0, R = 0;
  t.poll(now, E, C, R);
  run(t, now, E, C, R, 300, 2.0, 1.0, 3.0);
  t.reset();
  EXPECT_FALSE(t.ready());
}

// A node with a given confirmed rate (per second) and a confirm ratio p, rx = 1.5 x exposure.
TrialPlanner::Input input(double conf_rate, double p = 0.5) {
  TrialPlanner::Input in;
  in.spread_pct = 17;   // the cases below were worked out at 17% (c = 0.0289)
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
  in.spread_c = 0.0;   // counting noise alone
  uint32_t none = TrialPlanner::plan(in).pairs;
  in.spread_c = -1;
  in.spread_pct = 10;
  uint32_t ten = TrialPlanner::plan(in).pairs;
  in.spread_pct = 17;
  uint32_t seventeen = TrialPlanner::plan(in).pairs;
  EXPECT_LT(none, ten);
  EXPECT_LT(ten, seventeen);
  in.spread_pct = 0;   // no override: the built-in start, c = 0.02 (14%)
  TrialPlanner::Plan built_in = TrialPlanner::plan(in);
  EXPECT_EQ(14, built_in.spread_pct);
  EXPECT_LT(ten, built_in.pairs);
  EXPECT_LT(built_in.pairs, seventeen);
}

TEST(TrialPlanner, AtLeastOneCycleOfPairsAboveTheLookMinimum) {
  TrialPlanner::Input in = input(0.1);
  in.min_gain_pct = 50;
  in.spread_c = 0.0;
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

TEST(TrialPlanner, ReplanKeepsTheBlockAndFollowsTheARates) {
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Plan first = TrialPlanner::plan(in);   // 500 s blocks
  ASSERT_EQ(500, first.block_s);
  TrialPlanner::Plan p;
  // traffic as planned: the same schedule
  ASSERT_TRUE(TrialPlanner::replan(in, in.rates, first.block_s, false, p));
  EXPECT_EQ(first.blocks, p.blocks);
  EXPECT_EQ(500, p.block_s);
  // half the confirmed rate: blocks of 500 s now hold 25 deliveries, v doubles, more pairs
  TrialPlanner::Input half = input(0.05);
  ASSERT_TRUE(TrialPlanner::replan(in, half.rates, first.block_s, false, p));
  EXPECT_GT(p.blocks, first.blocks);
  EXPECT_EQ(500, p.block_s);
  // busier: fewer
  TrialPlanner::Input busy = input(0.4);
  ASSERT_TRUE(TrialPlanner::replan(in, busy.rates, first.block_s, false, p));
  EXPECT_LT(p.blocks, first.blocks);
}

TEST(TrialPlanner, ReplanGrowsUpToTheCapThenIsUnderpowered) {
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Input slow = input(0.0005);
  TrialPlanner::Plan p;
  ASSERT_TRUE(TrialPlanner::replan(in, slow.rates, 500, false, p));
  EXPECT_EQ(TrialPlanner::UNDERPOWERED, p.status);
  uint32_t cap = TrialPlanner::MAX_TRIAL_S / (500 + TrialPlanner::SETTLE_S);
  EXPECT_EQ(cap & ~3u, p.blocks);   // the most the cap allows, whole cycles
  EXPECT_LE((uint64_t)p.blocks * (500 + TrialPlanner::SETTLE_S), TrialPlanner::MAX_TRIAL_S);
}

TEST(TrialPlanner, ReplanWithoutConfirmedDeliveriesSaysNothing) {
  TrialPlanner::Input in = input(0.1);
  TrialRates none;
  TrialPlanner::Plan p;
  EXPECT_FALSE(TrialPlanner::replan(in, none, 500, false, p));
}

TEST(TrialPlanner, ReplanIgnoresAFixedBlockCountOfTheInput) {
  TrialPlanner::Input in = input(0.1);
  in.blocks = 16;   // the caller does not replan a fixed count; replan itself always plans
  TrialPlanner::Plan p;
  ASSERT_TRUE(TrialPlanner::replan(in, in.rates, 500, false, p));
  EXPECT_NE(16, p.blocks);
}

TEST(TrialPlanner, AdaptiveReplanKeepsTheCountsPerBlockAndSoThePairs) {
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Plan first = TrialPlanner::plan(in);   // 500 s blocks, 50 counts
  TrialPlanner::Plan p;
  // four times the traffic: blocks a quarter as long, the same noise, the same pairs
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.4).rates, first.block_s, true, p));
  EXPECT_EQ(125, p.block_s);
  EXPECT_NEAR(first.v, p.v, 1e-9);
  EXPECT_EQ(first.blocks, p.blocks);
  // 70% of the rate: longer blocks, the same noise and pairs
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.07).rates, first.block_s, true, p));
  EXPECT_EQ(715, p.block_s);
  EXPECT_EQ(first.blocks, p.blocks);
  // so slow that the block hits its ceiling: the noise is higher and the cap binds
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.01).rates, first.block_s, true, p));
  EXPECT_EQ(TrialPlanner::MAX_BLOCK_S, p.block_s);
  EXPECT_EQ(TrialPlanner::UNDERPOWERED, p.status);
}

TEST(TrialPlanner, AdaptiveReplanMovesTheBlockOnlyByTheStep) {
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Plan p;
  // 10% off: inside the step, the length stays and so does the schedule
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.11).rates, 500, true, p));
  EXPECT_EQ(500, p.block_s);
  // 20% off: it moves
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.125).rates, 500, true, p));
  EXPECT_EQ(400, p.block_s);
  // not adaptive: never
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.4).rates, 500, false, p));
  EXPECT_EQ(500, p.block_s);
}

TEST(TrialPlanner, ReplanCapCountsTheTimeAlreadySpent) {
  TrialPlanner::Input in = input(0.1);
  in.elapsed_s = TrialPlanner::MAX_TRIAL_S - 5100;   // 5100 s left
  in.blocks_run = 40;
  TrialPlanner::Plan p;
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.0005).rates, 500, false, p));
  EXPECT_EQ(TrialPlanner::UNDERPOWERED, p.status);
  EXPECT_EQ((40u + 5100u / (500 + TrialPlanner::SETTLE_S)) & ~3u, p.blocks);
  in.elapsed_s = TrialPlanner::MAX_TRIAL_S + 10;   // out of time: the schedule ends where it is
  ASSERT_TRUE(TrialPlanner::replan(in, input(0.0005).rates, 500, false, p));
  EXPECT_EQ(40u, p.blocks);
}

TEST(TrialPlanner, BlendedSpreadStartsAtTheSettingAndMovesWithThePairs) {
  TrialFSM::Noise z = {};
  // too few pairs: the setting
  z.pairs = 3; z.blocks = 6; z.var_d = 1.0; z.mean_v = 0.02;
  EXPECT_NEAR(TrialPlanner::spreadC(17), TrialPlanner::blendSpread(17, z), 1e-12);
  // 3 pairs of weight for the prior: 3 pairs seeing c = 0.1 against 0.0289 is half way
  z.pairs = 3; z.blocks = 6;
  EXPECT_NEAR(TrialPlanner::spreadC(17), TrialPlanner::blendSpread(17, z), 1e-12);   // under 4 pairs: the prior
  z.pairs = 6; z.blocks = 12; z.var_d = 2 * (0.02 + 0.1); z.mean_v = 0.02;
  EXPECT_NEAR((3 * 0.0289 + 6 * 0.1) / 9, TrialPlanner::blendSpread(17, z), 1e-9);
  // 90 pairs: the data dominate
  z.pairs = 90; z.blocks = 180;
  EXPECT_NEAR((3 * 0.0289 + 90 * 0.1) / 93, TrialPlanner::blendSpread(17, z), 1e-9);
  // quieter than counting noise alone: c = 0, the blend moves down
  z.var_d = 2 * 0.01;
  EXPECT_NEAR(3 * 0.0289 / 93, TrialPlanner::blendSpread(17, z), 1e-9);
  // no override: the built-in start, c = 0.02
  EXPECT_NEAR(0.02, TrialPlanner::blendSpread(0, TrialFSM::Noise{}), 1e-12);
}

TEST(TrialPlanner, ReplanPlansWithTheBlendedSpread) {
  TrialPlanner::Input in = input(0.1);
  TrialPlanner::Plan base = TrialPlanner::plan(in);
  EXPECT_EQ(17, base.spread_pct);
  TrialPlanner::Plan p;
  in.spread_c = 0.09;   // 30%: noisier than the setting
  ASSERT_TRUE(TrialPlanner::replan(in, in.rates, 500, false, p));
  EXPECT_EQ(30, p.spread_pct);
  EXPECT_GT(p.blocks, base.blocks);
  in.spread_c = 0.0;   // counting noise alone: fewer
  ASSERT_TRUE(TrialPlanner::replan(in, in.rates, 500, false, p));
  EXPECT_EQ(0, p.spread_pct);
  EXPECT_LT(p.blocks, base.blocks);
  EXPECT_EQ(255, TrialPlanner::spreadPct(100.0));   // saturated
}

TEST(TrialPlanner, CountsPerMinuteAreTheScoresOwnEvents) {
  TrialPlanner::Input in = input(0.1);   // 0.1 confirmed per second, exposure 0.2, rx 0.3
  EXPECT_NEAR(6.0, TrialPlanner::countsPerMin(in.rates), 1e-9);   // the default score: confirmed
  Objective routed;
  routed.weights[Objective::DELIVERED] = 0;
  EXPECT_NEAR(12.0, TrialPlanner::countsPerMin(in.rates, &routed), 1e-9);
  Objective rx;
  rx.weights[Objective::ROUTED] = 0;
  rx.weights[Objective::DELIVERED] = 0;
  rx.weights[Objective::RECEIVED] = 10;
  EXPECT_NEAR(18.0, TrialPlanner::countsPerMin(in.rates, &rx), 1e-9);
  EXPECT_EQ(0.0, TrialPlanner::countsPerMin(TrialRates()));       // no rates
}

TEST(TrialPlanner, CountsPerMinuteDoNotMoveWithAUniformWeightScale) {
  TrialPlanner::Input in = input(0.1);
  Objective twice;   // every weight doubled: the score moves twice as much per change
  twice.weights[Objective::ROUTED] = 20;
  twice.weights[Objective::DELIVERED] = 20;
  EXPECT_NEAR(TrialPlanner::countsPerMin(in.rates), TrialPlanner::countsPerMin(in.rates, &twice), 1e-9);
  Objective half;
  half.weights[Objective::ROUTED] = 5;
  half.weights[Objective::DELIVERED] = 5;
  EXPECT_NEAR(6.0, TrialPlanner::countsPerMin(in.rates, &half), 1e-9);
  // a mix keeps its proportions: routed 10 with received 20, the largest weight squared is 4
  Objective mix;
  mix.weights[Objective::DELIVERED] = 0;
  mix.weights[Objective::RECEIVED] = 20;
  double a = 1.0 / 0.2 + 4.0 / 0.3;   // wr^2 / E + wx^2 / R, E = 0.2 and R = 0.3 a second
  EXPECT_NEAR(60.0 * 4.0 / a, TrialPlanner::countsPerMin(in.rates, &mix), 1e-9);
  EXPECT_EQ(1.0, Objective().maxWeightSq());
  Objective none;
  for (int i = 0; i < Objective::NUM_INDICATORS; i++) none.weights[i] = 0;
  EXPECT_EQ(0.0, TrialPlanner::countsPerMin(in.rates, &none));
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
