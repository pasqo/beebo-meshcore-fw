#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <helpers/TrialFSM.h>

namespace {

// A closed block as TrialFSM::onBlock() receives it. `ros_rate` is confirmed
// deliveries per hour, `ratio` the confirm ratio (0-10000); every measured block
// heard 10 packets unless a test says otherwise.
EvalWindow::Result blk(uint32_t ros_rate, uint16_t ratio = 9000, uint8_t flags = 0) {
  EvalWindow::Result r = {};
  r.measured = true;
  r.outcome = EVAL_ACCEPTED;
  r.ros_rate = ros_rate;
  r.confirm_ratio = ratio;
  r.flags = flags;
  r.exposure = 200;
  r.rx_valid = 10;
  return r;
}

// A block that heard `rx` packets: measured when rx > 0, else insufficient data
// (EvalWindow's rule for trial blocks).
EvalWindow::Result heard(uint32_t rx) {
  if (rx == 0) {
    EvalWindow::Result r = {};
    r.measured = false;
    r.outcome = EVAL_INSUFFICIENT_DATA;
    return r;
  }
  EvalWindow::Result r = blk(100);
  r.rx_valid = rx;
  return r;
}

// Feed blocks whose RX count depends on the arm, until the trial finishes or
// `blocks` have run.
TrialFSM::Step runHeard(TrialFSM &t, uint8_t original, int blocks, uint32_t a_rx, uint32_t b_rx) {
  TrialFSM::Step s = {};
  for (int i = 0; i < blocks && !s.finished; i++) {
    s = t.onBlock(heard(t.currentValue() == original ? a_rx : b_rx), 0, 0);
  }
  return s;
}

EvalWindow::Result unmeasured(uint8_t outcome, uint8_t flags = 0) {
  EvalWindow::Result r = {};
  r.measured = false;
  r.outcome = outcome;
  r.flags = flags;
  return r;
}

TrialFSM::Config cfg(uint16_t blocks) {
  TrialFSM::Config c;
  c.block_s = 1800;
  c.blocks = blocks;
  return c;
}

// Baseline A goodput varies a little so the paired differences have a spread.
uint32_t aRate(int pair) { return 100 + (pair % 3) * 10; }

// Feed up to `pairs` adjacent pairs; `b_of_a` maps A's ros_rate to B's.
// Returns the final Step -- stops as soon as the trial finishes (early stop
// or an abort mid-pair), since calling onBlock() again once DONE returns an
// empty, unfinished Step that would otherwise overwrite the real one. Each
// pair is (A block, B block) or (B block, A block) per ABBA, so the value is
// taken from the FSM's own value, not assumed.
TrialFSM::Step runPairs(TrialFSM &t, uint8_t original, int pairs,
                        uint32_t (*b_of_a)(uint32_t), uint16_t a_ratio = 9000,
                        uint16_t b_ratio = 9000) {
  TrialFSM::Step step = {};
  for (int p = 0; p < pairs; p++) {
    uint32_t a = aRate(p);
    for (int k = 0; k < 2; k++) {
      bool is_a = (t.currentValue() == original);
      step = t.onBlock(is_a ? blk(a, a_ratio) : blk(b_of_a(a), b_ratio), 0, 0);
      if (step.finished) return step;
    }
  }
  return step;
}

uint32_t clearlyBetter(uint32_t a) { return a * 3 / 2; }
uint32_t clearlyWorse(uint32_t a) { return a * 2 / 3; }
// same mean as A, but a different spread: no consistent gain
uint32_t noise(uint32_t a) { return 220 - a; }

}  // namespace

TEST(TrialFSM, StartEntersRunOnTheOriginalValue) {
  TrialFSM t;
  t.begin(cfg(96));
  EXPECT_EQ(TrialFSM::IDLE, t.state());
  ASSERT_TRUE(t.start(1));
  EXPECT_EQ(TrialFSM::RUN, t.state());
  EXPECT_EQ(1, t.currentValue());
  EXPECT_EQ(0, t.blockIndex());
}

TEST(TrialFSM, StartWhileRunningIsRefused) {
  TrialFSM t;
  t.begin(cfg(96));
  ASSERT_TRUE(t.start(0));
  EXPECT_FALSE(t.start(0));
}

TEST(TrialFSM, ScheduleIsAbbaCyclic) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  uint8_t expect[8] = {1, 0, 0, 1, 1, 0, 0, 1};
  for (int i = 0; i < 8; i++) {
    EXPECT_EQ(expect[i], t.currentValue()) << "block " << i;
    t.onBlock(blk(100 + (i * 37) % 29), 0, 0);   // spread, so no early decision
  }
}

TEST(TrialFSM, OriginalZeroInvertsTheSchedule) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  uint8_t expect[4] = {0, 1, 1, 0};
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(expect[i], t.currentValue());
    t.onBlock(blk(100), 0, 0);
  }
}

TEST(TrialFSM, AdoptsBAsSoonAsTheEvidenceSettles) {
  TrialFSM t;
  t.begin(cfg(48));   // 24 pairs available
  t.start(1);
  // clearlyBetter's gain is a constant relative diff every pair (the
  // multiplicative model cancels aRate()'s jitter -- SE stays exactly 0), so
  // the sequential test decides at the first look, MIN_LOOK_PAIRS, long
  // before the schedule ends.
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyBetter);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_EQ(0, s.final_value);       // B = the opposite of the original 1
  EXPECT_EQ(0, s.value);             // and it is applied
  EXPECT_EQ(TrialFSM::DONE, t.state());
  EXPECT_EQ(TrialFSM::MIN_LOOK_PAIRS, t.stats().n_pairs);
  EXPECT_GT(t.stats().mean_rel_x1000, 0);
}

TEST(TrialFSM, KeepsAWhenBIsWorse) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyWorse);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(TrialFSM::MIN_LOOK_PAIRS, t.stats().n_pairs);
  EXPECT_LT(t.stats().mean_rel_x1000, 0);
}

TEST(TrialFSM, KeepsAWhenTheDifferenceIsWithinNoise) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 24, noise);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(1, s.final_value);
}

TEST(TrialFSM, EndsUndecidedWithFewerThanMinLookPairsEvenIfBLooksBetter) {
  TrialFSM t;
  t.begin(cfg(4));   // only 2 pairs < MIN_LOOK_PAIRS
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 2, clearlyBetter);
  ASSERT_TRUE(s.finished);   // the schedule ends
  EXPECT_EQ(TrialFSM::UNDECIDED, s.outcome);
  EXPECT_EQ(1, s.final_value);   // A kept
  EXPECT_EQ(2u, t.stats().n_pairs);
}

TEST(TrialFSM, AliveBAgainstADeadAAdoptsBWithoutWaitingForPairs) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  // A hears nothing, B about 12 a block: no measured pair ever forms, but the
  // arms' RX counts alone settle it after two pairs (B alive in two blocks).
  TrialFSM::Step s = runHeard(t, 0, 96, 0, 12);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ALIVE_B, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(1, s.value);
  EXPECT_EQ(4, t.blockIndex());
  EXPECT_EQ(0u, t.stats().n_pairs);
}

TEST(TrialFSM, DeadBAgainstAnAliveAKeepsA) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  TrialFSM::Step s = runHeard(t, 0, 96, 12, 0);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ALIVE_A, s.outcome);
  EXPECT_EQ(0, s.final_value);
}

TEST(TrialFSM, NearlyDeadArmStillLosesToAnAliveOne) {
  TrialFSM t;
  t.begin(cfg(16));   // 8 pairs: a stricter per-look threshold than 96 blocks
  t.start(0);
  // the fem.rxgain night: A hears a packet now and then, B tens a block
  TrialFSM::Step s = runHeard(t, 0, 16, 1, 12);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ALIVE_B, s.outcome);
}

TEST(TrialFSM, OneBurstyBlockIsNotEnoughToCallAnArmAlive) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  t.onBlock(heard(0), 0, 0);    // A
  t.onBlock(heard(60), 0, 0);   // B: one burst, capped
  // then both arms hear the same: no dead arm
  TrialFSM::Step s = runHeard(t, 0, 8, 10, 10);
  EXPECT_NE(TrialFSM::ALIVE_B, s.outcome);
  EXPECT_NE(TrialFSM::ALIVE_A, s.outcome);
}

TEST(TrialFSM, TwiceTheTrafficIsNotADeadArm) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  TrialFSM::Step s = runHeard(t, 0, 96, 4, 8);
  EXPECT_NE(TrialFSM::ALIVE_B, s.outcome);
  EXPECT_NE(TrialFSM::ALIVE_A, s.outcome);
}

TEST(TrialFSM, InvalidatedBlocksDoNotCountAsDead) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(0);
  // every A block invalidated (a config change, ...), not silent: no verdict on A
  TrialFSM::Step s = {};
  for (int i = 0; i < 12 && !s.finished; i++) {
    s = t.onBlock(t.currentValue() == 0 ? unmeasured(EVAL_INVALIDATED, EVALF_CONFIG) : heard(12), 0, 0);
  }
  EXPECT_NE(TrialFSM::ALIVE_B, s.outcome);
}

TEST(TrialFSM, AQuietCycleEndsTheTrialIdle) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // one packet in a whole A B B A cycle: nothing to decide on, stop
  t.onBlock(heard(0), 0, 0);
  t.onBlock(heard(1), 0, 0);
  t.onBlock(heard(0), 0, 0);
  TrialFSM::Step s = t.onBlock(heard(0), 0, 0);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ABORTED_IDLE, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(1, s.value);
}

TEST(TrialFSM, EnoughTrafficInACycleKeepsRunning) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  TrialFSM::Step s = runHeard(t, 1, 4, 1, 1);   // one packet a block: the floor
  EXPECT_FALSE(s.finished);
}

TEST(TrialFSM, IdleMinRxZeroTurnsTheIdleStopOff) {
  TrialFSM t;
  TrialFSM::Config c = cfg(8);
  c.idle_min_rx = 0;
  t.begin(c);
  t.start(1);
  TrialFSM::Step s = runHeard(t, 1, 8, 0, 0);   // silent to the end of the schedule
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::UNDECIDED, s.outcome);
  EXPECT_EQ(8, t.blockIndex());
}

TEST(TrialFSM, ACycleWithAnInvalidatedBlockIsNotJudgedIdle) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(heard(0), 0, 0);
  t.onBlock(unmeasured(EVAL_INVALIDATED, EVALF_CONFIG), 0, 0);
  t.onBlock(heard(0), 0, 0);
  TrialFSM::Step s = t.onBlock(heard(0), 0, 0);
  EXPECT_FALSE(s.finished);
}

TEST(TrialFSM, BinomialTailIsTwoSidedAndCapped) {
  EXPECT_NEAR(1.0, TrialFSM::binomialTwoSided(0, 0), 1e-12);
  EXPECT_NEAR(2.0 / 65536.0, TrialFSM::binomialTwoSided(16, 0), 1e-12);
  EXPECT_NEAR(2.0 * 18.0 / 131072.0, TrialFSM::binomialTwoSided(17, 1), 1e-12);
  EXPECT_NEAR(1.0, TrialFSM::binomialTwoSided(10, 5), 1e-12);
  EXPECT_GT(TrialFSM::binomialTwoSided(768, 0), 0.0);   // no underflow to a NaN
}

TEST(TrialFSM, StopsEarlyWhenTheGainIsBelowTheWorthwhileBand) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // +1% is real but well under the default 5% minimum gain: no reason to keep
  // measuring, keep A.
  TrialFSM::Step s = runPairs(t, 1, 48, [](uint32_t a) { return a * 101 / 100; });
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_LT(t.stats().n_pairs, 48u);
}

TEST(TrialFSM, RatioBelowToleranceBlocksAdoptionAndTheStableTrialEndsInconclusive) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // The goodput gain alone would adopt at once, but a confirm ratio far worse
  // than RATIO_TOLERANCE blocks it, and the gain is above the minimum so the
  // trial does not stop for "no gain" either. Undecided, but the bounds stop
  // moving, so it ends inconclusive (A kept) long before the schedule does.
  TrialFSM::Step s = runPairs(t, 1, 48, clearlyBetter, /*a_ratio=*/9000, /*b_ratio=*/8500);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::INCONCLUSIVE, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_LT(t.stats().n_pairs, 48u);
  EXPECT_GE(t.stats().n_pairs, TrialFSM::MIN_LOOK_PAIRS + TrialFSM::STABLE_PAIRS);
}

TEST(TrialFSM, NoiseDoesNotDecideWithinAFewPairs) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // noise() has real spread (SE > 0) and no consistent direction, so a handful
  // of pairs must not decide either way -- still running, no false positive.
  TrialFSM::Step s = runPairs(t, 1, (int)TrialFSM::MIN_LOOK_PAIRS + 1, noise);
  EXPECT_FALSE(s.finished);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, NoisyRealGainIsAdoptedWithinTheSchedule) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  // +20% with a per-block wobble: a decision needs more than 3 pairs but
  // does not need the whole schedule.
  TrialFSM::Step s = runPairs(t, 1, 24, [](uint32_t a) { return a * 6 / 5 + (a % 7) - 3; });
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_GE(t.stats().n_pairs, TrialFSM::MIN_LOOK_PAIRS);
}

TEST(TrialFSM, TCritTablesAndClamping) {
  EXPECT_FLOAT_EQ(2.776f, TrialFSM::tCrit(4, 5));
  EXPECT_FLOAT_EQ(1.812f, TrialFSM::tCrit(10, 10));
  EXPECT_FLOAT_EQ(3.169f, TrialFSM::tCrit(10, 1));
  EXPECT_FLOAT_EQ(2.776f, TrialFSM::tCrit(4, 7));    // unknown alpha uses 5%
  EXPECT_FLOAT_EQ(12.706f, TrialFSM::tCrit(0, 5));   // df floors at 1
  EXPECT_FLOAT_EQ(2.042f, TrialFSM::tCrit(35, 5));   // between tabulated: the lower bracket
  EXPECT_FLOAT_EQ(2.021f, TrialFSM::tCrit(40, 5));
  EXPECT_FLOAT_EQ(1.960f, TrialFSM::tCrit(500, 5));  // normal quantile
}

TEST(TrialFSM, KeepsAWhenBGainsVolumeButConfirmRatioIsWorse) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyBetter, /*a_ratio=*/9000, /*b_ratio=*/8500);
  ASSERT_TRUE(s.finished);
  EXPECT_NE(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_EQ(1, s.final_value);
}

TEST(TrialFSM, NoStableStopWithoutEnoughLooks) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, (int)(TrialFSM::MIN_LOOK_PAIRS + TrialFSM::STABLE_PAIRS) - 1,
                              clearlyBetter, 9000, 8500);
  EXPECT_FALSE(s.finished);
}

TEST(TrialFSM, ProgressReportsPairsMeanAndBoundsFromTheThirdPair) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  EXPECT_EQ(0u, t.progress().n_pairs);
  EXPECT_FALSE(t.progress().bounds);
  runPairs(t, 1, 2, noise);
  TrialFSM::Progress p = t.progress();
  EXPECT_EQ(2u, p.n_pairs);
  EXPECT_FALSE(p.bounds);
  runPairs(t, 1, 1, noise);
  p = t.progress();
  EXPECT_EQ(3u, p.n_pairs);
  EXPECT_TRUE(p.bounds);
  EXPECT_LT(p.lower, p.mean);
  EXPECT_GT(p.upper, p.mean);
}

TEST(TrialFSM, GuardrailTripAbortsAndRevertsToTheOriginal) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100), 0, 0);                 // A
  ASSERT_EQ(0, t.currentValue());            // B running
  TrialFSM::Step s = t.onBlock(blk(100, 9000, EVALF_GUARDRAIL), 0, 0);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ABORTED_GUARDRAIL, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(1, s.value);
  EXPECT_EQ(TrialFSM::DONE, t.state());
}

TEST(TrialFSM, BBlockRatioDropDoesNotAbort) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100, 9000), 0, 0);           // A
  TrialFSM::Step s = t.onBlock(blk(100, 1000), 0, 0);  // B: the score and the adopt veto judge it
  EXPECT_FALSE(s.finished);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, ABlockRatioDropDoesNotAbort) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100, 9000), 0, 0);           // A
  t.onBlock(blk(100, 9000), 0, 0);           // B
  t.onBlock(blk(100, 9000), 0, 0);           // B
  TrialFSM::Step s = t.onBlock(blk(100, 1000), 0, 0);   // A collapses: not B's fault
  EXPECT_FALSE(s.finished);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, UnmeasuredBlockDiscardsItsPairButTheTrialContinues) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(unmeasured(EVAL_INSUFFICIENT_DATA), 0, 0);   // A
  t.onBlock(blk(120), 0, 0);                               // B: pair invalid
  t.onBlock(blk(120), 0, 0);                               // B
  t.onBlock(blk(100), 0, 0);                               // A: valid pair
  EXPECT_EQ(1u, t.stats().n_pairs);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, InvalidatedBlockIsDiscardedToo) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100), 0, 0);
  t.onBlock(unmeasured(EVAL_INVALIDATED, EVALF_CONFIG), 0, 0);
  EXPECT_EQ(0u, t.stats().n_pairs);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, PairOfZeroGoodputBlocksCountsAsNoDifference) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(0, 0), 0, 0);
  t.onBlock(blk(0, 0), 0, 0);
  // both floored the same: a real (measured) pair whose difference is 0
  EXPECT_EQ(1u, t.stats().n_pairs);
  EXPECT_EQ(0, t.stats().mean_rel_x1000);
}

TEST(TrialFSM, DeadBlockOnOneSideIsALargeFiniteDifference) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(200, 9000), 0, 0);   // A
  t.onBlock(blk(0, 8000), 0, 0);     // B: no deliveries (the ratio guard tolerates 1500)
  EXPECT_EQ(1u, t.stats().n_pairs);
  // routed 200/h -> 0: ln(1 + 0) - ln(1 + 9.1/0.1) plus the ratio's small step, ~ -4.6
  EXPECT_LT(t.stats().mean_rel_x1000, -3000);
  EXPECT_GT(t.stats().mean_rel_x1000, -8000);
}

TEST(TrialFSM, BlocksAreRoundedDownToWholePairsWithAMinimumOfTwo) {
  TrialFSM t;
  t.begin(cfg(7));
  EXPECT_EQ(6, t.totalBlocks());
  t.begin(cfg(1));
  EXPECT_EQ(2, t.totalBlocks());
}

TEST(TrialFSM, ExplicitAbortRevertsAndFinishes) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100), 0, 0);
  ASSERT_EQ(0, t.currentValue());
  TrialFSM::Step s = t.abort();
  EXPECT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ABORTED, s.outcome);
  EXPECT_EQ(1, s.value);
  EXPECT_EQ(TrialFSM::DONE, t.state());
}

TEST(TrialFSM, ReachIsAveragedPerValueForDiagnostics) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100), /*heard=*/10, /*marginal=*/2);   // A
  t.onBlock(blk(100), 16, 5);                           // B
  t.onBlock(blk(100), 14, 3);                           // B
  t.onBlock(blk(100), 12, 1);                           // A
  EXPECT_EQ(11, t.stats().reach_heard_a_x10 / 10);      // (10+12)/2
  EXPECT_EQ(15, t.stats().reach_heard_b_x10 / 10);      // (16+14)/2
  EXPECT_EQ(15, t.stats().reach_marginal_a_x10);        // (2+1)/2 = 1.5
  EXPECT_EQ(40, t.stats().reach_marginal_b_x10);        // (5+3)/2 = 4.0
}

TEST(TrialFSM, CanRestartAfterDone) {
  TrialFSM t;
  t.begin(cfg(2));
  t.start(1);
  t.onBlock(blk(100), 0, 0);
  TrialFSM::Step s = t.onBlock(blk(100), 0, 0);
  ASSERT_TRUE(s.finished);
  ASSERT_TRUE(t.start(0));
  EXPECT_EQ(TrialFSM::RUN, t.state());
  EXPECT_EQ(0u, t.stats().n_pairs);
  EXPECT_EQ(0, t.blockIndex());
}

TEST(TrialFSM, OnBlockWhenNotRunningIsInert) {
  TrialFSM t;
  t.begin(cfg(96));
  TrialFSM::Step s = t.onBlock(blk(100), 0, 0);
  EXPECT_FALSE(s.finished);
  EXPECT_FALSE(s.set_value);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(TrialFSM, ExplicitAlternativeValueRunsAbbaAndAdoptsIt) {
  TrialFSM t;
  t.begin(cfg(48));
  ASSERT_TRUE(t.start(5, 8));   // coding rate 5 vs 8
  EXPECT_EQ(5, t.original());
  EXPECT_EQ(8, t.alternative());
  const uint8_t expect[8] = {5, 8, 8, 5, 5, 8, 8, 5};
  TrialFSM probe;
  probe.begin(cfg(48));
  probe.start(5, 8);
  for (int i = 0; i < 8; i++) {
    EXPECT_EQ(expect[i], probe.currentValue());
    probe.onBlock(blk(100 + (i * 37) % 29), 0, 0);
  }
  TrialFSM::Step s = runPairs(t, 5, 24, clearlyBetter);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_EQ(8, s.final_value);
}

TEST(TrialFSM, ExplicitAlternativeKeepsTheOriginalWhenWorse) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(6, 8);
  TrialFSM::Step s = runPairs(t, 6, 24, clearlyWorse);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(6, s.final_value);
}

TEST(TrialFSM, ObjectiveWeightsChangeWhatTheTrialOptimizes) {
  // B has the same goodput as A but hears twice as many valid packets.
  auto run = [](const Objective *obj) {
    TrialFSM t;
    TrialFSM::Config c = cfg(48);
    c.objective = obj;
    t.begin(c);
    t.start(1);
    TrialFSM::Step s = {};
    for (int i = 0; i < 48 && !s.finished; i++) {
      bool is_b = t.currentValue() != 1;
      EvalWindow::Result r = blk(100 + (i % 3) * 5, 9000);
      r.window_ms = 1800000;
      r.rx_valid = is_b ? 200 : 100;
      s = t.onBlock(r, 0, 0);
    }
    return s;
  };
  Objective goodput;                  // rx_valid weight 0: no difference, keeps A
  TrialFSM::Step a = run(&goodput);
  ASSERT_TRUE(a.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, a.outcome);
  Objective rx;
  rx.weights[Objective::RX_VALID] = 10;   // rx_valid counts: B wins
  TrialFSM::Step b = run(&rx);
  ASSERT_TRUE(b.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, b.outcome);
}
