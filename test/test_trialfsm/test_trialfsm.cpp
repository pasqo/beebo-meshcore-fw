#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <helpers/TrialFSM.h>

namespace {

// A closed block as TrialFSM::onBlock() receives it. `ros_rate` is confirmed
// deliveries per hour, `ratio` the confirm ratio (0-10000).
EvalWindow::Result blk(uint32_t ros_rate, uint16_t ratio = 9000, uint8_t flags = 0) {
  EvalWindow::Result r = {};
  r.measured = true;
  r.outcome = EVAL_ACCEPTED;
  r.ros_rate = ros_rate;
  r.confirm_ratio = ratio;
  r.flags = flags;
  r.exposure = 200;
  return r;
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
    t.onBlock(blk(100), 0, 0);
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

TEST(TrialFSM, AdoptsBWhenClearlyBetterOverEnoughPairs) {
  TrialFSM t;
  t.begin(cfg(48));   // 24 pairs available
  t.start(1);
  // clearlyBetter's gain is a constant relative diff every pair (the
  // multiplicative model cancels aRate()'s jitter -- SE stays exactly 0),
  // so the pragmatic early stop (EARLY_STOP_MIN_PAIRS=5) fires well before
  // all 24 run; that IS the well-before-MIN_PAIRS(20) case this now covers.
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyBetter);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_EQ(0, s.final_value);       // B = the opposite of the original 1
  EXPECT_EQ(0, s.value);             // and it is applied
  EXPECT_EQ(TrialFSM::DONE, t.state());
  EXPECT_EQ(TrialFSM::EARLY_STOP_MIN_PAIRS, t.stats().n_pairs);
  EXPECT_GT(t.stats().mean_rel_x1000, 0);
}

TEST(TrialFSM, KeepsAWhenBIsWorse) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  // clearlyWorse is the mirror case: early stop's losing branch fires at
  // the same floor, well before the full 24 pairs or MIN_PAIRS(20).
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyWorse);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(TrialFSM::EARLY_STOP_MIN_PAIRS, t.stats().n_pairs);
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

TEST(TrialFSM, KeepsAWithTooFewValidPairsEvenIfBLooksBetter) {
  TrialFSM t;
  // Below EARLY_STOP_MIN_PAIRS too, not just MIN_PAIRS -- this exercises
  // decide()'s own floor at the end of the schedule, not the early-stop path
  // (see EarlyStopAdoptsBWellBeforeMinPairs for that).
  ASSERT_LT((int)TrialFSM::EARLY_STOP_MIN_PAIRS - 1, 5);
  t.begin(cfg(8));   // only 4 pairs < both floors
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 4, clearlyBetter);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(4u, t.stats().n_pairs);
}

TEST(TrialFSM, EarlyStopAdoptsBWellBeforeMinPairs) {
  TrialFSM t;
  t.begin(cfg(96));   // 48 pairs available -- far more than needed
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 48, clearlyBetter);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ADOPT_B, s.outcome);
  EXPECT_EQ(TrialFSM::EARLY_STOP_MIN_PAIRS, t.stats().n_pairs);
  EXPECT_LT(t.stats().n_pairs, TrialFSM::MIN_PAIRS);
  EXPECT_LT(t.blockIndex(), t.totalBlocks());   // really stopped early
}

TEST(TrialFSM, EarlyStopDoesNotFireOnRatioTooFarBelowTolerance) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // clearlyBetter's goodput gain alone would trigger the early stop, but a
  // confirm ratio far worse than RATIO_TOLERANCE must still block it, same
  // as the equivalent full-schedule case (KeepsAWhenBGainsVolumeButConfirmRatioIsWorse).
  TrialFSM::Step s = runPairs(t, 1, 48, clearlyBetter, /*a_ratio=*/9000, /*b_ratio=*/8500);
  ASSERT_TRUE(s.finished);   // still finishes -- via decide() at schedule end
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
  EXPECT_EQ(48u, t.stats().n_pairs);   // ran the full schedule, never early-stopped
}

TEST(TrialFSM, EarlyStopDoesNotFireOnNoise) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  // noise() has real spread (SE > 0) and no consistent direction, so it
  // must not cross the strict EARLY_STOP_Z bar within a handful of pairs --
  // confirm the trial is still running, not finished on a false positive.
  TrialFSM::Step s = runPairs(t, 1, (int)TrialFSM::EARLY_STOP_MIN_PAIRS + 1, noise);
  EXPECT_FALSE(s.finished);
  EXPECT_EQ(TrialFSM::RUN, t.state());
}

TEST(TrialFSM, KeepsAWhenBGainsVolumeButConfirmRatioIsWorse) {
  TrialFSM t;
  t.begin(cfg(48));
  t.start(1);
  TrialFSM::Step s = runPairs(t, 1, 24, clearlyBetter, /*a_ratio=*/9000, /*b_ratio=*/8500);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::KEEP_A, s.outcome);
}

TEST(TrialFSM, GuardrailTripAbortsAndRevertsToTheOriginal) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100), 0, 0);                 // A
  ASSERT_EQ(0, t.currentValue());            // B running
  TrialFSM::Step s = t.onBlock(blk(100, 9000, EVALF_GUARDRAIL), 0, 0);
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ABORTED, s.outcome);
  EXPECT_EQ(1, s.final_value);
  EXPECT_EQ(1, s.value);
  EXPECT_EQ(TrialFSM::DONE, t.state());
}

TEST(TrialFSM, BBlockRatioCollapseVsATheAbortsTheTrial) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(100, 9000), 0, 0);           // A
  TrialFSM::Step s = t.onBlock(blk(100, 9000 - TrialFSM::COLLAPSE_THRESHOLD - 1), 0, 0);  // B
  ASSERT_TRUE(s.finished);
  EXPECT_EQ(TrialFSM::ABORTED, s.outcome);
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

TEST(TrialFSM, ZeroGoodputPairIsSkipped) {
  TrialFSM t;
  t.begin(cfg(96));
  t.start(1);
  t.onBlock(blk(0, 0), 0, 0);
  t.onBlock(blk(0, 0), 0, 0);
  EXPECT_EQ(0u, t.stats().n_pairs);
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
    probe.onBlock(blk(100), 0, 0);
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
