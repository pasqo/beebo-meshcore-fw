#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <helpers/EvalWindow.h>

namespace {

using Stats = MonRing::QosStats;

Stats stats(uint32_t ack_ok, uint32_t ack_to, uint32_t echo_attempt, uint32_t echo_ok) {
  return Stats{ack_ok, ack_to, echo_attempt, echo_ok};
}

EvalWindow::Config cfg(uint32_t min_ms = 300000, uint32_t max_ms = 3600000) {
  EvalWindow::Config c;
  c.min_ms = min_ms;
  c.max_ms = max_ms;
  return c;
}

// Run one window of `exposure` confirmable attempts (`ok` confirmed) and
// `rx_delta` received packets, starting at `t0`, lasting `dur_ms`; returns
// the closed result. `rx` is the caller's running lifetime RX count, threaded
// through like `cur` -- rx_delta defaults to 1 so callers testing baseline/
// guardrail/rate math (not validity itself) don't need to think about RX.
EvalWindow::Result runWindow(EvalWindow &w, uint32_t t0, Stats &cur, uint32_t &rx,
                             uint32_t exposure, uint32_t ok, uint32_t dur_ms = 300000,
                             uint32_t rx_delta = 1) {
  w.open(t0, cur, 1, 0, rx);
  cur.echo_attempt_count += exposure;
  cur.echo_success_count += ok;
  rx += rx_delta;
  EvalWindow::Result r{};
  EXPECT_TRUE(w.poll(t0 + dur_ms, cur, 0, r, rx));
  return r;
}

}  // namespace

TEST(EvalWindow, PollBeforeOpenReturnsFalse) {
  EvalWindow w;
  w.begin(cfg());
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(1000, stats(0, 0, 0, 0), 0, r, 1));
}

TEST(EvalWindow, DoesNotCloseBeforeMinDurationEvenWithRxActivity) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(299999, stats(0, 0, 100, 90), 0, r, 500));
  EXPECT_TRUE(w.poll(300000, stats(0, 0, 100, 90), 0, r, 500));
}

TEST(EvalWindow, StaysOpenPastMinDurationUntilRxActivity) {
  EvalWindow w;
  w.begin(cfg(300000, 3600000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 1000);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(900000, stats(0, 0, 0, 0), 0, r, 1000));    // 15 min, no RX yet
  EXPECT_TRUE(w.poll(1200000, stats(0, 0, 0, 0), 0, r, 1001));    // 20 min, one packet heard
  EXPECT_EQ(1200000u, r.window_ms);
}

TEST(EvalWindow, InsufficientDataAtMaxDurationWithNoRxActivityAtAll) {
  EvalWindow w;
  w.begin(cfg(300000, 3600000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 1000);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(3599999, stats(0, 0, 10, 8), 0, r, 1000));
  ASSERT_TRUE(w.poll(3600000, stats(0, 0, 10, 8), 0, r, 1000));   // rx delta still 0
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
  EXPECT_EQ(10u, r.exposure);   // still reported, just diagnostic now
}

TEST(EvalWindow, ZeroExposureWithRealRxIsStillMeasured) {
  // The core of the RX-activity gate: a window with real RX but zero
  // confirmable attempts (nothing forwarded) is real information for a
  // repeater -- not discarded as insufficient_data. confirm_ratio is a
  // genuine 0% (computeQos()'s own zero-denominator convention), not a
  // fabricated one, since the window is measured.
  EvalWindow w;
  EvalWindow::Config c = cfg();
  c.use_baseline = false;   // isolate the RX-gate behavior from cold-start
  w.begin(c);
  w.open(0, stats(5, 5, 5, 5), 1, 0, 1000);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(3600000, stats(5, 5, 5, 5), 0, r, 1200));   // 200 packets heard, 0 forwarded
  EXPECT_TRUE(r.measured);
  EXPECT_EQ(EVAL_ACCEPTED, r.outcome);
  EXPECT_EQ(0u, r.exposure);
  EXPECT_EQ(0u, r.confirm_ratio);
}

TEST(EvalWindow, ZeroRxActivityIsInsufficientRegardlessOfExposure) {
  // The mirror case: even with real confirmable-attempt exposure, zero RX
  // activity in the window means genuinely nothing happened (no new
  // information at all) -- this is the one case still worth discarding.
  EvalWindow w;
  w.begin(cfg(100, 200));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 500);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(200, stats(0, 0, 50, 40), 0, r, 500));   // exposure=50, rx delta=0
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
  EXPECT_EQ(50u, r.exposure);
}

TEST(EvalWindow, UsesCounterDeltasNotLifetimeTotals) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(1000000, 5000, 900000, 800000);
  w.open(0, cur, 1, 0, 2000000);
  cur.ack_success_count += 30;
  cur.ack_timeout_count += 10;
  cur.echo_attempt_count += 20;
  cur.echo_success_count += 15;
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, cur, 0, r, 2000010));   // 10 packets heard this window
  EXPECT_EQ(60u, r.exposure);      // 30 + 10 + 20
  EXPECT_EQ(45u, r.ros_count);     // 30 + 15
  EXPECT_EQ(7500u, r.confirm_ratio);
}

TEST(EvalWindow, CounterDecreaseInvalidatesAsCounterReset) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(100, 0, 100, 100), 1, 0, 1000);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(1000, stats(0, 0, 3, 2), 0, r, 1000));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_COUNTER_RST);
}

TEST(EvalWindow, RxCounterDecreaseAlsoInvalidatesAsCounterReset) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1, 0, 1000);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(1000, stats(0, 0, 0, 0), 0, r, 3));   // rx went backwards: device reset
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_COUNTER_RST);
}

TEST(EvalWindow, FlagInvalidateClosesImmediatelyWithThatFlag) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  w.invalidate(EVALF_CONFIG);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(10, stats(0, 0, 500, 500), 0, r, 500));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_CONFIG);
}

TEST(EvalWindow, InvalidatedWindowDoesNotFeedBaseline) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  w.invalidate(EVALF_CAPTURE_GAP);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(10, stats(0, 0, 500, 500), 0, r, 500));
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, ColdStartIsInsufficientUntilBaselineFilled) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < EvalWindow::MIN_BASELINE; i++) {
    EvalWindow::Result r = runWindow(w, i * 300000u, cur, rx, 50, 40);
    EXPECT_FALSE(r.measured) << "window " << i;
    EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
    EXPECT_EQ(i, r.n_baseline);  // baseline size BEFORE this window
  }
  EXPECT_EQ(EvalWindow::MIN_BASELINE, w.baselineCount());
  EvalWindow::Result r = runWindow(w, 10 * 300000u, cur, rx, 50, 40);
  EXPECT_TRUE(r.measured);
  EXPECT_EQ(EvalWindow::MIN_BASELINE, r.n_baseline);
}

TEST(EvalWindow, RosNormIsFixedPointOneThousandAtBaseline) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 4; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);  // 40 ros / 5 min = 480/h
  EvalWindow::Result same = runWindow(w, 4 * 300000u, cur, rx, 50, 40);
  ASSERT_TRUE(same.measured);
  EXPECT_EQ(480u, same.baseline_ros_rate);
  EXPECT_EQ(1000u, same.ros_norm);
  EvalWindow::Result dbl = runWindow(w, 5 * 300000u, cur, rx, 100, 80);
  EXPECT_EQ(2000u, dbl.ros_norm);
}

TEST(EvalWindow, RateIsPerHourIndependentOfWindowLength) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  EvalWindow::Result r = runWindow(w, 0, cur, rx, 50, 40, 600000);  // 40 ros in 10 min
  EXPECT_EQ(240u, r.ros_rate);
}

TEST(EvalWindow, BaselineRollsOverLastEightWindows) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 20; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  EXPECT_EQ(EvalWindow::BASELINE_WINDOWS, w.baselineCount());
  // eight windows of a new, higher rate fully replace the old median
  for (int i = 0; i < 8; i++) runWindow(w, (20 + i) * 300000u, cur, rx, 100, 80);
  EvalWindow::Result r = runWindow(w, 28 * 300000u, cur, rx, 100, 80);
  EXPECT_EQ(960u, r.baseline_ros_rate);
}

TEST(EvalWindow, BaselineMedianIgnoresASingleOutlierWindow) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 5; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  runWindow(w, 5 * 300000u, cur, rx, 5000, 4000);   // burst window
  EvalWindow::Result r = runWindow(w, 6 * 300000u, cur, rx, 50, 40);
  EXPECT_EQ(480u, r.baseline_ros_rate);
}

TEST(EvalWindow, InsufficientWindowsDoNotFeedBaseline) {
  EvalWindow w;
  w.begin(cfg(300000, 900000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 500);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(900000, stats(0, 0, 10, 9), 0, r, 500));   // rx delta 0: insufficient
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, UtilIsMeanOfSamplesOverTheWindow) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  w.sampleUtil(20); w.sampleUtil(40); w.sampleUtil(60);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 0, r, 500));
  EXPECT_EQ(40u, r.util_pct);
}

TEST(EvalWindow, NoUtilSamplesReportsZero) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 0, r, 500));
  EXPECT_EQ(0u, r.util_pct);
}

TEST(EvalWindow, UtilSamplesResetWhenNextWindowOpens) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  w.sampleUtil(90);
  w.open(300000, stats(0, 0, 0, 0), 2, 0, 0);
  w.sampleUtil(10);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(600000, stats(0, 0, 50, 40), 0, r, 500));
  EXPECT_EQ(10u, r.util_pct);
}

TEST(EvalWindow, UtilSamplesIgnoredWhileNoWindowOpen) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.sampleUtil(100);
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 0, r, 500));
  EXPECT_EQ(0u, r.util_pct);
}

TEST(EvalWindow, CadBusyPctIsCumulativeDeltaOverWindow) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, /*cad_wait_ms=*/1000, 0);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 1000 + 60000, r, 500));  // 60 s of 300 s
  EXPECT_EQ(20u, r.cad_busy_pct);
}

TEST(EvalWindow, CadBusyPctClampsAtOneHundred) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 900000, r, 500));
  EXPECT_EQ(100u, r.cad_busy_pct);
}

TEST(EvalWindow, MeanUtilizationAboveThresholdSetsGuardrailButStaysMeasured) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 4; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  w.open(4 * 300000u, cur, 9, 0, rx);
  w.sampleUtil(100); w.sampleUtil(100); w.sampleUtil(50);   // mean 83
  cur.echo_attempt_count += 50;
  cur.echo_success_count += 40;
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(5 * 300000u, cur, 0, r, rx + 500));
  EXPECT_TRUE(r.measured);
  EXPECT_TRUE(r.flags & EVALF_GUARDRAIL);
  EXPECT_EQ(83u, r.util_pct);
}

TEST(EvalWindow, MeanUtilizationAtThresholdDoesNotTripGuardrail) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 4; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  w.open(4 * 300000u, cur, 9, 0, rx);
  w.sampleUtil(EvalWindow::GUARDRAIL_UTIL_PCT);
  cur.echo_attempt_count += 50;
  cur.echo_success_count += 40;
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(5 * 300000u, cur, 0, r, rx + 500));
  EXPECT_FALSE(r.flags & EVALF_GUARDRAIL);
}

TEST(EvalWindow, SetConfigInvalidatesOpenWindowAsConfigChange) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  w.setConfig(cfg(60000, 120000));
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(10, stats(0, 0, 500, 500), 0, r, 500));
  EXPECT_FALSE(r.measured);
  EXPECT_TRUE(r.flags & EVALF_CONFIG);
}

TEST(EvalWindow, SetConfigKeepsBaselineAndAppliesToNextWindow) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 5; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  ASSERT_EQ(5u, w.baselineCount());
  w.setConfig(cfg(60000, 120000));
  EXPECT_EQ(5u, w.baselineCount());
  w.open(0, cur, 2, 0, rx);
  cur.echo_attempt_count += 10;
  cur.echo_success_count += 8;
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(59999, cur, 0, r, rx));          // no RX yet
  EXPECT_TRUE(w.poll(60000, cur, 0, r, rx + 1));       // new min_ms in force, one packet heard
}

TEST(EvalWindow, SetConfigWithNoOpenWindowIsHarmless) {
  EvalWindow w;
  w.begin(cfg());
  w.setConfig(cfg(60000, 120000));
  EXPECT_FALSE(w.isOpen());
}

TEST(EvalWindow, WithoutBaselineTheFirstExposedWindowIsMeasured) {
  EvalWindow w;
  EvalWindow::Config c = cfg(300000, 300000);
  c.use_baseline = false;
  w.begin(c);
  w.open(0, stats(0, 0, 0, 0), 1, 0, 0);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 0, r, 500));
  EXPECT_TRUE(r.measured);
  EXPECT_EQ(480u, r.ros_rate);
  EXPECT_EQ(0u, r.baseline_ros_rate);
  EXPECT_EQ(0u, r.ros_norm);
  EXPECT_EQ(0u, w.baselineCount());   // trial windows never feed a baseline
}

TEST(EvalWindow, WithoutBaselineAWindowWithNoRxIsStillInsufficient) {
  EvalWindow w;
  EvalWindow::Config c = cfg(300000, 300000);
  c.use_baseline = false;
  w.begin(c);
  w.open(0, stats(0, 0, 0, 0), 1, 0, 500);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 10, 8), 0, r, 500));   // rx delta 0
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
}

TEST(EvalWindow, ResetBaselineClearsHistory) {
  EvalWindow w;
  w.begin(cfg(300000));
  Stats cur = stats(0, 0, 0, 0);
  uint32_t rx = 0;
  for (int i = 0; i < 5; i++) runWindow(w, i * 300000u, cur, rx, 50, 40);
  w.resetBaseline();
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, ToRecordsFillsAllThreeSlotsAndSaturates) {
  EvalWindow::Result r{};
  r.flags = EVALF_GUARDRAIL;
  r.exposure = 100000;      // > u16
  r.ros_count = 70000;
  r.confirm_ratio = 9400;
  r.util_pct = 55;
  r.window_ms = 900000;
  r.baseline_ros_rate = 480;
  r.ros_norm = 1000;
  r.n_baseline = 8;
  r.cad_busy_pct = 12;
  r.reach_heard = 17;
  r.reach_marginal = 4;
  r.rx_valid = 80000;       // > u16
  r.rx_invalid = 30;
  r.rx_errors = 9;
  r.tx_dispatched = 200;
  EvalRecordA a; EvalRecordB b; EvalRecordC c;
  EvalWindow::toRecords(r, 42, EVAL_ROLLBACK, a, b, c);
  EXPECT_EQ(42, a.window_id);
  EXPECT_EQ(EVAL_ROLLBACK, a.outcome);
  EXPECT_EQ(EVALF_GUARDRAIL, a.flags);
  EXPECT_EQ(65535, a.exposure);
  EXPECT_EQ(65535, a.ros_count);
  EXPECT_EQ(9400, a.confirm_ratio);
  EXPECT_EQ(55, a.util_pct);
  EXPECT_EQ(EvalWindow::RECORD_VERSION, a.version);
  EXPECT_EQ(42, b.window_id);
  EXPECT_EQ(900, b.window_s);
  EXPECT_EQ(480, b.baseline_ros_rate);
  EXPECT_EQ(1000, b.ros_norm);
  EXPECT_EQ(8, b.n_baseline);
  EXPECT_EQ(12, b.cad_busy_pct);
  EXPECT_EQ(17, b.reach_heard);
  EXPECT_EQ(4, b.reach_marginal);
  EXPECT_EQ(42, c.window_id);
  EXPECT_EQ(65535, c.rx_valid);
  EXPECT_EQ(30, c.rx_invalid);
  EXPECT_EQ(9, c.rx_errors);
  EXPECT_EQ(200, c.tx_dispatched);
}

TEST(EvalWindow, RxTxBreakdownIsCounterDeltasOverTheWindow) {
  EvalWindow w;
  w.begin(cfg(300000));
  w.open(0, stats(0, 0, 0, 0), 1, 0, /*rx_count=*/1000,
         /*rx_invalid=*/50, /*rx_errors=*/20, /*tx_count=*/300);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, stats(0, 0, 50, 40), 0, r,
                     /*rx_count=*/1010, /*rx_invalid=*/55, /*rx_errors=*/23, /*tx_count=*/312));
  EXPECT_EQ(10u, r.rx_valid);
  EXPECT_EQ(5u, r.rx_invalid);
  EXPECT_EQ(3u, r.rx_errors);
  EXPECT_EQ(12u, r.tx_dispatched);
}

TEST(EvalWindow, RxInvalidDecreaseAlsoInvalidatesAsCounterReset) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1, 0, /*rx_count=*/1000, /*rx_invalid=*/50);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(1000, stats(0, 0, 0, 0), 0, r, /*rx_count=*/1000, /*rx_invalid=*/3));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_COUNTER_RST);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
