#include <gtest/gtest.h>
#include <helpers/MonRing.h>
#include <helpers/EvalWindow.h>

namespace {

using Stats = MonRing::QosStats;

Stats stats(uint32_t ack_ok, uint32_t ack_to, uint32_t echo_attempt, uint32_t echo_ok) {
  return Stats{ack_ok, ack_to, echo_attempt, echo_ok};
}

EvalWindow::Config cfg(uint16_t min_exposure = 40, uint32_t min_ms = 300000,
                       uint32_t max_ms = 3600000) {
  EvalWindow::Config c;
  c.min_exposure = min_exposure;
  c.min_ms = min_ms;
  c.max_ms = max_ms;
  return c;
}

// Run one exposed window of `exposure` attempts (`ok` confirmed) starting at
// `t0`, lasting `min_ms`; returns the closed result.
EvalWindow::Result runWindow(EvalWindow &w, uint32_t t0, Stats &cur, uint32_t exposure,
                             uint32_t ok, uint32_t dur_ms = 300000) {
  w.open(t0, cur, 1);
  cur.echo_attempt_count += exposure;
  cur.echo_success_count += ok;
  EvalWindow::Result r{};
  EXPECT_TRUE(w.poll(t0 + dur_ms, cur, 10, 5, r));
  return r;
}

TEST(EvalWindow, PollBeforeOpenReturnsFalse) {
  EvalWindow w;
  w.begin(cfg());
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(1000, stats(0, 0, 0, 0), 0, 0, r));
}

TEST(EvalWindow, DoesNotCloseBeforeMinDurationEvenWithEnoughExposure) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  w.open(0, stats(0, 0, 0, 0), 1);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(299999, stats(0, 0, 100, 90), 0, 0, r));
  EXPECT_TRUE(w.poll(300000, stats(0, 0, 100, 90), 0, 0, r));
}

TEST(EvalWindow, StaysOpenPastMinDurationUntilExposureReached) {
  EvalWindow w;
  w.begin(cfg(40, 300000, 3600000));
  w.open(0, stats(0, 0, 0, 0), 1);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(900000, stats(0, 0, 39, 30), 0, 0, r));   // 15 min, 39 attempts
  EXPECT_TRUE(w.poll(1200000, stats(0, 0, 40, 30), 0, 0, r));   // 20 min, 40 attempts
  EXPECT_EQ(1200000u, r.window_ms);
}

TEST(EvalWindow, InsufficientDataAtMaxDurationWithoutEnoughExposure) {
  EvalWindow w;
  w.begin(cfg(40, 300000, 3600000));
  w.open(0, stats(0, 0, 0, 0), 1);
  EvalWindow::Result r{};
  EXPECT_FALSE(w.poll(3599999, stats(0, 0, 10, 8), 0, 0, r));
  ASSERT_TRUE(w.poll(3600000, stats(0, 0, 10, 8), 0, 0, r));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
  EXPECT_EQ(10u, r.exposure);
}

TEST(EvalWindow, ZeroExposureIsInsufficientNotAZeroRatio) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(5, 5, 5, 5), 1);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(3600000, stats(5, 5, 5, 5), 0, 0, r));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
  EXPECT_EQ(0u, r.exposure);
}

TEST(EvalWindow, UsesCounterDeltasNotLifetimeTotals) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(1000000, 5000, 900000, 800000);
  w.open(0, cur, 1);
  cur.ack_success_count += 30;
  cur.ack_timeout_count += 10;
  cur.echo_attempt_count += 20;
  cur.echo_success_count += 15;
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(300000, cur, 0, 0, r));
  EXPECT_EQ(60u, r.exposure);      // 30 + 10 + 20
  EXPECT_EQ(45u, r.ros_count);     // 30 + 15
  EXPECT_EQ(7500u, r.confirm_ratio);
}

TEST(EvalWindow, CounterDecreaseInvalidatesAsCounterReset) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(100, 0, 100, 100), 1);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(1000, stats(0, 0, 3, 2), 0, 0, r));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_COUNTER_RST);
}

TEST(EvalWindow, FlagInvalidateClosesImmediatelyWithThatFlag) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1);
  w.invalidate(EVALF_CONFIG);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(10, stats(0, 0, 500, 500), 0, 0, r));
  EXPECT_FALSE(r.measured);
  EXPECT_EQ(EVAL_INVALIDATED, r.outcome);
  EXPECT_TRUE(r.flags & EVALF_CONFIG);
}

TEST(EvalWindow, InvalidatedWindowDoesNotFeedBaseline) {
  EvalWindow w;
  w.begin(cfg());
  w.open(0, stats(0, 0, 0, 0), 1);
  w.invalidate(EVALF_CAPTURE_GAP);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(10, stats(0, 0, 500, 500), 0, 0, r));
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, ColdStartIsInsufficientUntilBaselineFilled) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < EvalWindow::MIN_BASELINE; i++) {
    EvalWindow::Result r = runWindow(w, i * 300000u, cur, 50, 40);
    EXPECT_FALSE(r.measured) << "window " << i;
    EXPECT_EQ(EVAL_INSUFFICIENT_DATA, r.outcome);
    EXPECT_EQ(i, r.n_baseline);  // baseline size BEFORE this window
  }
  EXPECT_EQ(EvalWindow::MIN_BASELINE, w.baselineCount());
  EvalWindow::Result r = runWindow(w, 10 * 300000u, cur, 50, 40);
  EXPECT_TRUE(r.measured);
  EXPECT_EQ(EvalWindow::MIN_BASELINE, r.n_baseline);
}

TEST(EvalWindow, RosNormIsFixedPointOneThousandAtBaseline) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < 4; i++) runWindow(w, i * 300000u, cur, 50, 40);  // 40 ros / 5 min = 480/h
  EvalWindow::Result same = runWindow(w, 4 * 300000u, cur, 50, 40);
  ASSERT_TRUE(same.measured);
  EXPECT_EQ(480u, same.baseline_ros_rate);
  EXPECT_EQ(1000u, same.ros_norm);
  EvalWindow::Result dbl = runWindow(w, 5 * 300000u, cur, 100, 80);
  EXPECT_EQ(2000u, dbl.ros_norm);
}

TEST(EvalWindow, RateIsPerHourIndependentOfWindowLength) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  EvalWindow::Result r = runWindow(w, 0, cur, 50, 40, 600000);  // 40 ros in 10 min
  EXPECT_EQ(240u, r.ros_rate);
}

TEST(EvalWindow, BaselineRollsOverLastEightWindows) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < 20; i++) runWindow(w, i * 300000u, cur, 50, 40);
  EXPECT_EQ(EvalWindow::BASELINE_WINDOWS, w.baselineCount());
  // eight windows of a new, higher rate fully replace the old median
  for (int i = 0; i < 8; i++) runWindow(w, (20 + i) * 300000u, cur, 100, 80);
  EvalWindow::Result r = runWindow(w, 28 * 300000u, cur, 100, 80);
  EXPECT_EQ(960u, r.baseline_ros_rate);
}

TEST(EvalWindow, BaselineMedianIgnoresASingleOutlierWindow) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < 5; i++) runWindow(w, i * 300000u, cur, 50, 40);
  runWindow(w, 5 * 300000u, cur, 5000, 4000);   // burst window
  EvalWindow::Result r = runWindow(w, 6 * 300000u, cur, 50, 40);
  EXPECT_EQ(480u, r.baseline_ros_rate);
}

TEST(EvalWindow, InsufficientWindowsDoNotFeedBaseline) {
  EvalWindow w;
  w.begin(cfg(40, 300000, 900000));
  w.open(0, stats(0, 0, 0, 0), 1);
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(900000, stats(0, 0, 10, 9), 0, 0, r));
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, HighUtilizationSetsGuardrailFlagButStaysMeasured) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < 4; i++) runWindow(w, i * 300000u, cur, 50, 40);
  w.open(4 * 300000u, cur, 9);
  cur.echo_attempt_count += 50;
  cur.echo_success_count += 40;
  EvalWindow::Result r{};
  ASSERT_TRUE(w.poll(5 * 300000u, cur, EvalWindow::GUARDRAIL_UTIL_PCT + 1, 30, r));
  EXPECT_TRUE(r.measured);
  EXPECT_TRUE(r.flags & EVALF_GUARDRAIL);
  EXPECT_EQ(EvalWindow::GUARDRAIL_UTIL_PCT + 1, r.util_pct);
  EXPECT_EQ(30u, r.cad_busy_pct);
}

TEST(EvalWindow, ResetBaselineClearsHistory) {
  EvalWindow w;
  w.begin(cfg(40, 300000));
  Stats cur = stats(0, 0, 0, 0);
  for (int i = 0; i < 5; i++) runWindow(w, i * 300000u, cur, 50, 40);
  w.resetBaseline();
  EXPECT_EQ(0u, w.baselineCount());
}

TEST(EvalWindow, ToRecordsFillsBothSlotsAndSaturates) {
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
  EvalRecordA a; EvalRecordB b;
  EvalWindow::toRecords(r, 42, EVAL_ROLLBACK, a, b);
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
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
