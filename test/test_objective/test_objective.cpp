#include <gtest/gtest.h>
#include <helpers/Objective.h>

namespace {
constexpr double C = Objective::C;
}

namespace {

EvalWindow::Result win(uint32_t ros_rate, uint16_t ratio = 9000) {
  EvalWindow::Result r = {};
  r.measured = true;
  r.ros_rate = ros_rate;
  r.confirm_ratio = ratio;
  r.window_ms = 600000;   // 10 min: per-hour rate = count * 6
  r.ros_norm = 1000;
  return r;
}

}  // namespace

TEST(Objective, TransformIsZeroAtZeroAndLogarithmicAboveC) {
  Objective o;
  o.weights[Objective::ROUTED] = 10;
  o.weights[Objective::CONFIRM] = 0;
  EXPECT_DOUBLE_EQ(0.0, o.trialLog(win(0)));                    // continuous, exactly 0
  // a level of 100 (one reference volume): ln(1 + 100 / C)
  EXPECT_NEAR(log1p(100.0 / C), o.trialLog(win((uint32_t)o.volumeReference)), 1e-3);
  // well above C, a 10% change is ~ln 1.1 (the weighted relative change)
  double d = o.trialLog(win(220)) - o.trialLog(win(200));
  EXPECT_NEAR(log(1.1), d, 0.01);
}

TEST(Objective, DefaultWeightsTrackGoodputsRelativeChange) {
  Objective o;
  double d = o.trialLog(win(220, 9900)) - o.trialLog(win(200, 9000));
  EXPECT_NEAR(log(1.1) + log(1.1), d, 0.02);
}

TEST(Objective, WeightIsAnExponentInTenths) {
  Objective o;
  o.weights[Objective::RX_VALID] = 5;   // sqrt
  EvalWindow::Result a = win(100, 10000), b = win(100, 10000);
  a.rx_valid = 100; b.rx_valid = 400;   // x4 -> +0.5 * ln 4
  EXPECT_NEAR(0.5 * log(4.0), o.trialLog(b) - o.trialLog(a), 0.02);
}

TEST(Objective, VolumeReferenceComesFromAirtime) {
  // 64 bytes at ~240 ms: 0.184 * 3600 / 0.24 = 2760 per hour
  EXPECT_NEAR(2760.0f, Objective::referenceFor(240), 1.0f);
  EXPECT_FLOAT_EQ(Objective::DEFAULT_VOLUME_REFERENCE, Objective::referenceFor(0));
  Objective o;
  o.weights[Objective::RX_VALID] = 10;
  EvalWindow::Result r = win(100); r.rx_valid = 50;
  o.banditReward(r);                       // seeds a baseline
  o.setVolumeReference(o.volumeReference * 1.005f);   // under 1%: ignored
  EXPECT_GT(o.banditReward(r), 0.0f);
  float before = o.volumeReference;
  o.setVolumeReference(before * 2.0f);     // a real change resets the baselines
  EXPECT_FLOAT_EQ(before * 2.0f, o.volumeReference);
}

TEST(Objective, ZeroWeightIgnoresTheIndicator) {
  Objective o;
  EvalWindow::Result a = win(100), b = win(100);
  b.rx_valid = 50; b.rx_errors = 30; b.tx_dispatched = 9; b.reach_heard = 7; b.util_pct = 80;
  EXPECT_DOUBLE_EQ(o.trialValue(a), o.trialValue(b));
}

TEST(Objective, ZeroLevelsAreFiniteWithAnyWeight) {
  Objective o;
  o.weights[Objective::RX_ERRORS] = -10;
  o.weights[Objective::RX_VALID] = 10;
  EXPECT_TRUE(std::isfinite(o.trialLog(win(0))));
  EXPECT_TRUE(std::isfinite(o.trialLog(win(100))));
}

TEST(Objective, CostPercentagesAreGentleNearZero) {
  Objective o;
  o.weights[Objective::CAD_BUSY] = -10;
  EvalWindow::Result idle = win(100, 10000), busy = win(100, 10000);
  busy.cad_busy_pct = 1;
  // 0% -> 1%: ln(1 + 1/C) = ln 11, not the 100x jump a hard floor gives
  EXPECT_NEAR(-log1p(1.0 / C), o.trialLog(busy) - o.trialLog(idle), 1e-6);
}

TEST(Objective, BanditRewardDefaultsToGoodputScale) {
  Objective o;
  EvalWindow::Result r = win(100, 8000);
  r.ros_norm = 1500;   // 1.5x baseline
  // close to ros_norm x ratio (150 x 80 = 12000); the transform is exact only far above C
  EXPECT_NEAR(12000.0, o.banditReward(r), 300.0);
}

TEST(Objective, BanditRewardScoresExtraIndicatorsAgainstTheirOwnAverage) {
  Objective o;
  o.weights[Objective::RX_VALID] = 10;
  EvalWindow::Result r = win(100, 10000);
  r.rx_valid = 10;
  float first = o.banditReward(r);          // seeds the average: ratio 1
  EXPECT_NEAR(10000.0f, first, 300.0f);
  r.rx_valid = 20;                          // twice the seeded average
  float second = o.banditReward(r);
  EXPECT_NEAR(20000.0f, second, 600.0f);
  o.resetBaselines();
  EXPECT_NEAR(10000.0f, o.banditReward(r), 300.0f);   // reseeds
}

TEST(Objective, NegativeBanditWeightWithZeroLevelIsFinite) {
  Objective o;
  o.weights[Objective::RX_ERRORS] = -10;
  EvalWindow::Result r = win(100, 10000);
  r.rx_errors = 0;
  float v = o.banditReward(r);
  EXPECT_TRUE(std::isfinite(v));
  EXPECT_GT(v, 0.0f);
}

TEST(Objective, ExtremeWeightsStayInTheLogDomain) {
  Objective o;
  for (int i = 0; i < Objective::NUM_INDICATORS; i++) o.weights[i] = 50;   // 8 indicators at 5.0
  EvalWindow::Result r = win(100000, 10000);
  r.rx_valid = 60000; r.rx_errors = 60000; r.tx_dispatched = 60000; r.reach_heard = 40;
  r.util_pct = 100; r.cad_busy_pct = 100;
  double j = o.trialLog(r);
  EXPECT_TRUE(std::isfinite(j));                 // a plain product would be ~1e150
  EXPECT_GT(j, 100.0);                           // beyond float range (ln 3e38 = 88)
  // the tuner's reward is capped, never inf or NaN
  EvalWindow::Result b = r;
  b.ros_norm = 3000;
  float reward = o.banditReward(b);
  EXPECT_TRUE(std::isfinite(reward));
  EXPECT_FLOAT_EQ(20000.0f, reward);
  // and the opposite extreme stays finite too
  Objective neg;
  for (int i = 0; i < Objective::NUM_INDICATORS; i++) neg.weights[i] = -50;
  EXPECT_TRUE(std::isfinite(neg.trialLog(r)));
  EXPECT_GE(neg.banditReward(b), 0.0f);
}

TEST(Objective, LogScoreOfAChangeIsTheWeightedRelativeChange) {
  Objective o;
  o.weights[Objective::RX_VALID] = 5;
  EvalWindow::Result a = win(100, 9000), b = win(110, 9000);
  a.rx_valid = 100; b.rx_valid = 121;   // +10% routed, +21% rx_valid
  double d = o.trialLog(b) - o.trialLog(a);
  EXPECT_NEAR(log(1.10) + 0.5 * log(1.21), d, 0.01);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
