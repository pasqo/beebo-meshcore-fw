#include <gtest/gtest.h>
#include <helpers/Objective.h>

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

TEST(Objective, DefaultWeightsAreGoodput) {
  Objective o;
  EXPECT_NEAR(200.0 * 0.9, o.trialValue(win(200, 9000)), 1e-4);
  EXPECT_NEAR(50.0 * 0.5, o.trialValue(win(50, 5000)), 1e-4);
}

TEST(Objective, WeightIsAnExponentInTenths) {
  Objective o;
  o.weights[Objective::RX_VALID] = 5;   // sqrt
  EvalWindow::Result r = win(100, 10000);
  r.rx_valid = 16;   // 96 per hour
  EXPECT_NEAR(100.0 * 1.0 * sqrt(96.0), o.trialValue(r), 1e-3);
}

TEST(Objective, ZeroWeightIgnoresTheIndicator) {
  Objective o;
  EvalWindow::Result a = win(100), b = win(100);
  b.rx_valid = 50; b.rx_errors = 30; b.tx_dispatched = 9; b.reach_heard = 7; b.util_pct = 80;
  EXPECT_DOUBLE_EQ(o.trialValue(a), o.trialValue(b));
}

TEST(Objective, ZeroLevelIsFlooredNotInfinite) {
  Objective o;
  double dead = o.trialLog(win(0));   // routed == 0, either sign of weight
  EXPECT_TRUE(std::isfinite(dead));
  EXPECT_NEAR(log(Objective::FLOOR) + log(0.9), dead, 1e-4);
  o.weights[Objective::RX_ERRORS] = -10;
  EXPECT_TRUE(std::isfinite(o.trialLog(win(100))));   // rx_errors == 0, negative weight
  o.weights[Objective::RX_VALID] = 10;
  EXPECT_TRUE(std::isfinite(o.trialLog(win(100))));   // rx_valid == 0, positive weight
}

TEST(Objective, CostPercentagesPenalizeWithoutBlowingUp) {
  Objective o;
  o.weights[Objective::CAD_BUSY] = -10;
  EvalWindow::Result idle = win(100, 10000), busy = win(100, 10000);
  busy.cad_busy_pct = 50;   // level 1.5
  EXPECT_NEAR(100.0, o.trialValue(idle), 1e-6);
  EXPECT_NEAR(100.0 / 1.5, o.trialValue(busy), 1e-6);
}

TEST(Objective, BanditRewardDefaultsToGoodputScale) {
  Objective o;
  EvalWindow::Result r = win(100, 8000);
  r.ros_norm = 1500;   // 1.5x baseline
  EXPECT_NEAR(1500.0 * 8000.0 / 1000.0, o.banditReward(r), 1.0);
}

TEST(Objective, BanditRewardScoresExtraIndicatorsAgainstTheirOwnAverage) {
  Objective o;
  o.weights[Objective::RX_VALID] = 10;
  EvalWindow::Result r = win(100, 10000);
  r.rx_valid = 10;
  float first = o.banditReward(r);          // seeds the average: ratio 1
  EXPECT_NEAR(10000.0f, first, 1.0f);
  r.rx_valid = 20;                          // twice the seeded average
  float second = o.banditReward(r);
  EXPECT_NEAR(20000.0f, second, 5.0f);
  o.resetBaselines();
  EXPECT_NEAR(10000.0f, o.banditReward(r), 1.0f);   // reseeds
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
  EXPECT_NEAR(log(1.10) + 0.5 * log(1.21), d, 1e-3);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
