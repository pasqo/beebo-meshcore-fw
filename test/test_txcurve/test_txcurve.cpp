#include <gtest/gtest.h>
#include <BeeboTxCurve.h>

namespace {

TEST(TxCurve, SmaPowerFollowsTheMeasuredSweep) {
  EXPECT_EQ(33, beeboTxSmaX10(-9));
  EXPECT_EQ(223, beeboTxSmaX10(10));
  EXPECT_EQ(273, beeboTxSmaX10(17));
  EXPECT_EQ(273, beeboTxSmaX10(22));
}

TEST(TxCurve, OutOfRangeClampsToTheTable) {
  EXPECT_EQ(33, beeboTxSmaX10(-20));
  EXPECT_EQ(273, beeboTxSmaX10(30));
}

TEST(TxCurve, TheTableNeverFalls) {
  for (int tp = -9; tp < 22; tp++) EXPECT_LE(beeboTxSmaX10(tp), beeboTxSmaX10(tp + 1)) << tp;
}

TEST(TxCurve, AntennaPowerAddsTheExternalGain) {
  EXPECT_EQ(223, beeboTxAntX10(10, 0));
  EXPECT_EQ(303, beeboTxAntX10(10, 8));    // whole dB: the 5 V amplifier measured 30.6 dBm (+8.3 dB)
  EXPECT_EQ(113, beeboTxAntX10(3, -3));    // an external attenuator
}

TEST(TxCurve, TheDefaultLimitLeavesTheStockBoardAlone) {
  EXPECT_EQ(22, beeboTxMaxPower(22, 0, 30));   // the SMA plateaus at 27.3 dBm
  EXPECT_EQ(18, beeboTxMaxPower(18, 0, 30));
}

TEST(TxCurve, LimitPicksTheHighestPowerWithinIt) {
  // +8 dB external, 30 dBm: SMA <= 22.0 dBm -> txpower 9 (19.8); 10 gives 22.3 dBm
  EXPECT_EQ(9, beeboTxMaxPower(22, 8, 30));
  // no external gain, 27 dBm: txpower 16 gives 26.3, 17 gives 27.3
  EXPECT_EQ(16, beeboTxMaxPower(22, 0, 27));
}

TEST(TxCurve, LimitAtThePlateauAllowsEverything) {
  EXPECT_EQ(22, beeboTxMaxPower(22, 0, 28));
}

TEST(TxCurve, TheHardwareMaximumStillBinds) {
  EXPECT_EQ(12, beeboTxMaxPower(12, 0, 28));
}

TEST(TxCurve, ALimitBelowTheLowestPowerStillAllowsTheMinimum) {
  EXPECT_EQ(-9, beeboTxMaxPower(22, 20, 10));
}

TEST(TxCurve, TargetPicksTheHighestPowerNotAbove) {
  int tp = 99;
  ASSERT_TRUE(beeboTxPowerForAntX10(200, 0, 22, &tp));   // 20.0 dBm: 9 gives 19.8, 10 gives 22.3
  EXPECT_EQ(9, tp);
  ASSERT_TRUE(beeboTxPowerForAntX10(223, 0, 22, &tp));   // exactly txpower 10
  EXPECT_EQ(10, tp);
}

TEST(TxCurve, TargetAddsTheExternalGain) {
  int tp = 99;
  ASSERT_TRUE(beeboTxPowerForAntX10(300, 8, 22, &tp));   // 30.0 dBm behind +8 dB: SMA <= 22.0
  EXPECT_EQ(9, tp);
}

TEST(TxCurve, FlatStretchTakesTheLowestPowerWithTheSameEstimate) {
  int tp = 99;
  ASSERT_TRUE(beeboTxPowerForAntX10(300, 0, 22, &tp));   // above the plateau: 17..22 all read 27.3
  EXPECT_EQ(17, tp);
  ASSERT_TRUE(beeboTxPowerForAntX10(190, 0, 22, &tp));   // 6 and 7 both read 18.3
  EXPECT_EQ(6, tp);
}

TEST(TxCurve, TargetNeverExceedsTheHardwareMaximum) {
  int tp = 99;
  ASSERT_TRUE(beeboTxPowerForAntX10(300, 0, 12, &tp));
  EXPECT_EQ(12, tp);
}

TEST(TxCurve, TargetBelowTheMinimumIsRefused) {
  int tp = 99;
  EXPECT_FALSE(beeboTxPowerForAntX10(30, 0, 22, &tp));   // 3.0 dBm; the minimum gives 3.3
  EXPECT_EQ(99, tp);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
