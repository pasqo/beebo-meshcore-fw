#include <gtest/gtest.h>
#include <helpers/BattTrend.h>

namespace {

constexpr uint8_t ADC_12BIT = 12;
constexpr uint16_t CHARGED_MV = BATT_FULL_MV_DEFAULT;  // 4200

struct Trend {
  uint16_t ref_mv;
  uint8_t state;

  explicit Trend(uint16_t seed_mv, uint8_t initial_state = BATT_STATE_INIT)
    : ref_mv(seed_mv), state(initial_state) {}

  uint8_t sample(uint16_t mv, uint8_t batt_present = BATT_PRESENT_YES,
                 uint16_t charged_mv = CHARGED_MV) {
    return classifyBattTrend(mv, ref_mv, state, batt_present, ADC_12BIT, charged_mv);
  }
};

TEST(BattTrend, InitEntersChargedAtOrAboveThreshold) {
  Trend t(4100);
  EXPECT_EQ(t.sample(4200), BATT_STATE_CHARGED);
}

TEST(BattTrend, InitEntersChargingOnRise) {
  Trend t(4000);
  EXPECT_EQ(t.sample(4050), BATT_STATE_CHARGING);
}

TEST(BattTrend, InitEntersDischargingOnFall) {
  Trend t(4000);
  EXPECT_EQ(t.sample(3950), BATT_STATE_DISCHARGING);
}

TEST(BattTrend, ChargedStaysChargedOnRippleAboveThreshold) {
  // A dip that's still above charged_mv is ripple near the top, not a real
  // discharge -- must stay CHARGED regardless of dir.
  Trend t(4205, BATT_STATE_CHARGED);
  EXPECT_EQ(t.sample(4201), BATT_STATE_CHARGED);
  EXPECT_EQ(t.sample(4205), BATT_STATE_CHARGED);
}

TEST(BattTrend, ChargedExitsImmediatelyOnSingleStepBelowThreshold) {
  Trend t(4205, BATT_STATE_CHARGED);
  EXPECT_EQ(t.sample(4163), BATT_STATE_DISCHARGING);
}

TEST(BattTrend, ChargedExitsOnSlowContinuousDeclinePastThreshold) {
  // Regression test for the real-hardware bug (BUGS.md 2026-08-23): a node
  // stuck reporting CHARGED for 22+ hours while idle voltage fell ~150mV
  // past charged_mv, sampled correctly and continuously the whole time.
  // A dir<0-gated exit (the old behavior) requires a fresh hysteresis-sized
  // edge relative to ref_mv, which is pinned at the entry peak and never
  // moves while CHARGED -- so once one dip fails to clear that stale-peak
  // delta, every dip after it that's still within the same margin does too,
  // even though every one of them is already below charged_mv. The fixed
  // absolute-threshold check must catch this the moment any sample first
  // drops to/below charged_mv, however small each individual step was.
  Trend t(4205, BATT_STATE_CHARGED);
  uint16_t steps[] = {4201, 4201, 4196, 4192, 4187, 4183, 4178, 4174,
                       4169, 4165, 4160, 4156, 4151, 4147, 4142, 4138};
  uint8_t last = BATT_STATE_CHARGED;
  for (uint16_t mv : steps) {
    last = t.sample(mv);
  }
  EXPECT_EQ(last, BATT_STATE_DISCHARGING);
}

TEST(BattTrend, ChargedRefMvPinnedAtEntryPeakUntilExit) {
  // Documents existing, unchanged behavior: unlike CHARGING/DISCHARGING
  // (which track a moving peak/trough every sample), CHARGED's ref_mv only
  // ever moves on the transition out to DISCHARGING -- a further rise while
  // still CHARGED doesn't rebase it.
  Trend t(4205, BATT_STATE_CHARGED);
  EXPECT_EQ(t.sample(4250), BATT_STATE_CHARGED);
  EXPECT_EQ(t.ref_mv, 4205);
}

TEST(BattTrend, ChargedExitsToPluggedWhenBattPresentGoesNo) {
  Trend t(4205, BATT_STATE_CHARGED);
  EXPECT_EQ(t.sample(4205, BATT_PRESENT_NO), BATT_STATE_PLUGGED);
}

TEST(BattTrend, PluggedReturnsToInitWhenBattPresentGoesYes) {
  Trend t(0, BATT_STATE_PLUGGED);
  EXPECT_EQ(t.sample(3800), BATT_STATE_INIT);
}

TEST(BattTrend, DischargingReversesToChargingOnRise) {
  Trend t(4000, BATT_STATE_DISCHARGING);
  EXPECT_EQ(t.sample(4050), BATT_STATE_CHARGING);
}

TEST(BattTrend, ChargingReachesChargedAtThreshold) {
  Trend t(4000, BATT_STATE_CHARGING);
  EXPECT_EQ(t.sample(4200), BATT_STATE_CHARGED);
}

TEST(BattTrend, RespectsCustomChargedMv) {
  Trend t(3990, BATT_STATE_INIT);
  EXPECT_EQ(t.sample(4000, BATT_PRESENT_YES, /*charged_mv=*/4000), BATT_STATE_CHARGED);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
