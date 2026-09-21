#include <gtest/gtest.h>
#include <helpers/NeighbourReach.h>

namespace {

// Same field names as Beebo.h's NeighbourInfo, without its Arduino deps.
struct Nb {
  uint32_t heard_timestamp;
  int8_t snr;       // x4
  uint16_t rx_count;
  uint8_t win_count;
};

}  // namespace

TEST(NeighbourReach, CountsOnlyEntriesHeardEnoughTimesInTheBlock) {
  Nb t[4] = {
    {100, 20, 5, 1},    // once: a one-off decode, not reach
    {100, 20, 9, 2},
    {100, 20, 9, 7},
    {0, 20, 0, 9},      // empty slot
  };
  uint8_t heard = 0, marginal = 0;
  NeighbourReach::scan(t, 4, heard, marginal);
  EXPECT_EQ(2, heard);
  EXPECT_EQ(0, marginal);
}

TEST(NeighbourReach, MarginalIsHeardEntriesBelowTheSnrThreshold) {
  Nb t[3] = {
    {100, NeighbourReach::MARGINAL_SNR_X4 - 1, 5, 3},   // marginal
    {100, NeighbourReach::MARGINAL_SNR_X4, 5, 3},       // at threshold: not marginal
    {100, 40, 5, 3},
  };
  uint8_t heard = 0, marginal = 0;
  NeighbourReach::scan(t, 3, heard, marginal);
  EXPECT_EQ(3, heard);
  EXPECT_EQ(1, marginal);
}

TEST(NeighbourReach, MarginalIgnoresEntriesNotHeardEnough) {
  Nb t[1] = {{100, -80, 5, 1}};
  uint8_t heard = 9, marginal = 9;
  NeighbourReach::scan(t, 1, heard, marginal);
  EXPECT_EQ(0, heard);
  EXPECT_EQ(0, marginal);
}

TEST(NeighbourReach, SaturatesAtTwoFiftyFive) {
  Nb t[300];
  for (int i = 0; i < 300; i++) t[i] = {100, 40, 5, 3};
  uint8_t heard = 0, marginal = 0;
  NeighbourReach::scan(t, 300, heard, marginal);
  EXPECT_EQ(255, heard);
}

TEST(NeighbourReach, BumpIncrementsBothCountersAndSaturates) {
  Nb n = {100, 0, 65534, 254};
  NeighbourReach::bump(n);
  EXPECT_EQ(65535, n.rx_count);
  EXPECT_EQ(255, n.win_count);
  NeighbourReach::bump(n);
  EXPECT_EQ(65535, n.rx_count);
  EXPECT_EQ(255, n.win_count);
}

TEST(NeighbourReach, ResetWindowZeroesOnlyTheBlockCounters) {
  Nb t[2] = {{100, 0, 40, 6}, {100, 0, 12, 3}};
  NeighbourReach::resetWindow(t, 2);
  EXPECT_EQ(0, t[0].win_count);
  EXPECT_EQ(0, t[1].win_count);
  EXPECT_EQ(40, t[0].rx_count);   // lifetime counts survive
  EXPECT_EQ(12, t[1].rx_count);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
