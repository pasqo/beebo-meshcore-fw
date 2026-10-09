#include <gtest/gtest.h>
#include <helpers/TrafficInjector.h>

namespace {

TEST(TrafficInjector, OffUntilChannelAndRateAreSet) {
  TrafficInjector t;
  EXPECT_FALSE(t.active());
  t.setRate(10);
  EXPECT_FALSE(t.active());
  t.setChannel(2, 0xAB);
  EXPECT_TRUE(t.active());
  t.setRate(0);
  EXPECT_FALSE(t.active());
  t.setRate(10);
  t.clearChannel();
  EXPECT_FALSE(t.active());
  EXPECT_EQ(TrafficInjector::NO_CHANNEL, t.channel());
}

TEST(TrafficInjector, GapIsExponentialWithMeanSixtyOverRate) {
  // u = 65536 / e^1 gives -ln(u) = 1: one mean gap.
  EXPECT_NEAR(6000, TrafficInjector::gapMs(10, 24109), 10);
  EXPECT_NEAR(3000, TrafficInjector::gapMs(20, 24109), 10);
}

TEST(TrafficInjector, GapIsClampedBetweenMinAndMax) {
  EXPECT_EQ(TrafficInjector::MIN_GAP_MS, TrafficInjector::gapMs(255, 65535));
  EXPECT_EQ(TrafficInjector::MAX_GAP_MS, TrafficInjector::gapMs(1, 1));
  EXPECT_EQ(TrafficInjector::MAX_GAP_MS, TrafficInjector::gapMs(0, 30000));
}

TEST(TrafficInjector, SendsOnlyWhileRateIsBelowTargetAndWindowFull) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  EXPECT_FALSE(t.shouldSend(10, true));   // at/above target
  EXPECT_FALSE(t.shouldSend(11, true));
  EXPECT_FALSE(t.shouldSend(3, false));   // window not full yet
}

TEST(TrafficInjector, SendDecisionBelowTarget) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  EXPECT_TRUE(t.shouldSend(4, true));
  t.setRate(0);
  EXPECT_FALSE(t.shouldSend(0, true));
}

TEST(TrafficInjector, CountsSendsAndResetsOnSet) {
  TrafficInjector t;
  t.setChannel(1, 7);
  EXPECT_EQ(1u, t.nextSeq());
  EXPECT_EQ(2u, t.nextSeq());
  EXPECT_EQ(2u, t.sent());
  t.setChannel(1, 7);
  EXPECT_EQ(0u, t.sent());
}

TEST(TrafficInjector, TextIsCounterPaddedToSize) {
  char buf[40];
  EXPECT_EQ(32, TrafficInjector::text(buf, 7, 32));
  EXPECT_STREQ("000007..........................", buf);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
