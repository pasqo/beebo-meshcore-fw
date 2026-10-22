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

// Trial probe mode (kbase/DYNAMIC_OPTIMIZER.md): the trial borrows the injector at a
// fixed rate and hop cap, and gives it back as the user left it.
TEST(TrafficInjectorProbe, SendsAtItsRateWhateverTheNodeReceives) {
  TrafficInjector t;
  ASSERT_TRUE(t.beginProbe(3, 0x5A, 6, 2));
  EXPECT_TRUE(t.probing());
  EXPECT_TRUE(t.active());
  EXPECT_TRUE(t.shouldSend(500, true));     // a busy channel does not hold a probe back
  EXPECT_TRUE(t.shouldSend(0, false));      // nor does a window still filling
  EXPECT_EQ(6, t.rate());
  EXPECT_EQ(2, t.hops());
  EXPECT_EQ(3, t.channel());
  EXPECT_EQ(0x5A, t.channelHash());
}

TEST(TrafficInjectorProbe, NeedsAChannelAndARate) {
  TrafficInjector t;
  EXPECT_FALSE(t.beginProbe(TrafficInjector::NO_CHANNEL, 0, 6, 2));
  EXPECT_FALSE(t.beginProbe(1, 7, 0, 2));
  EXPECT_FALSE(t.probing());
  EXPECT_FALSE(t.active());
}

TEST(TrafficInjectorProbe, EndRestoresWhatTheUserHadSet) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  t.nextSeq();
  t.nextSeq();
  ASSERT_TRUE(t.beginProbe(4, 9, 6, 2));
  t.nextSeq();
  EXPECT_EQ(1u, t.sent());                  // the probe's own count
  t.endProbe();
  EXPECT_FALSE(t.probing());
  EXPECT_EQ(1, t.channel());
  EXPECT_EQ(7, t.channelHash());
  EXPECT_EQ(10, t.rate());
  EXPECT_EQ(0, t.hops());
  EXPECT_EQ(2u, t.sent());
  EXPECT_FALSE(t.shouldSend(10, true));     // back to the RX-rate rule
}

TEST(TrafficInjectorProbe, EndWithNothingSetLeavesItOff) {
  TrafficInjector t;
  ASSERT_TRUE(t.beginProbe(1, 7, 6, 2));
  t.endProbe();
  EXPECT_FALSE(t.active());
  EXPECT_EQ(TrafficInjector::NO_CHANNEL, t.channel());
}

TEST(TrafficInjectorProbe, EndWithoutBeginChangesNothing) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  t.endProbe();
  EXPECT_EQ(1, t.channel());
  EXPECT_EQ(10, t.rate());
}

TEST(TrafficInjectorProbe, ASecondBeginKeepsTheFirstSavedState) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  ASSERT_TRUE(t.beginProbe(4, 9, 6, 2));
  ASSERT_TRUE(t.beginProbe(5, 8, 6, 3));    // a restart of the trial
  t.endProbe();
  EXPECT_EQ(1, t.channel());
  EXPECT_EQ(10, t.rate());
}

TEST(TrafficInjectorProbe, ClearingTheChannelStopsSendsButEndStillRestores) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  ASSERT_TRUE(t.beginProbe(4, 9, 6, 2));
  t.clearChannel();                         // the slot lost its channel
  EXPECT_FALSE(t.active());
  EXPECT_TRUE(t.probing());                 // still the trial's until it ends
  t.endProbe();
  EXPECT_EQ(1, t.channel());
}

TEST(TrafficInjectorProbe, OrdinaryInjectorHasNoHopCap) {
  TrafficInjector t;
  t.setChannel(1, 7);
  t.setRate(10);
  EXPECT_EQ(0, t.hops());
  EXPECT_FALSE(t.probing());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
