#include <gtest/gtest.h>
#include <helpers/TrialSequence.h>

namespace {

TEST(TrialSequence, NextSwitchFollowsTheRunOrderNotTheBitOrder) {
  EXPECT_EQ(-1, TrialSequence::nextSwitch(0));
  for (int i = 0; i < TrialSequence::NUM_SWITCHES; i++) {
    EXPECT_EQ(TrialSequence::orderAt(i), TrialSequence::nextSwitch(1u << TrialSequence::orderAt(i)));
  }
  uint16_t all = (1u << TrialSequence::NUM_SWITCHES) - 1;
  EXPECT_EQ(TrialSequence::orderAt(0), TrialSequence::nextSwitch(all));
  // the first switch of the order is done: the second is next
  all &= ~(1u << TrialSequence::orderAt(0));
  EXPECT_EQ(TrialSequence::orderAt(1), TrialSequence::nextSwitch(all));
}

TEST(TrialSequence, RunOrderCoversEverySwitchOnce) {
  bool seen[TrialSequence::NUM_SWITCHES] = {};
  for (int i = 0; i < TrialSequence::NUM_SWITCHES; i++) {
    ASSERT_LT(TrialSequence::orderAt(i), TrialSequence::NUM_SWITCHES);
    EXPECT_FALSE(seen[TrialSequence::orderAt(i)]);
    seen[TrialSequence::orderAt(i)] = true;
  }
  // receive path first, then channel-busy behavior, coding rate, timing knobs
  EXPECT_EQ(TrialSequence::LNA, TrialSequence::orderAt(0));
  EXPECT_EQ(TrialSequence::AGC, TrialSequence::orderAt(2));
  EXPECT_EQ(TrialSequence::CR, TrialSequence::orderAt(4));
  EXPECT_EQ(TrialSequence::AIRTIME, TrialSequence::orderAt(8));
}

TEST(TrialSequence, NamesResolveToSwitches) {
  const char* rest = nullptr;
  EXPECT_EQ(TrialSequence::LNA, TrialSequence::byName("lna.values 0,1", &rest));
  EXPECT_STREQ("values 0,1", rest);
  EXPECT_EQ(TrialSequence::DIRECT_TX_DELAY, TrialSequence::byName("directtxdelay.enable", &rest));
  EXPECT_STREQ("enable", rest);
  EXPECT_EQ(TrialSequence::TX_DELAY, TrialSequence::byName("txdelay.values", &rest));
  EXPECT_EQ(-1, TrialSequence::byName("lnax.values", &rest));
  EXPECT_EQ(-1, TrialSequence::byName("lna", &rest));   // no dot
  EXPECT_EQ(-1, TrialSequence::byName("nothing.values", &rest));
  for (int i = 0; i < TrialSequence::NUM_SWITCHES; i++) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s.enable", TrialSequence::name(i));
    EXPECT_EQ(i, TrialSequence::byName(buf, &rest));
  }
}

TEST(TrialSequence, ValueRangesPerSwitch) {
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::LNA, 1));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::LNA, 2));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::RX_BOOST, 2));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::CR, 5));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::CR, 8));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::CR, 4));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::CR, 9));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::AGC, 255));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::INTERFERENCE, 9));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::INTERFERENCE, 10));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::RX_DELAY, 200));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::RX_DELAY, 201));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::TX_DELAY, 200));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::DIRECT_TX_DELAY, 201));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::AIRTIME, 180));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::AIRTIME, 181));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::NUM_SWITCHES, 0));
}

TEST(TrialSequence, EveryDefaultListIsValidAndDistinctNeighbors) {
  for (int sw = 0; sw < TrialSequence::NUM_SWITCHES; sw++) {
    TrialSequence::List l;
    l.reset(sw);
    ASSERT_GE(l.n, 2);
    ASSERT_LE((int)l.n, (int)TrialSequence::MAX_VALUES);
    for (int i = 0; i < l.n; i++) {
      EXPECT_TRUE(TrialSequence::validValue(sw, l.v[i])) << TrialSequence::name(sw) << " " << i;
      if (i) EXPECT_NE(l.v[i - 1], l.v[i]) << TrialSequence::name(sw);
    }
    TrialSequence::List copy = l;
    EXPECT_TRUE(copy.set(sw, l.v, l.n));   // the default survives its own validation
    EXPECT_EQ(l.packed(), copy.packed());
  }
}

TEST(TrialSequence, DefaultListsStartAtTheUpstreamDefaults) {
  TrialSequence::List l;
  l.reset(TrialSequence::LNA);          EXPECT_EQ(0, l.first());
  l.reset(TrialSequence::CR);           EXPECT_EQ(5, l.first());
  l.reset(TrialSequence::AGC);          EXPECT_EQ(0, l.first());   // off
  EXPECT_EQ(4, l.n);
  l.reset(TrialSequence::INTERFERENCE); EXPECT_EQ(0, l.first());   // disabled
  l.reset(TrialSequence::RX_DELAY);     EXPECT_EQ(0, l.first());   // off
  l.reset(TrialSequence::TX_DELAY);     EXPECT_EQ(50, l.first());  // 0.5
  l.reset(TrialSequence::DIRECT_TX_DELAY); EXPECT_EQ(30, l.first());   // 0.3
  l.reset(TrialSequence::AIRTIME);      EXPECT_EQ(20, l.first());  // 1.0
  EXPECT_EQ(10, l.v[1]);   // 0.5: the airtime list has no unlimited (0) entry
}

TEST(TrialSequence, SetRejectsTooShortTooLongAndOutOfRange) {
  TrialSequence::List l;
  l.reset(TrialSequence::CR);
  const uint8_t ok[] = {5, 6, 7, 8};
  EXPECT_TRUE(l.set(TrialSequence::CR, ok, 4));
  EXPECT_EQ(4, l.n);
  const uint8_t too_many[] = {5, 6, 7, 8, 5};
  EXPECT_FALSE(l.set(TrialSequence::CR, too_many, 5));
  const uint8_t bad[] = {5, 9};
  EXPECT_FALSE(l.set(TrialSequence::CR, bad, 2));
  EXPECT_FALSE(l.set(TrialSequence::CR, ok, 0));
  const uint8_t single[] = {6};
  EXPECT_FALSE(l.set(TrialSequence::CR, single, 1));   // A alone has nothing to compare
  const uint8_t same[] = {6, 6};
  EXPECT_FALSE(l.set(TrialSequence::CR, same, 2));     // collapses to one value
  EXPECT_EQ(4, l.n);   // a rejected set leaves the list as it was
}

TEST(TrialSequence, ListOfNValuesIsNMinusOneTrials) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  EXPECT_EQ(5, l.first());
  EXPECT_EQ(6, l.current());     // trial 1: 5 against 6
  EXPECT_TRUE(l.advance());
  EXPECT_EQ(7, l.current());     // trial 2: the winner against 7
  EXPECT_FALSE(l.advance());     // last challenger decided
  l.restart();
  EXPECT_EQ(6, l.current());
}

TEST(TrialSequence, ConsecutiveRepeatsCollapse) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 5, 6, 5};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 4));
  EXPECT_EQ(3, l.n);   // 5,6,5
  EXPECT_EQ(5u | (6u << 8) | (5u << 16) | (5u << 24), l.packed());
}

TEST(TrialSequence, SetRestartsAtTheFirstTrial) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  l.advance();
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  EXPECT_EQ(6, l.current());
}

TEST(TrialSequence, PackedValuesForTheWireAndTheSettingEvent) {
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 3));
  EXPECT_EQ(5u | (6u << 8) | (7u << 16) | (7u << 24), l.packed());   // padded with the last value
  l.reset(TrialSequence::CR);
  EXPECT_EQ(0x08080805u, l.packed());
}

TEST(TrialSequence, ParseAndFormatRoundTrip) {
  uint8_t out[TrialSequence::MAX_VALUES];
  EXPECT_EQ(3, TrialSequence::parse("8,16, 32", out));
  EXPECT_EQ(8, out[0]);
  EXPECT_EQ(16, out[1]);
  EXPECT_EQ(32, out[2]);
  EXPECT_EQ(1, TrialSequence::parse(" 7 ", out));
  EXPECT_EQ(-1, TrialSequence::parse("", out));
  EXPECT_EQ(-1, TrialSequence::parse("8,", out));
  EXPECT_EQ(-1, TrialSequence::parse("8;9", out));
  EXPECT_EQ(-1, TrialSequence::parse("256", out));
  EXPECT_EQ(-1, TrialSequence::parse("1,2,3,4,5", out));
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7, 8};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 4));
  char buf[4 * TrialSequence::MAX_VALUES];
  EXPECT_EQ(7, l.format(buf));
  EXPECT_STREQ("5,6,7,8", buf);
  l.n = 2; l.v[0] = 255; l.v[1] = 0;
  l.format(buf);
  EXPECT_STREQ("255,0", buf);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
