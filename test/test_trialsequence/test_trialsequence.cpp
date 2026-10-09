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
  // receive path first (the FEM LNA, the bigger lever, before RX boost), then
  // channel-busy behavior, coding rate, timing knobs
  EXPECT_EQ(TrialSequence::LNA, TrialSequence::orderAt(0));
  EXPECT_EQ(TrialSequence::RX_BOOST, TrialSequence::orderAt(1));
  EXPECT_EQ(TrialSequence::AGC, TrialSequence::orderAt(2));
  EXPECT_EQ(TrialSequence::INTERFERENCE, TrialSequence::orderAt(3));
  EXPECT_EQ(TrialSequence::CAD, TrialSequence::orderAt(4));   // channel-busy behavior
  EXPECT_EQ(TrialSequence::CR, TrialSequence::orderAt(5));
  EXPECT_EQ(TrialSequence::MULTI_ACKS, TrialSequence::orderAt(6));   // costs airtime, after the link setting
  EXPECT_EQ(TrialSequence::ECHO_AT_TX, TrialSequence::orderAt(7));   // when the echo window starts, before the retries that use it
  EXPECT_EQ(TrialSequence::RETRY, TrialSequence::orderAt(8));        // the retry policy
  EXPECT_EQ(TrialSequence::RX_DELAY, TrialSequence::orderAt(9));
  EXPECT_EQ(TrialSequence::AIRTIME, TrialSequence::orderAt(12));
  EXPECT_EQ(13, TrialSequence::NUM_SWITCHES);
  EXPECT_EQ(12, TrialSequence::ECHO_AT_TX);
  EXPECT_EQ(9, TrialSequence::CAD);          // wire bits of the newest switches
  EXPECT_EQ(10, TrialSequence::MULTI_ACKS);
  EXPECT_EQ(11, TrialSequence::RETRY);
}

TEST(TrialSequence, NamesResolveToSwitches) {
  const char* rest = nullptr;
  EXPECT_EQ(TrialSequence::LNA, TrialSequence::byName("lna.values 0,1", &rest));
  EXPECT_STREQ("values 0,1", rest);
  EXPECT_EQ(TrialSequence::DIRECT_TX_DELAY, TrialSequence::byName("directtxdelay.enable", &rest));
  EXPECT_STREQ("enable", rest);
  EXPECT_EQ(TrialSequence::TX_DELAY, TrialSequence::byName("txdelay.values", &rest));
  EXPECT_EQ(TrialSequence::CAD, TrialSequence::byName("cad.enable", &rest));
  EXPECT_EQ(TrialSequence::MULTI_ACKS, TrialSequence::byName("multiacks.values", &rest));
  EXPECT_EQ(TrialSequence::RETRY, TrialSequence::byName("retry.values", &rest));
  EXPECT_EQ(TrialSequence::ECHO_AT_TX, TrialSequence::byName("echo_at_tx.enable", &rest));
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
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::CAD, 1));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::CAD, 2));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::MULTI_ACKS, 1));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::MULTI_ACKS, 2));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::ECHO_AT_TX, 1));
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::ECHO_AT_TX, 2));
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::RETRY, 0));     // no retries
  EXPECT_TRUE(TrialSequence::validValue(TrialSequence::RETRY, 112));   // one retry at 4/8
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::RETRY, 1));    // bits of a retry not taken
  EXPECT_FALSE(TrialSequence::validValue(TrialSequence::RETRY, 5));
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
  l.reset(TrialSequence::RX_BOOST);     EXPECT_EQ(1, l.first());   // boosted, the Heltec V4 default
  EXPECT_EQ(0, l.v[1]);
  l.reset(TrialSequence::CR);           EXPECT_EQ(5, l.first());   // then 6, 7, 8
  EXPECT_EQ(4, l.n);
  EXPECT_EQ(6, l.v[1]); EXPECT_EQ(7, l.v[2]); EXPECT_EQ(8, l.v[3]);
  l.reset(TrialSequence::AGC);          EXPECT_EQ(0, l.first());   // off
  EXPECT_EQ(4, l.n);
  l.reset(TrialSequence::INTERFERENCE); EXPECT_EQ(0, l.first());   // disabled
  l.reset(TrialSequence::RX_DELAY);     EXPECT_EQ(0, l.first());   // off
  l.reset(TrialSequence::TX_DELAY);     EXPECT_EQ(50, l.first());  // 0.5
  l.reset(TrialSequence::DIRECT_TX_DELAY); EXPECT_EQ(30, l.first());   // 0.3
  l.reset(TrialSequence::AIRTIME);      EXPECT_EQ(20, l.first());  // 1.0
  EXPECT_EQ(10, l.v[1]);   // 0.5: the airtime list has no unlimited (0) entry
  l.reset(TrialSequence::CAD);          EXPECT_EQ(0, l.first());   // off, then on
  EXPECT_EQ(2, l.n); EXPECT_EQ(1, l.v[1]);
  l.reset(TrialSequence::MULTI_ACKS);   EXPECT_EQ(0, l.first());   // off, then on
  EXPECT_EQ(2, l.n); EXPECT_EQ(1, l.v[1]);
  l.reset(TrialSequence::RETRY);        EXPECT_EQ(0, l.first());   // off, then "8", then "5|8"
  EXPECT_EQ(3, l.n); EXPECT_EQ(112, l.v[1]); EXPECT_EQ(140, l.v[2]);
  l.reset(TrialSequence::ECHO_AT_TX);   EXPECT_EQ(0, l.first());   // at scheduling, then at the send
  EXPECT_EQ(2, l.n); EXPECT_EQ(1, l.v[1]);
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

TEST(TrialSequence, FromStoredEveryValueIsAChallenger) {
  TrialSequence::List l;
  l.reset(TrialSequence::AGC);   // 0, 2, 4, 8
  l.fromStored();
  EXPECT_EQ(0, l.current());     // the first value too: A is what is stored, not v[0]
  int challengers = 1;
  while (l.advance()) challengers++;
  EXPECT_EQ(4, challengers);
  l.restart();
  EXPECT_EQ(2, l.current());
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
  EXPECT_EQ(0x08070605u, l.packed());   // 5, 6, 7, 8
}

TEST(TrialSequence, ParseAndFormatRoundTrip) {
  uint8_t out[TrialSequence::MAX_VALUES];
  const int sw = TrialSequence::AGC;
  EXPECT_EQ(3, TrialSequence::parse(sw, "8,16, 32", out));
  EXPECT_EQ(8, out[0]);
  EXPECT_EQ(16, out[1]);
  EXPECT_EQ(32, out[2]);
  EXPECT_EQ(1, TrialSequence::parse(sw, " 7 ", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "8,", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "8;9", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "256", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "1,2,3,4,5", out));
  TrialSequence::List l;
  const uint8_t v[] = {5, 6, 7, 8};
  ASSERT_TRUE(l.set(TrialSequence::CR, v, 4));
  char buf[8 * TrialSequence::MAX_VALUES];
  EXPECT_EQ(7, l.format(TrialSequence::CR, buf));
  EXPECT_STREQ("5,6,7,8", buf);
  l.n = 2; l.v[0] = 255; l.v[1] = 0;
  l.format(sw, buf);
  EXPECT_STREQ("255,0", buf);
}

TEST(TrialSequence, RetryListsSeparateRatesWithBar) {
  uint8_t out[TrialSequence::MAX_VALUES];
  const int sw = TrialSequence::RETRY;
  ASSERT_EQ(3, TrialSequence::parse(sw, "0,8,5|8", out));
  EXPECT_EQ(0, out[0]); EXPECT_EQ(112, out[1]); EXPECT_EQ(140, out[2]);
  ASSERT_EQ(2, TrialSequence::parse(sw, " 5 | 6 , 7|8|5 ", out));
  EXPECT_EQ(0x80 | 0x00 | 0x04, out[0]);
  EXPECT_EQ(0xC0 | 0x20 | 0x0C | 0x00, out[1]);
  EXPECT_EQ(-1, TrialSequence::parse(sw, "", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "8,", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "4", out));          // a coding rate is 5..8
  EXPECT_EQ(-1, TrialSequence::parse(sw, "5|6|7|8", out));    // at most three retries
  EXPECT_EQ(-1, TrialSequence::parse(sw, "0|5", out));
  EXPECT_EQ(-1, TrialSequence::parse(sw, "0,5,6,7,8", out));  // at most four policies
  TrialSequence::List l;
  l.reset(sw);
  char buf[8 * TrialSequence::MAX_VALUES];
  l.format(sw, buf);
  EXPECT_STREQ("0,8,5|8", buf);
}

// participants(): who takes part in a combined trial. Side A is the stored value
// and side B the list's current challenger (see Beebo::loopTuning).
struct Part {
  TrialSequence::Lists lists;
  uint8_t stored[TrialSequence::NUM_SWITCHES] = {};
  Part() {
    for (int i = 0; i < TrialSequence::NUM_SWITCHES; i++) {
      lists.v[i].fromStored();
      stored[i] = lists.v[i].first();
    }
  }
  uint16_t run(uint16_t pending, bool fem_lna = true) const {
    return TrialSequence::participants(lists, stored, pending, fem_lna);
  }
};
static uint16_t bit(int sw) { return (uint16_t)(1u << sw); }

TEST(TrialSequence, ParticipantsNeedASideBThatDiffers) {
  Part p;
  uint8_t v[2] = {0, 1};
  ASSERT_TRUE(p.lists.v[TrialSequence::CAD].set(TrialSequence::CAD, v, 2));
  p.lists.v[TrialSequence::CAD].restart();   // B = 1
  p.stored[TrialSequence::CAD] = 1;
  EXPECT_EQ(0, p.run(bit(TrialSequence::CAD)));   // B equals A
  p.stored[TrialSequence::CAD] = 0;
  EXPECT_EQ(bit(TrialSequence::CAD), p.run(bit(TrialSequence::CAD)));
}

TEST(TrialSequence, ParticipantsLeaveOutSwitchesNotPending) {
  Part p;
  uint8_t v[2] = {0, 1};
  ASSERT_TRUE(p.lists.v[TrialSequence::CAD].set(TrialSequence::CAD, v, 2));
  p.lists.v[TrialSequence::CAD].restart();   // pos 1: B = 1, A = stored 0
  p.stored[TrialSequence::CAD] = 0;
  EXPECT_EQ(bit(TrialSequence::CAD), p.run(bit(TrialSequence::CAD)));
  EXPECT_EQ(0, p.run(0));                                   // disabled or done
  EXPECT_EQ(0, p.run(bit(TrialSequence::MULTI_ACKS)));      // another switch pending
}

TEST(TrialSequence, ParticipantsSkipLnaWithoutFemControl) {
  Part p;
  p.lists.v[TrialSequence::LNA].restart();   // B = 1
  p.stored[TrialSequence::LNA] = 0;
  EXPECT_EQ(bit(TrialSequence::LNA), p.run(bit(TrialSequence::LNA), true));
  EXPECT_EQ(0, p.run(bit(TrialSequence::LNA), false));
}

TEST(TrialSequence, ParticipantCount) {
  EXPECT_EQ(0, TrialSequence::countBits(0));
  EXPECT_EQ(3, TrialSequence::countBits(bit(0) | bit(5) | bit(12)));
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
