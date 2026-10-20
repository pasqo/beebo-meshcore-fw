#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <FeedLines.h>

namespace {

TEST(FeedLines, MonLine) {
  char b[100];
  feedFmtMon(b, sizeof(b), true, 412, 37, true, 9812, 10000);
  EXPECT_STREQ(b, "mon on rx=412 tx=37 qos=98 soh=100");
}

TEST(FeedLines, MonLineWithoutExposureAndCaptureOff) {
  char b[100];
  feedFmtMon(b, sizeof(b), false, 0, 0, false, 0, 9950);
  EXPECT_STREQ(b, "mon off rx=0 tx=0 qos=- soh=99");
}

TEST(FeedLines, GainHasSignAndOneDecimal) {
  char b[16];
  feedFmtGain(b, sizeof(b), 12);   EXPECT_STREQ(b, "+1.2");
  feedFmtGain(b, sizeof(b), -4);   EXPECT_STREQ(b, "-0.4");
  feedFmtGain(b, sizeof(b), 0);    EXPECT_STREQ(b, "+0.0");
  feedFmtGain(b, sizeof(b), -1234); EXPECT_STREQ(b, "-123.4");
}

TEST(FeedLines, TrialStart) {
  char b[100];
  feedFmtTrialStart(b, sizeof(b), "lna", "main", 1, 0, 96, 1800);
  EXPECT_STREQ(b, "trial lna start profile=main a=1 b=0 blocks=96 block=1800s");
  feedFmtTrialStart(b, sizeof(b), "directtxdelay", "exhaustive", 255, 255, 1000, 3600);
  EXPECT_STREQ(b, "trial directtxdelay start profile=exhaustive a=255 b=255 blocks=1000 block=3600s");
}

TEST(FeedLines, TrialBlockWithoutBoundsThenWith) {
  char b[100];
  feedFmtTrialBlock(b, sizeof(b), "cr", 3, 96, 2, 25, false, 0, 0);
  EXPECT_STREQ(b, "trial cr blk 4/96 pairs=2 gain=+2.5%");
  feedFmtTrialBlock(b, sizeof(b), "cr", 7, 96, 4, -8, true, -31, 14);
  EXPECT_STREQ(b, "trial cr blk 8/96 pairs=4 gain=-0.8% [-3.1,+1.4]");
}

TEST(FeedLines, TrialEnd) {
  char b[100];
  feedFmtTrialEnd(b, sizeof(b), "rxboost", 2, 0, 1, 40, 37, 9);
  EXPECT_STREQ(b, "trial rxboost adopt_b a=0 b=1 pairs=40 gain=+3.7% se=+0.9%");
}

TEST(FeedLines, TrialSkip) {
  char b[100];
  feedFmtTrialSkip(b, sizeof(b), "cad", 4);
  EXPECT_STREQ(b, "trial cad skipped_same");
}

TEST(FeedLines, TrialWaitingForTraffic) {
  char b[100];
  feedFmtTrialWait(b, sizeof(b));
  EXPECT_STREQ(b, "trial waiting for traffic");
}

TEST(FeedLines, TxPowerWarnsBelowTheSaturationOnsetOnly) {
  char b[100] = "untouched";
  EXPECT_EQ(0, feedFmtTxPowerWarn(b, sizeof(b), 16));
  EXPECT_EQ(0, feedFmtTxPowerWarn(b, sizeof(b), 22));
  EXPECT_STREQ(b, "untouched");
  EXPECT_GT(feedFmtTxPowerWarn(b, sizeof(b), 10), 0);
  EXPECT_STREQ(b, "warn: txpower=10 dBm underdrives the PA, run: radio.txpower 18");
  feedFmtTxPowerWarn(b, sizeof(b), -9);
  EXPECT_STREQ(b, "warn: txpower=-9 dBm underdrives the PA, run: radio.txpower 18");
}

TEST(FeedLines, OutcomeNamesMatchTheEnumOrder) {
  EXPECT_STREQ(feedTrialOutcomeName(1), "keep_a");
  EXPECT_STREQ(feedTrialOutcomeName(12), "skipped_underpowered");
  EXPECT_STREQ(feedTrialOutcomeName(99), "outcome?");
}

TEST(FeedLines, LongestLinesFitTheMessageBudget) {
  char b[200];
  int n = feedFmtTrialEnd(b, sizeof(b), "directtxdelay", 12, 255, 255, 4294967295u, -2147483647, -2147483647);
  EXPECT_LT(n, 150);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
