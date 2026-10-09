#include <gtest/gtest.h>
#include <helpers/RetryPolicy.h>

namespace {

TEST(RetryPolicy, PackAndUnpackTheCodingRates) {
  uint8_t p = 0;
  const uint8_t one[] = {8};
  ASSERT_TRUE(RetryPolicy::pack(one, 1, &p));
  EXPECT_EQ(112, p);
  EXPECT_EQ(1, RetryPolicy::count(p));
  EXPECT_EQ(8, RetryPolicy::crAt(p, 1));
  const uint8_t three[] = {5, 6, 7};
  ASSERT_TRUE(RetryPolicy::pack(three, 3, &p));
  EXPECT_EQ(3, RetryPolicy::count(p));
  EXPECT_EQ(5, RetryPolicy::crAt(p, 1));
  EXPECT_EQ(6, RetryPolicy::crAt(p, 2));
  EXPECT_EQ(7, RetryPolicy::crAt(p, 3));
  EXPECT_TRUE(RetryPolicy::valid(p));
}

TEST(RetryPolicy, NoSuchRetryHasNoCodingRate) {
  EXPECT_EQ(0, RetryPolicy::count(0));
  EXPECT_EQ(0, RetryPolicy::crAt(0, 1));
  EXPECT_EQ(0, RetryPolicy::crAt(112, 2));   // one retry only
  EXPECT_EQ(0, RetryPolicy::crAt(112, 0));
  EXPECT_EQ(0, RetryPolicy::crAt(0xFF, 4));
}

TEST(RetryPolicy, PackRejectsBadCountAndRate) {
  uint8_t p = 99;
  const uint8_t cr[] = {5, 6, 7, 8};
  EXPECT_FALSE(RetryPolicy::pack(cr, 0, &p));
  EXPECT_FALSE(RetryPolicy::pack(cr, 4, &p));
  const uint8_t low[] = {4}, high[] = {9};
  EXPECT_FALSE(RetryPolicy::pack(low, 1, &p));
  EXPECT_FALSE(RetryPolicy::pack(high, 1, &p));
  EXPECT_EQ(99, p);
}

TEST(RetryPolicy, EachPolicyHasOneCanonicalByte) {
  int valid = 0;
  for (int p = 0; p < 256; p++) valid += RetryPolicy::valid((uint8_t)p);
  EXPECT_EQ(1 + 4 + 16 + 64, valid);   // off, then 4^n policies of n retries
  EXPECT_FALSE(RetryPolicy::valid(1));       // none taken, yet a rate set
  EXPECT_FALSE(RetryPolicy::valid(0x41));    // one retry, a second rate set
  EXPECT_TRUE(RetryPolicy::valid(0xFF));
}

TEST(RetryPolicy, ParseStopsAtTheSeparator) {
  uint8_t p;
  const char* end;
  const char* text = "5|6,8";
  ASSERT_TRUE(RetryPolicy::parse(text, ',', &p, &end));
  EXPECT_EQ(2, RetryPolicy::count(p));
  EXPECT_EQ(',', *end);
  ASSERT_TRUE(RetryPolicy::parse(" 0 ", ',', &p, &end));
  EXPECT_EQ(0, p);
  EXPECT_EQ('\0', *end);
  EXPECT_FALSE(RetryPolicy::parse("5|", ',', &p, &end));
  EXPECT_FALSE(RetryPolicy::parse("", ',', &p, &end));
  EXPECT_FALSE(RetryPolicy::parse("5 6", ',', &p, &end));
}

TEST(RetryPolicy, FormatRoundTrips) {
  char buf[8];
  EXPECT_EQ(1, RetryPolicy::format(0, buf));
  EXPECT_STREQ("0", buf);
  EXPECT_EQ(3, RetryPolicy::format(140, buf));
  EXPECT_STREQ("5|8", buf);
  for (int p = 0; p < 256; p++) {
    if (!RetryPolicy::valid((uint8_t)p)) continue;
    RetryPolicy::format((uint8_t)p, buf);
    uint8_t q = 255;
    const char* end;
    ASSERT_TRUE(RetryPolicy::parse(buf, ',', &q, &end));
    EXPECT_EQ(p, q);
  }
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
