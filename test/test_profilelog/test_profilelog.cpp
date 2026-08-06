#include <gtest/gtest.h>
#include <vector>
#include <helpers/ProfileLog.h>

TEST(ProfileLog, EmptyRingReportsZeroCountAndTotal) {
  ProfileLog log;
  EXPECT_EQ(0u, log.count());
  uint8_t dest[64];
  uint16_t total = 123;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  EXPECT_EQ(0, n);
  EXPECT_EQ(0u, total);
}

TEST(ProfileLog, LogIncrementsCountUpToCapacity) {
  ProfileLog log;
  for (int i = 0; i < 5; i++) log.log((uint16_t)i, 100u);
  EXPECT_EQ(5u, log.count());
}

TEST(ProfileLog, LogWrapsAtMaxEventsWithoutGrowingCount) {
  ProfileLog log;
  for (int i = 0; i < PROFILE_MAX_EVENTS + 10; i++) log.log((uint16_t)i, 1u);
  EXPECT_EQ((uint16_t)PROFILE_MAX_EVENTS, log.count());
}

TEST(ProfileLog, DurationSaturatesAtUint16Max) {
  ProfileLog log;
  log.log(0x1234, 100000u);  // exceeds 0xFFFF

  uint8_t dest[8];
  uint16_t total = 0;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  ASSERT_EQ(8, n);
  ASSERT_EQ(1u, total);

  uint16_t id, duration;
  memcpy(&id, &dest[4], 2);
  memcpy(&duration, &dest[6], 2);
  EXPECT_EQ(0x1234, id);
  EXPECT_EQ(0xFFFF, duration);
}

TEST(ProfileLog, SerializeRoundTripsIdAndDuration) {
  ProfileLog log;
  log.log(0xBEEF, 42u);

  uint8_t dest[8];
  uint16_t total = 0;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  ASSERT_EQ(8, n);
  ASSERT_EQ(1u, total);

  uint16_t id, duration;
  memcpy(&id, &dest[4], 2);
  memcpy(&duration, &dest[6], 2);
  EXPECT_EQ(0xBEEF, id);
  EXPECT_EQ(42u, duration);
}

TEST(ProfileLog, SerializePaginatesAcrossMultipleCalls) {
  ProfileLog log;
  for (int i = 0; i < 20; i++) log.log((uint16_t)i, (uint32_t)i);

  const int per_event = 8;
  uint8_t page[per_event * 7];  // 7 events/page, uneven vs. 20 total
  uint16_t total = 0;
  uint16_t offset = 0;
  std::vector<uint16_t> ids_seen;

  while (true) {
    int n = log.serialize(page, sizeof(page), offset, &total);
    if (n == 0) break;
    int n_events = n / per_event;
    for (int j = 0; j < n_events; j++) {
      uint16_t id;
      memcpy(&id, &page[j * per_event + 4], 2);
      ids_seen.push_back(id);
    }
    offset += n_events;
    if (offset >= total) break;
  }

  ASSERT_EQ(20u, total);
  ASSERT_EQ(20u, ids_seen.size());
  for (uint16_t i = 0; i < 20; i++) EXPECT_EQ(i, ids_seen[i]);  // oldest-first order preserved
}

TEST(ProfileLog, ClearResetsCountAndSerializesEmpty) {
  ProfileLog log;
  for (int i = 0; i < 10; i++) log.log((uint16_t)i, 1u);
  ASSERT_EQ(10u, log.count());

  log.clear();
  EXPECT_EQ(0u, log.count());

  uint8_t dest[64];
  uint16_t total = 123;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  EXPECT_EQ(0, n);
  EXPECT_EQ(0u, total);
}

TEST(ProfileLog, LogAfterClearStartsFromScratch) {
  ProfileLog log;
  for (int i = 0; i < 10; i++) log.log((uint16_t)i, 1u);
  log.clear();
  log.log(0xABCD, 55u);

  EXPECT_EQ(1u, log.count());
  uint8_t dest[8];
  uint16_t total = 0;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  ASSERT_EQ(8, n);
  ASSERT_EQ(1u, total);
  uint16_t id, duration;
  memcpy(&id, &dest[4], 2);
  memcpy(&duration, &dest[6], 2);
  EXPECT_EQ(0xABCD, id);
  EXPECT_EQ(55u, duration);
}

TEST(ProfileLog, SerializeRespectsMaxLenBudget) {
  ProfileLog log;
  for (int i = 0; i < 10; i++) log.log((uint16_t)i, 1u);

  uint8_t dest[8 * 3];  // room for only 3 events
  uint16_t total = 0;
  int n = log.serialize(dest, sizeof(dest), 0, &total);
  EXPECT_EQ(8 * 3, n);
  EXPECT_EQ(10u, total);  // total still reflects the whole ring
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
