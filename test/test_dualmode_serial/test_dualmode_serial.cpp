#include <gtest/gtest.h>
#include <string.h>
#include "helpers/DualModeSerialInterface.h"
#include "helpers/DebugLog.h"
#include "Stream.h"
#include "Arduino.h"

namespace {

class DualModeSerial : public ::testing::Test {
protected:
  void SetUp() override {
    g_mock_millis = 1000;
    iface.begin(stream);
    iface.enable();
  }

  size_t poll(RecvFrameType& type) {
    return iface.checkRecvFrame(dest, sizeof(dest), &type);
  }

  FakeStream stream;
  DualModeSerialInterface iface;
  uint8_t dest[512];
};

TEST_F(DualModeSerial, BinaryFrameRoundTrips) {
  uint8_t frame[] = {'<', 2, 0, 0xAA, 0xBB};
  stream.feed(frame, sizeof(frame));

  RecvFrameType type = RecvFrameType::TEXT;
  size_t len = poll(type);

  EXPECT_EQ(2u, len);
  EXPECT_EQ(RecvFrameType::BINARY, type);
  EXPECT_EQ(0xAA, dest[0]);
  EXPECT_EQ(0xBB, dest[1]);
}

TEST_F(DualModeSerial, TextLineRoundTrips) {
  const char* line = "help\r";
  stream.feed((const uint8_t*)line, strlen(line));

  RecvFrameType type = RecvFrameType::BINARY;
  size_t len = poll(type);

  EXPECT_EQ(RecvFrameType::TEXT, type);
  ASSERT_EQ(4u, len);
  EXPECT_EQ(0, memcmp(dest, "help", 4));
}

// beebo: the actual bug this replaces -- a RAW_MARKER byte must be
// recognized and consumed by checkRecvFrame() itself, in the same read, not
// require a separate pre-check call that can race real asynchronous byte
// arrival. Feeding the marker alone (nothing else buffered yet) must not be
// misrouted into MODE_TEXT.
TEST_F(DualModeSerial, LoneRawMarkerIsNotMisroutedIntoText) {
  stream.feed(DualModeSerialInterface::RAW_MARKER);

  RecvFrameType type = RecvFrameType::BINARY;
  size_t len = poll(type);

  EXPECT_EQ(0u, len);   // still waiting on sub_id/data -- not a text line
  EXPECT_TRUE(stream.tx().empty());   // and never echoed as a text byte either
}

TEST_F(DualModeSerial, RawControlFrameCompletesAcrossMultipleCalls) {
  stream.feed(DualModeSerialInterface::RAW_MARKER);
  RecvFrameType type;
  EXPECT_EQ(0u, poll(type));

  stream.feed(BEEBO_RAW_SUB_DBG_ENABLE);
  EXPECT_EQ(0u, poll(type));   // sub_id alone -- still waiting on data

  stream.feed(1);   // data
  size_t len = poll(type);

  EXPECT_EQ(2u, len);
  EXPECT_EQ(RecvFrameType::DEBUG, type);
  EXPECT_EQ(BEEBO_RAW_SUB_DBG_ENABLE, dest[0]);
  EXPECT_EQ(1, dest[1]);
}

TEST_F(DualModeSerial, TimeSyncRawFrameCountsAsLivenessButEnableAloneDoesNot) {
  uint8_t enable_frame[] = {DualModeSerialInterface::RAW_MARKER, BEEBO_RAW_SUB_DBG_ENABLE, 1};
  stream.feed(enable_frame, sizeof(enable_frame));
  RecvFrameType type;
  poll(type);
  EXPECT_FALSE(iface.isConnected());   // debug-log-enable alone is not liveness

  uint8_t time_sync_frame[] = {
    DualModeSerialInterface::RAW_MARKER, BEEBO_RAW_SUB_TIME_SYNC, 0, 0, 0, 0,
  };
  stream.feed(time_sync_frame, sizeof(time_sync_frame));
  poll(type);
  EXPECT_TRUE(iface.isConnected());   // time-sync is the one sub-frame that counts
}

TEST_F(DualModeSerial, TimeSyncRawFrameCarriesFourByteEpochAndCountsAsLiveness) {
  EXPECT_FALSE(iface.isConnected());

  uint8_t time_sync_frame[] = {
    DualModeSerialInterface::RAW_MARKER, BEEBO_RAW_SUB_TIME_SYNC, 0x78, 0x56, 0x34, 0x12,
  };
  RecvFrameType type;
  for (size_t i = 0; i + 1 < sizeof(time_sync_frame); i++) {
    stream.feed(time_sync_frame[i]);
    EXPECT_EQ(0u, poll(type));   // payload still incomplete
  }
  stream.feed(time_sync_frame[sizeof(time_sync_frame) - 1]);
  size_t len = poll(type);

  EXPECT_EQ(5u, len);
  EXPECT_EQ(RecvFrameType::DEBUG, type);
  EXPECT_EQ(BEEBO_RAW_SUB_TIME_SYNC, dest[0]);
  EXPECT_EQ(0x78, dest[1]);
  EXPECT_EQ(0x56, dest[2]);
  EXPECT_EQ(0x34, dest[3]);
  EXPECT_EQ(0x12, dest[4]);
  EXPECT_TRUE(iface.isConnected());   // time-sync also counts as liveness
}

TEST_F(DualModeSerial, StuckRawFrameSelfHealsAfterResyncTimeout) {
  stream.feed(DualModeSerialInterface::RAW_MARKER);
  RecvFrameType type;
  EXPECT_EQ(0u, poll(type));   // parked in MODE_RAW_SUB, waiting on sub_id

  g_mock_millis += 3001;   // past RESYNC_TIMEOUT_MS with nothing more arriving
  EXPECT_EQ(0u, poll(type));   // resyncs back to MODE_IDLE, not stuck forever

  // A real command right after must parse cleanly, proving the parser
  // actually recovered rather than staying wedged mid-raw-frame.
  uint8_t frame[] = {'<', 1, 0, 0x42};
  stream.feed(frame, sizeof(frame));
  size_t len = poll(type);
  EXPECT_EQ(1u, len);
  EXPECT_EQ(RecvFrameType::BINARY, type);
  EXPECT_EQ(0x42, dest[0]);
}

TEST_F(DualModeSerial, BinaryFrameAfterRawFrameInSameBufferIsNotCorrupted) {
  // beebo: the exact scenario the original bug corrupted -- a raw control
  // frame immediately followed by a real command, all already buffered by
  // the time checkRecvFrame() runs. One call only ever completes one frame
  // (state machine invariant), so this drains in two polls, never merging
  // or misrouting bytes between them.
  uint8_t combined[] = {
    DualModeSerialInterface::RAW_MARKER, BEEBO_RAW_SUB_DBG_ENABLE, 0,
    '<', 1, 0, 0x99,
  };
  stream.feed(combined, sizeof(combined));

  RecvFrameType type;
  size_t len1 = poll(type);
  EXPECT_EQ(2u, len1);
  EXPECT_EQ(RecvFrameType::DEBUG, type);

  size_t len2 = poll(type);
  EXPECT_EQ(1u, len2);
  EXPECT_EQ(RecvFrameType::BINARY, type);
  EXPECT_EQ(0x99, dest[0]);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
