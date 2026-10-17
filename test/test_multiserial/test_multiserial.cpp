#include <gtest/gtest.h>
#include <deque>
#include <vector>
#define MULTI_TRANSPORT
#define MULTI_TRANSPORT_MAX 3
#include "Arduino.h"
#include "helpers/MultiSerialInterface.h"

namespace {

// beebo: a scripted transport -- frames the test queues come out of
// checkRecvFrame() one per call; every FSM-visible hook is counted.
class FakeTransport : public BaseSerialInterface {
public:
  bool enabled = true;
  bool connected = true;
  bool drained = true;      // txDrained(): the last write has left the radio
  int resets = 0;           // resetParserState() -- also what drops a BLE/TCP link
  int enables = 0;
  std::deque<uint8_t> frames;   // one byte per frame, enough to tell them apart

  void enable() override { enabled = true; enables++; }
  void disable() override { enabled = false; }
  bool isEnabled() const override { return enabled; }
  bool isConnected() const override { return connected; }
  bool isWriteBusy() const override { return false; }
  bool txDrained() const override { return drained; }
  void resetParserState() override { resets++; }
  size_t writeFrame(const uint8_t[], size_t len) override { return len; }
  size_t checkRecvFrame(uint8_t dest[], size_t, RecvFrameType* type) override {
    if (frames.empty()) return 0;
    dest[0] = frames.front();
    frames.pop_front();
    if (type) *type = RecvFrameType::BINARY;
    return 1;
  }
};

class MultiSerial : public ::testing::Test {
protected:
  void SetUp() override {
    g_mock_millis = 1000;
    multi.addInterface(&ble, nullptr, nullptr, true, RLOG_ID_XPORT_BLE);
    multi.addInterface(&tcp, nullptr, nullptr, true, RLOG_ID_XPORT_TCP);
    multi.enable();
    tick();   // DISABLED -> IDLE
    ble.frames.push_back(0x01);   // APP_START from the BLE central
    tick();   // IDLE -> ACTIVE, BLE owns the session
  }

  size_t tick(RecvFrameType* type = nullptr) {
    return multi.checkRecvFrame(dest, sizeof(dest), type);
  }
  void advance(uint32_t ms) { g_mock_millis += ms; }

  FakeTransport ble, tcp;
  MultiSerialInterface multi;
  uint8_t dest[MAX_FRAME_SIZE];
};

TEST_F(MultiSerial, SetupLocksTheSessionOnTheTransportThatSentTheFirstFrame) {
  EXPECT_TRUE(multi.isConnected());
  EXPECT_FALSE(multi.isSessionIdle());
  EXPECT_FALSE(tcp.enabled);   // the other exclusive transport is shut down
}

TEST_F(MultiSerial, DisconnectOfADrainedTransportReleasesOnTheNextTick) {
  multi.disconnectActive();
  tick();
  EXPECT_TRUE(multi.isSessionIdle());
  EXPECT_EQ(1, ble.resets);
  EXPECT_TRUE(tcp.enabled);   // siblings come back with the release
}

TEST_F(MultiSerial, DisconnectWaitsWhileTheOwnerStillHoldsUnsentData) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  advance(50);
  tick();
  EXPECT_FALSE(multi.isSessionIdle());
  EXPECT_EQ(0, ble.resets);    // the link must not be dropped under the pending reply
  EXPECT_FALSE(tcp.enabled);
}

TEST_F(MultiSerial, DisconnectReleasesOnceTheOwnerHasDrained) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  advance(50);
  tick();
  ble.drained = true;
  tick();
  EXPECT_TRUE(multi.isSessionIdle());
  EXPECT_EQ(1, ble.resets);
  EXPECT_TRUE(tcp.enabled);
}

TEST_F(MultiSerial, DisconnectReleasesAfterTheDeadlineEvenIfNeverDrained) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  advance(MultiSerialInterface::CLOSING_TIMEOUT_MS - 1);
  tick();
  EXPECT_FALSE(multi.isSessionIdle());
  advance(2);
  tick();
  EXPECT_TRUE(multi.isSessionIdle());
  EXPECT_EQ(1, ble.resets);
}

TEST_F(MultiSerial, ClosingAcceptsNoNewCommandsFromTheOwner) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  ble.frames.push_back(0x42);   // trailing frame after APP_DISCONNECT
  EXPECT_EQ(0u, tick());
  ble.drained = true;
  EXPECT_EQ(0u, tick());
  EXPECT_TRUE(multi.isSessionIdle());
}

TEST_F(MultiSerial, ClosingStillReportsTheLinkAndAllowsTheReplyToBeWritten) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  uint8_t ok[] = {0};
  EXPECT_TRUE(multi.isConnected());
  EXPECT_EQ(1u, multi.writeFrame(ok, 1));
}

TEST_F(MultiSerial, ALostLinkWhileClosingReleasesWithoutWaiting) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  ble.connected = false;     // the central dropped the link itself
  tick();
  EXPECT_TRUE(multi.isSessionIdle());
}

TEST_F(MultiSerial, AForcedDisableStillWinsOverClosing) {
  ble.drained = false;
  multi.disconnectActive();
  tick();
  multi.disable();
  tick();
  EXPECT_FALSE(multi.isEnabled());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
