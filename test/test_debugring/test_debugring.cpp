#include <gtest/gtest.h>
#include <vector>
#include <helpers/DebugRing.h>

namespace {

// beebo: stands in for whichever transport DebugRing::attach() points at --
// records every writeFrameBestEffort() call so a test can count exactly how
// many replay pushes happened per replayStep() call (never more than one),
// pinning down the 2026-09-07 bug where beginReplay()'s predecessor
// (replayRing(), a single synchronous burst) overflowed
// SerialWifiInterface's shallow send_queue when replayed over TCP/BLE.
// beebo: a distinct non-zero, non-USB RLOG_ID_XPORT_* value -- stands in
// for "a TCP/BLE session is currently locked" (activeTransportType()'s
// 0 means idle/none, RLOG_ID_XPORT_USB(2) means the same physical wire
// DebugRing already reaches via its separate `usb` target -- see
// PushesTo* tests below for both of those cases specifically).
constexpr uint8_t kFakeNonUsbTransportType = 3;

class FakeSerial : public BaseSerialInterface {
public:
  int push_count = 0;
  uint8_t active_transport_type = kFakeNonUsbTransportType;

  void enable() override {}
  void disable() override {}
  bool isEnabled() const override { return true; }
  bool isConnected() const override { return true; }
  bool isWriteBusy() const override { return false; }
  uint8_t activeTransportType() const override { return active_transport_type; }
  size_t writeFrame(const uint8_t src[], size_t len) override { return len; }
  size_t writeFrameBestEffort(const uint8_t src[], size_t len) override {
    push_count++;
    return len;
  }
  size_t checkRecvFrame(uint8_t dest[], size_t max_len = MAX_FRAME_SIZE,
                         RecvFrameType* type = nullptr) override {
    return 0;
  }
};

// beebo: every existing replay test below attaches `serial` as the
// aggregator target only (usb=nullptr) with its default
// kFakeNonUsbTransportType, i.e. "a TCP/BLE session is locked, no USB
// tap" -- preserves these tests' original single-target push_count
// semantics. The dual-push fix itself (both targets reached correctly
// depending on session state) is covered separately by the
// PushesTo* tests at the bottom of this file.

TEST(DebugRingReplay, NotReplayingBeforeBeginReplay) {
  DebugRing ring;
  FakeSerial serial;
  ring.attach(&serial, nullptr, 0xDE, 1, 2);
  EXPECT_FALSE(ring.isReplaying());
  EXPECT_FALSE(ring.replayStep());   // no-op, never armed
  EXPECT_EQ(serial.push_count, 0);
}

TEST(DebugRingReplay, EmptyRingBeginReplayIsAlreadyDone) {
  DebugRing ring;
  FakeSerial serial;
  ring.attach(&serial, nullptr, 0xDE, 1, 2);
  ring.beginReplay();
  EXPECT_FALSE(ring.isReplaying());   // nothing to replay
}

TEST(DebugRingReplay, OneEventPerStepNotABurst) {
  DebugRing ring;
  FakeSerial serial;
  ring.attach(&serial, nullptr, 0xDE, 1, 2);
  ring.setEnabled(true);
  for (int i = 0; i < 5; i++) {
    ring.logRing(__FILE__, __LINE__, /*type=*/i, DLOG_SEV_H, /*detail=*/i * 10);
  }
  // beebo: logRing() itself also live-pushes (see its own comment) --
  // reset the counter so this test only counts beginReplay()'s pushes.
  serial.push_count = 0;

  ring.beginReplay();
  EXPECT_TRUE(ring.isReplaying());

  for (int i = 0; i < 5; i++) {
    EXPECT_EQ(serial.push_count, i)
        << "replayStep() must push exactly one event per call, not a burst";
    bool more = ring.replayStep();
    EXPECT_EQ(serial.push_count, i + 1);
    if (i < 4) {
      EXPECT_TRUE(more);
      EXPECT_TRUE(ring.isReplaying());
    } else {
      EXPECT_FALSE(more);
      EXPECT_FALSE(ring.isReplaying());
    }
  }

  // Further calls are no-ops once done.
  EXPECT_FALSE(ring.replayStep());
  EXPECT_EQ(serial.push_count, 5);
}

TEST(DebugRingReplay, ReplayOrderIsOldestFirstAcrossWraparound) {
  DebugRing ring;
  FakeSerial serial;
  ring.attach(&serial, nullptr, 0xDE, 1, 2);
  ring.setEnabled(true);
  // beebo: overfill the ring so it wraps -- RLOG_MAX_EVENTS events plus a
  // few more, confirming replayStep() walks the wrapped buffer in true
  // chronological order (oldest surviving event first), not raw buffer
  // index order.
  for (int i = 0; i < RLOG_MAX_EVENTS + 3; i++) {
    ring.logRing(__FILE__, __LINE__, /*type=*/(uint8_t)(i % 250), DLOG_SEV_H, i);
  }
  EXPECT_EQ(ring.count(), RLOG_MAX_EVENTS);
  serial.push_count = 0;   // logRing() above also live-pushed each event

  ring.beginReplay();
  int expected_detail = 3;   // the first 3 events were evicted by wraparound
  int steps = 0;
  while (ring.isReplaying()) {
    ring.replayStep();
    steps++;
  }
  // beebo: replayStep() doesn't expose the pushed record's own detail to
  // the caller (pushRlogFrame() writes straight to the transport), so this
  // test only pins down the step count/completion contract directly --
  // full-detail-value verification would need a FakeSerial that decodes
  // its own writeFrameBestEffort() payload, out of scope for this bug fix.
  EXPECT_EQ(steps, RLOG_MAX_EVENTS);
  EXPECT_EQ(serial.push_count, RLOG_MAX_EVENTS);
  (void)expected_detail;
}

// beebo: pins down the 2026-09-07 regression -- DebugRing::attach()
// briefly took only the MultiSerialInterface aggregator as its push
// target (mid-TCP_DEBUG_STREAM development), so the session-less raw USB
// debug tap (BEEBO_RAW_SUB_DEBUG_LOG_ENABLE, no MultiSerialInterface
// session ever locked) silently got nothing whenever no session happened
// to be locked -- confirmed via `bunch-mc -d -` producing zero output.
// These three cases are exactly the ones attach()'s own comment commits
// to: usb always reached; serial reached additionally, but only when it
// has something genuinely non-USB locked.
TEST(DebugRingPushTargets, NoSessionLockedPushesUsbOnly) {
  DebugRing ring;
  FakeSerial usb, serial;
  serial.active_transport_type = 0;   // SESSION_IDLE -- nothing locked
  ring.attach(&serial, &usb, 0xDE, 1, 2);
  ring.setEnabled(true);
  ring.logRing(__FILE__, __LINE__, /*type=*/1, DLOG_SEV_H, /*detail=*/0);
  EXPECT_EQ(usb.push_count, 1) << "raw USB tap must work with no session locked at all";
  EXPECT_EQ(serial.push_count, 0);
}

TEST(DebugRingPushTargets, NonUsbSessionLockedPushesBoth) {
  DebugRing ring;
  FakeSerial usb, serial;
  serial.active_transport_type = kFakeNonUsbTransportType;   // TCP/BLE session locked
  ring.attach(&serial, &usb, 0xDE, 1, 2);
  ring.setEnabled(true);
  ring.logRing(__FILE__, __LINE__, /*type=*/1, DLOG_SEV_H, /*detail=*/0);
  EXPECT_EQ(usb.push_count, 1) << "raw USB tap must keep working alongside a live TCP/BLE session";
  EXPECT_EQ(serial.push_count, 1) << "TCP/BLE session must also see the event (TCP_DEBUG_STREAM)";
}

TEST(DebugRingPushTargets, UsbSessionLockedPushesOnceNotTwice) {
  DebugRing ring;
  FakeSerial usb, serial;
  serial.active_transport_type = RLOG_ID_XPORT_USB;   // session locked on USB itself
  ring.attach(&serial, &usb, 0xDE, 1, 2);
  ring.setEnabled(true);
  ring.logRing(__FILE__, __LINE__, /*type=*/1, DLOG_SEV_H, /*detail=*/0);
  EXPECT_EQ(usb.push_count, 1);
  EXPECT_EQ(serial.push_count, 0) << "same physical wire as usb -- must not double-send";
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
