#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <cstring>
#include <helpers/DebugLog.h>

namespace {

// beebo: stands in for whichever transport DebugLog::attach() points at --
// records every writeFrameBestEffort() call so a test can count exactly how
// many replay pushes happened per replayStep() call (never more than one),
// pinning down the 2026-09-07 bug where beginReplay()'s predecessor
// (replayRing(), a single synchronous burst) overflowed
// SerialWifiInterface's shallow send_queue when replayed over TCP/BLE.
// beebo: a distinct non-zero, non-USB RLOG_ID_XPORT_* value -- stands in
// for "a TCP/BLE session is currently locked" (activeTransportType()'s
// 0 means idle/none, RLOG_ID_XPORT_USB(2) means the same physical wire
// DebugLog already reaches via its separate `usb` target -- see
// PushesTo* tests below for both of those cases specifically).
constexpr uint8_t kFakeNonUsbTransportType = 3;

class FakeSerial : public BaseSerialInterface {
public:
  int push_count = 0;
  uint8_t active_transport_type = kFakeNonUsbTransportType;
  std::vector<uint8_t> last_frame;
  // beebo: simulates a full USB CDC TX FIFO -- every write reports 0 bytes
  // sent, forcing DebugLog::enqueueUsb()'s caller down its queue-full path.
  bool reject_writes = false;

  void enable() override {}
  void disable() override {}
  bool isEnabled() const override { return true; }
  bool isConnected() const override { return true; }
  bool isWriteBusy() const override { return false; }
  uint8_t activeTransportType() const override { return active_transport_type; }
  size_t writeFrame(const uint8_t src[], size_t len) override { return len; }
  size_t writeFrameBestEffort(const uint8_t src[], size_t len) override {
    push_count++;
    last_frame.assign(src, src + len);
    return reject_writes ? 0 : len;
  }
  size_t checkRecvFrame(uint8_t dest[], size_t max_len = MAX_FRAME_SIZE,
                         RecvFrameType* type = nullptr) override {
    return 0;
  }
};

// beebo: pins down the 2026-09-07 regression -- DebugLog::attach()
// briefly took only the MultiSerialInterface aggregator as its push
// target (mid-TCP_DEBUG_STREAM development), so the session-less raw USB
// debug tap (BEEBO_RAW_SUB_DBG_ENABLE, no MultiSerialInterface
// session ever locked) silently got nothing whenever no session happened
// to be locked -- confirmed via `bunch-mc -d -` producing zero output.
// These three cases are exactly the ones attach()'s own comment commits
// to: usb always reached; serial reached additionally, but only when it
// has something genuinely non-USB locked.
TEST(DebugLogPushTargets, NoSessionLockedPushesUsbOnly) {
  DebugLog ring;
  FakeSerial usb, serial;
  serial.active_transport_type = 0;   // SESSION_IDLE -- nothing locked
  ring.attach(&serial, &usb, 0xDE, 1);
  ring.setUsbEnabled(true);
  ring.setSessionEnabled(true);
  // beebo: logLink() (DLOG) via a real transport target -- the only push
  // DebugLog still does itself now that RLOGH/M go straight to the sink
  // (MON_DEBUG) with no transport push of their own (see DebugLog.h's own
  // top comment) and RLOGL doesn't exist anymore.
  ring.logLink(__FILE__, __LINE__, /*id=*/1, DLOG_SEV_H, "msg");
  EXPECT_EQ(usb.push_count, 1) << "raw USB tap must work with no session locked at all";
  EXPECT_EQ(serial.push_count, 0);
}

TEST(DebugLogPushTargets, NonUsbSessionLockedPushesBoth) {
  DebugLog ring;
  FakeSerial usb, serial;
  serial.active_transport_type = kFakeNonUsbTransportType;   // TCP/BLE session locked
  ring.attach(&serial, &usb, 0xDE, 1);
  ring.setUsbEnabled(true);
  ring.setSessionEnabled(true);
  ring.logLink(__FILE__, __LINE__, /*id=*/1, DLOG_SEV_H, "msg");
  EXPECT_EQ(usb.push_count, 1) << "raw USB tap must keep working alongside a live TCP/BLE session";
  EXPECT_EQ(serial.push_count, 1) << "TCP/BLE session must also see the event (TCP_DEBUG_STREAM)";
}

TEST(DebugLogPushTargets, UsbSessionLockedPushesOnceNotTwice) {
  DebugLog ring;
  FakeSerial usb, serial;
  serial.active_transport_type = RLOG_ID_XPORT_USB;   // session locked on USB itself
  ring.attach(&serial, &usb, 0xDE, 1);
  ring.setUsbEnabled(true);
  ring.setSessionEnabled(true);
  ring.logLink(__FILE__, __LINE__, /*id=*/1, DLOG_SEV_H, "msg");
  EXPECT_EQ(usb.push_count, 1);
  EXPECT_EQ(serial.push_count, 0) << "same physical wire as usb -- must not double-send";
}

// beebo: pins down pushToTargets()'s ordering fix -- a frame arriving
// while an earlier one is still queued (from a prior busy/rejected write)
// must itself queue behind it rather than writing straight through, or
// the stream would deliver frame2 before frame1 whenever the peripheral
// happened to free up between the two calls.
TEST(DebugLogPushTargets, QueuedFrameBlocksLaterDirectWritePreservingOrder) {
  DebugLog ring;
  FakeSerial usb;
  ring.attach(nullptr, &usb, 0xDE, 1);

  uint8_t frame1[3] = {0xAA, 0x01, 0x02};
  usb.reject_writes = true;
  ring.writeUsbQueued(frame1, sizeof(frame1));   // fails to send -- queued
  int attempts_before = usb.push_count;

  uint8_t frame2[3] = {0xBB, 0x03, 0x04};
  usb.reject_writes = false;   // peripheral is free again
  ring.writeUsbQueued(frame2, sizeof(frame2));
  EXPECT_EQ(attempts_before, usb.push_count)
      << "frame2 must not be written directly while frame1 is still queued";

  ring.retryUsbQueue();
  EXPECT_EQ(std::vector<uint8_t>(frame1, frame1 + 3), usb.last_frame);
  ring.retryUsbQueue();
  EXPECT_EQ(std::vector<uint8_t>(frame2, frame2 + 3), usb.last_frame);
}

// beebo: MLOG (plans/MLOG_LIVE_STREAM.md) -- pushMlogFrame() gated on its
// own _usb_mlog_enabled/_session_mlog_enabled flags, independent of DLOG/
// RLOG's _usb_enabled/_session_enabled, but routed through the same
// pushToTargets() targeting logic (attach()'s usb-always/session-if-
// non-usb-and-not-same-wire rules).
TEST(DebugLogMlog, NotSentWhenMlogDisabledEvenIfDlogEnabled) {
  DebugLog ring;
  FakeSerial usb, serial;
  ring.attach(&serial, &usb, 0xDE, 1, 3);
  ring.setUsbEnabled(true);
  ring.setSessionEnabled(true);
  // MLOG bits left off.
  MonRecord rec; memset(&rec, 0, sizeof(rec)); rec.kind = MON_RX;
  ring.pushMlogFrame(rec);
  EXPECT_EQ(usb.push_count, 0);
  EXPECT_EQ(serial.push_count, 0);
}

TEST(DebugLogMlog, UsbMlogEnabledPushesUsbOnly) {
  DebugLog ring;
  FakeSerial usb, serial;
  serial.active_transport_type = 0;   // no session locked
  ring.attach(&serial, &usb, 0xDE, 1, 3);
  ring.setUsbMlogEnabled(true);
  MonRecord rec; memset(&rec, 0, sizeof(rec)); rec.kind = MON_TX;
  ring.pushMlogFrame(rec);
  EXPECT_EQ(usb.push_count, 1);
  EXPECT_EQ(serial.push_count, 0);
}

TEST(DebugLogMlog, SessionMlogEnabledPushesSessionOnly) {
  DebugLog ring;
  FakeSerial usb, serial;
  serial.active_transport_type = kFakeNonUsbTransportType;
  ring.attach(&serial, &usb, 0xDE, 1, 3);
  ring.setSessionMlogEnabled(true);
  MonRecord rec; memset(&rec, 0, sizeof(rec)); rec.kind = MON_RADIO;
  ring.pushMlogFrame(rec);
  EXPECT_EQ(usb.push_count, 0);
  EXPECT_EQ(serial.push_count, 1);
}

TEST(DebugLogMlog, FrameCarriesRecordVerbatim) {
  DebugLog ring;
  FakeSerial usb, serial;
  ring.attach(&serial, &usb, 0xDE, 1, 3);
  ring.setUsbMlogEnabled(true);
  MonRecord rec; memset(&rec, 0, sizeof(rec)); rec.kind = MON_RX;
  rec.rx.pkt_hash = 0xCAFEBABE;
  ring.pushMlogFrame(rec);
  ASSERT_EQ(usb.last_frame.size(), 2 + sizeof(MonRecord));
  EXPECT_EQ(usb.last_frame[0], 0xDE);   // resp_code
  EXPECT_EQ(usb.last_frame[1], 3);      // mlog_sub_id
  MonRecord decoded;
  memcpy(&decoded, &usb.last_frame[2], sizeof(MonRecord));
  EXPECT_EQ(decoded.kind, MON_RX);
  EXPECT_EQ(decoded.rx.pkt_hash, 0xCAFEBABEu);
}

// beebo: DebugSink forwarding (plans/MONITORING_UNIFICATION.md Design #3/#6)
// -- logRing() must call the sink for every H/M event. RLOGL doesn't exist
// (every former call site converted to DLOGL, since MON_DEBUG can't
// represent Low severity), so there's no "never for L" case left to test.
namespace {
struct SinkCall {
  uint8_t type; uint8_t severity; int32_t detail; uint32_t ms; std::string file; int line;
  uint8_t user[5];
};
std::vector<SinkCall> g_sink_calls;
void captureDebugSink(uint8_t type, uint8_t severity, int32_t detail, uint32_t ms,
                       const char* file, int line, const uint8_t user[5]) {
  SinkCall call{type, severity, detail, ms, file, line, {}};
  memcpy(call.user, user, sizeof(call.user));
  g_sink_calls.push_back(call);
}
}  // namespace

TEST(DebugLogSink, FiresForHighAndMediumSeverity) {
  g_sink_calls.clear();
  DebugLog ring;
  ring.setDebugSink(&captureDebugSink);
  int line1 = __LINE__ + 1;
  ring.logRing(__FILE__, __LINE__, 5, DLOG_SEV_H, 0x1234);
  int line2 = __LINE__ + 1;
  ring.logRing(__FILE__, __LINE__, 6, DLOG_SEV_M, 0x5678);
  ASSERT_EQ(2u, g_sink_calls.size());
  EXPECT_EQ(5, g_sink_calls[0].type);
  EXPECT_EQ(DLOG_SEV_H, g_sink_calls[0].severity);
  EXPECT_EQ(0x1234, g_sink_calls[0].detail);
  EXPECT_EQ(__FILE__, g_sink_calls[0].file);
  EXPECT_EQ(line1, g_sink_calls[0].line);
  EXPECT_EQ(6, g_sink_calls[1].type);
  EXPECT_EQ(DLOG_SEV_M, g_sink_calls[1].severity);
  EXPECT_EQ(line2, g_sink_calls[1].line);
}

TEST(DebugLogSink, NoOpWhenNoSinkSet) {
  DebugLog ring;   // no setDebugSink() call -- must not crash calling through nullptr
  ring.logRing(__FILE__, __LINE__, 5, DLOG_SEV_H, 0);
}

// beebo: pins down RLOG_ID_USB_QUEUE_DROP's own reentrancy guard
// (DebugLog.h's _reporting_usb_queue_drop) -- the sink here calls back
// into writeUsbQueued() the same way MonRing's real forwarding chain does
// (logRing() -> _debug_sink -> MonRing::appendDebug() -> _store() ->
// _live_sink() -> pushMlogFrame() -> pushToTargets() -> enqueueUsb()), so
// without the guard this would recurse without bound instead of the
// single extra (silently swallowed) reentrant attempt per real drop this
// test expects.
DebugLog* g_reentrant_ring = nullptr;
int g_reentrant_sink_calls = 0;
void reentrantDebugSink(uint8_t type, uint8_t, int32_t, uint32_t,
                         const char*, int, const uint8_t[5]) {
  g_reentrant_sink_calls++;
  ASSERT_EQ(RLOG_ID_USB_QUEUE_DROP, type);
  uint8_t frame[3] = {0xBE, 0x01, 0x02};
  g_reentrant_ring->writeUsbQueued(frame, sizeof(frame));   // simulates the real forwarding chain
}

TEST(DebugLogUsbQueueDrop, QueueFullReportsOnceNotRecursively) {
  DebugLog ring;
  FakeSerial usb;
  ring.attach(nullptr, &usb, 0xDE, 1);
  usb.reject_writes = true;   // every write fails to send -- forces enqueueUsb() every time
  g_reentrant_ring = &ring;
  g_reentrant_sink_calls = 0;
  ring.setDebugSink(&reentrantDebugSink);

  uint8_t frame[3] = {0xBE, 0x01, 0x02};
  // First 4 calls (USB_QUEUE_CAP) fill the retry queue without dropping.
  for (int i = 0; i < 4; i++) ring.writeUsbQueued(frame, sizeof(frame));
  EXPECT_EQ(0, g_reentrant_sink_calls);

  // The queue is now full: this call (and the next) each report exactly
  // once, not twice -- the reentrant call the sink itself makes must be
  // swallowed by the guard, or this test would hang/crash on unbounded
  // recursion before ever reaching these assertions.
  ring.writeUsbQueued(frame, sizeof(frame));
  EXPECT_EQ(1, g_reentrant_sink_calls);
  ring.writeUsbQueued(frame, sizeof(frame));
  EXPECT_EQ(2, g_reentrant_sink_calls);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
