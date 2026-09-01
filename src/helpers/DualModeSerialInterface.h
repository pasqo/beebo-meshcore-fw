#pragma once

#include "BaseSerialInterface.h"
#include <Arduino.h>

// beebo: USB CLI that lets the classic raw-text repeater CLI (`meshcli -r`:
// plain "cmd\r" lines, per-byte echo, "  -> reply\n" response) keep working
// over the same port that other clients (config.meshcore.io, OTA-over-serial)
// use for the framed binary companion protocol (`<len>`/`>len>` envelope,
// same wire format as ArduinoSerialInterface).
//
// Text vs binary is decided fresh for every single command, from its first
// byte: '<' starts a binary frame (the same marker ArduinoSerialInterface/
// SerialWifiInterface already use for every binary command on every other
// transport -- never legitimate as the first byte of a text command), any
// other byte starts a text line. No persistent mode, no handshake: nothing
// carries over between commands, so there's no stale cross-connection state
// to reset and no dependency on correctly detecting a session boundary.
class DualModeSerialInterface : public BaseSerialInterface {
  enum { MODE_IDLE, MODE_TEXT, MODE_FRAMED_LEN1, MODE_FRAMED_LEN2, MODE_FRAMED_BODY };

  bool _isEnabled;
  uint8_t _state;
  bool _lastWasText;
  uint16_t _frame_len;
  uint16_t rx_len;
  uint32_t _last_byte_at;
  Stream* _serial;
  // beebo: sized for raw-binary OTA frames (OTA_FRAME_SIZE = OTA_CHUNK_SIZE +
  // 2-byte header), same as SerialWifiInterface's getMaxRecvFrameSize()
  // override below -- USB is at least as fast as WiFi, no reason
  // OTA-over-serial should be stuck at MAX_FRAME_SIZE chunks while WiFi gets
  // 23x fewer round trips.
  uint8_t rx_buf[OTA_FRAME_SIZE];

  // beebo: a stray leading byte (port-open noise, a reset artifact) can
  // latch a partial command with no rest of it ever arriving, swallowing
  // every subsequent byte forever. Self-resync to MODE_IDLE if a partial
  // command goes quiet. Generous, since a real client (e.g. a browser Web
  // Serial writer) may split a frame's header/body across separate writes
  // with real USB-stack latency.
  static const uint32_t RESYNC_TIMEOUT_MS = 3000;

  bool feedTextByte(int c, uint8_t dest[], size_t max_len, size_t& outLen);

public:
  // beebo: fixed-format, session-less control byte -- [RAW_MARKER][sub_id]
  // [data] -- recognized here, below and independent of both the framed-
  // command/text-line state machine above and MultiSerialInterface's
  // session arbitration entirely. Never '<' (0x3C, a real framed command)
  // or '>' (0x3E, this transport's own reply marker), and never a byte any
  // legitimate Serial.println() debug line would emit (those are always
  // printable ASCII + CR/LF) -- see pollRawControl()'s own comment for why
  // this exists and where it's called from.
  static const uint8_t RAW_MARKER = 0x01;

  DualModeSerialInterface() { _isEnabled = false; _state = MODE_IDLE; _lastWasText = false; _last_byte_at = 0; }

  void begin(Stream& serial) {
    _serial = &serial;
  #ifdef RAK_4631
    pinMode(WB_IO2, OUTPUT);
  #endif
  }

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }
  void discardStaleRx() override;

  bool isConnected() const override;

  // beebo: called directly by Beebo::checkSerialInterface(), before
  // _serial->checkRecvFrame() (the MultiSerialInterface aggregator) runs at
  // all -- not through it, and not through this class's own checkRecvFrame()
  // either. That matters for two reasons: (1) MultiSerialInterface only
  // polls USB's checkRecvFrame() while USB is idle-polled or already the
  // locked/active transport (MultiSerialInterface.h's own checkRecvFrame()
  // comment) -- while a different transport (BLE/TCP) holds the session,
  // USB's checkRecvFrame() is never invoked at all, so a raw control byte
  // routed through it would simply never be seen; (2) even when it would be
  // invoked, going through checkRecvFrame() means MultiSerialInterface's
  // lockOn() fires and pins `_active` to USB for as long as the raw
  // command's issuer keeps polling it, locking out every BLE/TCP connection
  // attempt for that whole time -- exactly the side effect this bypasses.
  // Only ever consumes bytes when _state == MODE_IDLE (no framed/text parse
  // already in flight -- if one is, this defers entirely to the normal
  // parser) and only once the full fixed-size frame has already arrived; a
  // lone stray RAW_MARKER byte with nothing following it yet is left
  // untouched (peeked, not consumed) rather than partially eaten, so it's
  // still there to retry on the next call. Returns true and fills
  // sub_id/data once a full raw frame was consumed.
  bool pollRawControl(uint8_t& sub_id, uint8_t& data);

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t writeFrameBestEffort(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override;
  size_t getMaxRecvFrameSize() const override { return OTA_FRAME_SIZE; }
  // beebo: USB is at least as fast as WiFi (see rx_buf's own comment on the
  // recv side) -- no reason its outbound cap should stay stuck at the BLE
  // MTU floor while SerialWifiInterface::getMaxSendFrameSize() already
  // advertises MAX_SEND_FRAME_SIZE. Matters for GET_PREFS_TLV's paginated
  // dump (Beebo.cpp/BeeboRepeater.cpp's encodePrefsTlv): without this
  // override the whole table's worth of fields needed several USB round
  // trips it didn't actually need, unlike WiFi's single-page dump.
  size_t getMaxSendFrameSize() const override { return MAX_SEND_FRAME_SIZE; }
  bool lastRecvWasText() const override { return _lastWasText; }
};
