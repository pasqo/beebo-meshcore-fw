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
  // beebo: MODE_RAW_SUB/MODE_RAW_DATA recognize and consume the session-less
  // raw control frame ([RAW_MARKER][sub_id][data], see RAW_MARKER below)
  // inline, as two more states of this same parser -- entered from
  // MODE_IDLE the instant RAW_MARKER is read, exactly like '<' enters
  // MODE_FRAMED_LEN1. This used to be a separate mechanism entirely
  // (pollRawControl()/hasPendingRawMarker(), called directly on this class
  // by Beebo::checkSerialInterface() *before* checkRecvFrame() ever ran, to
  // avoid checkRecvFrame() stealing a marker byte it had no way to
  // recognize) -- that split required two independent peeks of the same
  // live stream (one in hasPendingRawMarker(), one in checkRecvFrame()'s own
  // read), and real hardware bytes arriving in the gap between those two
  // reads (this is a byte-stream transport with real, asynchronous
  // hardware latency, not an atomic buffer snapshot) caused exactly the
  // corruption this replaces: a marker byte that hadn't arrived yet at
  // hasPendingRawMarker()'s check landing by the time checkRecvFrame() did
  // its own separate read a few instructions later, getting mis-routed into
  // MODE_TEXT as if it were an ordinary command byte (confirmed via a live
  // BEEBO_USB_RXTX_TRACE capture, 2026-09-06). Folding recognition into this
  // one state machine's one read removes the second peek entirely -- there
  // is no longer a second, later, independent read of the same byte for a
  // race to open up in.
  enum { MODE_IDLE, MODE_TEXT, MODE_FRAMED_LEN1, MODE_FRAMED_LEN2, MODE_FRAMED_BODY, MODE_RAW_SUB, MODE_RAW_DATA };

  bool _isEnabled;
  uint8_t _state;
  // beebo: internal only -- writeFrame()/writeFrameBestEffort()'s own
  // reply-format dispatch (text line vs binary <len> envelope), decided by
  // the last command actually consumed. Distinct from the public `type`
  // out-param callers read from checkRecvFrame() itself.
  bool _lastWasText;
  uint16_t _frame_len;
  uint16_t rx_len;
  uint32_t _last_byte_at;
  // beebo: sub_id read in MODE_RAW_SUB, held until MODE_RAW_DATA completes
  // the frame. Only meaningful while _state is one of the two raw states.
  uint8_t _raw_sub_id;
  // beebo: separate from _last_byte_at -- a raw control frame's own marker/
  // sub_id bytes deliberately do NOT refresh _last_byte_at/_seen_traffic
  // (see isConnected()'s own comment: only BEEBO_RAW_SUB_KEEPALIVE, once the
  // full frame is known, counts as link liveness), but the RESYNC_TIMEOUT_MS
  // self-heal below still needs its own clock so a stray marker with
  // nothing following doesn't park this parser in MODE_RAW_SUB forever --
  // set once, on entering MODE_RAW_SUB, read only while _state is a raw
  // state.
  uint32_t _raw_frame_started_at;
  // beebo: isConnected() gate -- false until a real byte actually arrives
  // on the link (see the framed/text parser's and checkRecvFrame()'s own
  // MODE_RAW_DATA-completion _last_byte_at writes), so a freshly enable()'d
  // transport reads as not connected until then. Reset false in enable() --
  // a toggle-off/on cycle starts the link-liveness question over, same as
  // boot does. Link-level only -- this class has no notion of a session
  // (MultiSerialInterface's own concept, layered on top).
  bool _seen_traffic;
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

  // beebo: how long with no byte at all (any kind -- framed command, text
  // line, or the session-less raw control frame) before isConnected()
  // infers the far side is gone, not just quiet -- see isConnected()'s own
  // comment. Generous relative to CommandHandler's own request/reply
  // timeouts (connect.py's _CONNECT_HANDSHAKE_TIMEOUT_S=3.0/library
  // default 15.0s) so a slow-but-alive exchange never trips it on its
  // own; the CLI's periodic keepalive (BEEBO_RAW_SUB_KEEPALIVE, sent well
  // under this interval during an idle `beebo -i` prompt) is what keeps a
  // genuinely idle-but-alive session under it indefinitely.
  static const uint32_t USB_IDLE_TIMEOUT_MS = 30000;

  bool feedTextByte(int c, uint8_t dest[], size_t max_len, size_t& outLen);

public:
  // beebo: fixed-format, session-less control byte -- [RAW_MARKER][sub_id]
  // [data] -- recognized here, below and independent of both the framed-
  // command/text-line state machine above and MultiSerialInterface's
  // session arbitration entirely. Never '<' (0x3C, a real framed command)
  // or '>' (0x3E, this transport's own reply marker), and never a byte any
  // legitimate Serial.println() debug line would emit (those are always
  // printable ASCII + CR/LF) -- see checkRecvFrame()'s own MODE_RAW_SUB/
  // MODE_RAW_DATA handling in the .cpp for where this is consumed.
  static const uint8_t RAW_MARKER = 0x01;

  DualModeSerialInterface() {
    _isEnabled = false; _state = MODE_IDLE; _lastWasText = false; _last_byte_at = 0; _seen_traffic = false;
    _raw_sub_id = 0; _raw_frame_started_at = 0;
  }

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
  // beebo: MultiSerialInterface::release() calls this on the released
  // transport specifically to reset a byte-parser left mid-command when a
  // session ends abruptly (see release()'s own comment) -- BaseSerialInterface's
  // default is a no-op, which left this class's _state/rx_len/_frame_len
  // never actually reset that way, so a session torn down while _state was
  // MODE_TEXT/MODE_FRAMED_* (e.g. a companion handshake the client gave up
  // on without ever sending CMD_APP_DISCONNECT, so this transport never
  // even calls release() on its own) carried that same stale state into
  // the next connection attempt on this transport, corrupting it from the
  // very first byte.
  void resetParserState() override;

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t writeFrameBestEffort(const uint8_t src[], size_t len) override;
  // beebo: also recognizes and consumes the session-less raw control frame
  // ([RAW_MARKER][sub_id][data], see RAW_MARKER above and this class's own
  // enum comment) inline as two extra parser states -- returns 2 with
  // *type == RecvFrameType::DEBUG and dest[0]/dest[1] holding sub_id/data
  // when one completes. Works regardless of MultiSerialInterface's session
  // arbitration (called on this object the same as any other frame; see
  // MultiSerialInterface::checkRecvFrame()'s own handling of
  // RecvFrameType::DEBUG for why a raw frame never wins/evicts a session).
  size_t checkRecvFrame(uint8_t dest[], size_t max_len, RecvFrameType* type) override;
  size_t getMaxRecvFrameSize() const override { return OTA_FRAME_SIZE; }
  // beebo: USB is at least as fast as WiFi (see rx_buf's own comment on the
  // recv side) -- no reason its outbound cap should stay stuck at the BLE
  // MTU floor while SerialWifiInterface::getMaxSendFrameSize() already
  // advertises MAX_SEND_FRAME_SIZE. Matters for GET_PREFS_TLV's paginated
  // dump (Beebo.cpp/BeeboRepeater.cpp's encodePrefsTlv): without this
  // override the whole table's worth of fields needed several USB round
  // trips it didn't actually need, unlike WiFi's single-page dump.
  size_t getMaxSendFrameSize() const override { return MAX_SEND_FRAME_SIZE; }
};
