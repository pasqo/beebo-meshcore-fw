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

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override;
  size_t getMaxRecvFrameSize() const override { return OTA_FRAME_SIZE; }
  bool lastRecvWasText() const override { return _lastWasText; }
};
