#pragma once

#include "BaseSerialInterface.h"
#include "TransportLog.h"

// Aggregates several BaseSerialInterface transports (e.g. BLE / WiFi / USB)
// behind the single interface the companion mesh expects.
//
// Transports registered as *exclusive* (default) participate in mutual
// exclusion: when one locks the session the others are disabled and
// optionally powered down.  This prevents cross-talk on the single-client
// companion protocol and avoids BT+WiFi RF coexistence issues.
//
// Transports registered as *non-exclusive* (exclusive=false) are never
// disabled by other transports locking, and locking on them never disables
// others.  Use this for USB/Serial: it can run a companion CLI session
// without killing wireless, and wireless sessions leave USB available for
// Serial debug output.
//
// Per transport you may supply two optional callbacks:
//   connected_fn() -> bool   : overrides isConnected() for release detection.
//   power_fn(bool on)        : powers the transport's radio up/down.

#ifndef MULTI_TRANSPORT_MAX
  #define MULTI_TRANSPORT_MAX 2
#endif

class MultiSerialInterface : public BaseSerialInterface {
  typedef bool (*ConnectedFn)();
  typedef void (*PowerFn)(bool on);

  struct SubTransport {
    BaseSerialInterface* iface;
    ConnectedFn connected_fn;
    PowerFn power_fn;
    bool exclusive;
    uint8_t type;   // TLOG_XPORT_* — stable id for logging (order-independent)
  };
  SubTransport _transports[MULTI_TRANSPORT_MAX];
  int _count = 0;
  int _active = -1;          // locked transport, or -1
  bool _enabled = false;

  bool transportConnected(int i) const {
    return _transports[i].connected_fn ? _transports[i].connected_fn() : _transports[i].iface->isConnected();
  }
  // Never ask a transport for more than the caller's buffer holds: a transport's own max
  // (e.g. WiFi's OTA_CHUNK_SIZE) can exceed max_len while unlocked, and dest is
  // only guaranteed to be max_len bytes.
  size_t transportRecvLimit(int i, size_t max_len) const {
    size_t m = _transports[i].iface->getMaxRecvFrameSize();
    return m < max_len ? m : max_len;
  }
  // beebo: TLOG_APP_SESSION_START is logged by each transport itself, at the very
  // first moment of its own accept (e.g. SerialWifiInterface logs it as the
  // first statement of its own accept-a-new-client branch, ahead of its own
  // TCP new client/session ON) -- not here. This function only runs once a
  // first frame has already been parsed (the arbitration decision), which
  // is strictly later than the connection that produced it; logging the
  // start marker here as well would either double it up or (an earlier,
  // broken attempt at this) defer it a further tick past the transport's own
  // connect-time events instead of preceding them.
  void lockOn(int i) {
    _active = i;
    if (!_transports[i].exclusive) return;
    for (int j = 0; j < _count; j++) {
      if (j == i || !_transports[j].exclusive) continue;
      _transports[j].iface->disable();
      if (_transports[j].power_fn) _transports[j].power_fn(false);
    }
  }
  // beebo: `end_reason` is logged exactly once here, not by the caller --
  // every caller already knows the real reason a session is ending
  // (disconnectActive()'s explicit request, or the LOST-timeout branch
  // below), so logging both that reason AND a generic "released" here too
  // produced two SESSION END lines back to back for every real session
  // end. Defaults to TLOG_APP_SESSION_END_RELEASED for a hypothetical
  // future caller with no more specific reason to give.
  void release(int prev, uint8_t end_reason = TLOG_APP_SESSION_END_RELEASED) {
    transport_log.log(end_reason, _transports[prev].type);
    // Reset the released transport's own byte-parser state: a session can end
    // mid-frame (host closes the port between test runs), leaving the
    // parser expecting the rest of a frame that's never coming. Left alone,
    // the transport's *next* session has its opening bytes consumed as that
    // phantom frame's payload instead of parsed as new commands -- a
    // desync that persists (and can cascade) until the stray byte count
    // happens to complete on its own. Was disable()+enable() (same net
    // effect via a side effect of that pair) -- switched to the dedicated
    // resetParserState() hook so this reset doesn't also log a fake
    // transport disable/enable event, or (for SerialWifiInterface)
    // needlessly tear down and rebind the listen socket.
    _transports[prev].iface->resetParserState();
    _active = -1;

    // Only the released transport's own hardware RX buffer can hold a stale
    // phantom-frame remnant from the session that just ended --
    // resetParserState() above resets parser state but not the underlying
    // hardware buffer, so that remnant needs an explicit discard too. An unrelated
    // transport may have a brand-new connection attempt's opening bytes sitting
    // in its own buffer right now (this transport going unpolled while another
    // held the lock doesn't mean whatever arrived on it is stale); sweeping
    // every transport here used to wipe that out too, stalling a same-moment
    // reconnect on another transport until the client's own retry recovered
    // it a couple seconds later.
    _transports[prev].iface->discardStaleRx();

    if (!_enabled) return;
    if (!_transports[prev].exclusive) return;
    for (int i = 0; i < _count; i++) {
      if (i == prev || !_transports[i].exclusive) continue;
      if (_transports[i].power_fn) _transports[i].power_fn(true);
      _transports[i].iface->enable();
    }
  }

public:
  void addInterface(BaseSerialInterface* iface, ConnectedFn connected_fn = nullptr,
                    PowerFn power_fn = nullptr, bool exclusive = true, uint8_t type = 0) {
    if (_count < MULTI_TRANSPORT_MAX) {
      _transports[_count].iface = iface;
      _transports[_count].connected_fn = connected_fn;
      _transports[_count].power_fn = power_fn;
      _transports[_count].exclusive = exclusive;
      _transports[_count].type = type;
      _count++;
    }
  }

  // BaseSerialInterface methods
  void enable() override {
    _enabled = true;
    _active = -1;
    for (int i = 0; i < _count; i++) _transports[i].iface->enable();
  }
  void disable() override {
    _enabled = false;
    for (int i = 0; i < _count; i++) _transports[i].iface->disable();
    _active = -1;
  }
  void disconnectActive() override {
    if (_active < 0) return;
    release(_active, TLOG_APP_SESSION_END_DISCONNECT);   // beebo: release() already does its own disable()/enable() cycle
  }
  bool isEnabled() const override { return _enabled; }

  // Stable TLOG_XPORT_* type of the locked transport, or 0 if idle.
  uint8_t activeTransportType() const { return _active >= 0 ? _transports[_active].type : 0; }

  // beebo: exclusive transports are BLE/TCP (mutually exclusive with each other for
  // RF coexistence, see class comment above); non-exclusive transports are USB. So
  // "any exclusive transport currently powered" / "any non-exclusive transport currently
  // powered" is exactly the BLE-or-TCP / USB power-draw split, independent of
  // which one (if any) currently holds the locked session.
  bool is24GUp() const override {
    for (int i = 0; i < _count; i++) {
      if (_transports[i].exclusive && _transports[i].iface->isEnabled()) return true;
    }
    return false;
  }
  bool isUsbUp() const override {
    for (int i = 0; i < _count; i++) {
      if (!_transports[i].exclusive && _transports[i].iface->isEnabled()) return true;
    }
    return false;
  }

  // beebo: monotonic count of frames exchanged with the companion app (TX + RX),
  // used by main.cpp loop() to flash the status LED on transport activity.
  uint32_t activityCount() const { return _activity; }

  bool isConnected() const override {
    return _active >= 0 && transportConnected(_active);
  }
  bool isWriteBusy() const override {
    return _active >= 0 ? _transports[_active].iface->isWriteBusy() : false;
  }
  size_t writeFrame(const uint8_t src[], size_t len) override {
    if (_active < 0) return 0;
    _activity++;   // beebo: TX frame to app -> transport-activity flash
    return _transports[_active].iface->writeFrame(src, len);
  }

  size_t getMaxRecvFrameSize() const override {
    return _active >= 0 ? _transports[_active].iface->getMaxRecvFrameSize() : MAX_FRAME_SIZE;
  }

  size_t getMaxSendFrameSize() const override {
    return _active >= 0 ? _transports[_active].iface->getMaxSendFrameSize() : MAX_FRAME_SIZE;
  }

  bool lastRecvWasText() const override {
    return _active >= 0 ? _transports[_active].iface->lastRecvWasText() : false;
  }

  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override {
    if (!_enabled) return 0;

    if (_active >= 0) {
      // Whichever transport is active -- exclusive or not -- holds the
      // session until it releases on its own (disconnect debounce below),
      // never preempted by an incoming connection on another transport. A
      // second connection attempt just goes unpolled until then -- see
      // release()'s discardStaleRx() call for why bytes that pile up on an
      // unpolled transport in the meantime are safe to drop. The `exclusive` flag
      // only governs power-management coexistence (lockOn()/release()
      // above): USB stays powered and enabled regardless of who else is
      // locked in, so it remains available as a monitor/debug conduit, but
      // it is never displaced mid-session by BLE/WiFi locking in, and it
      // never displaces them either.
      //
      // Locked: poll only the active transport.
      size_t n = _transports[_active].iface->checkRecvFrame(dest, max_len);
      if (n > 0) { _activity++; _disconnect_since_ms = 0; return n; }   // beebo: RX frame from app

      if (!transportConnected(_active)) {
        uint32_t now = millis();
        if (_disconnect_since_ms == 0) _disconnect_since_ms = now;
        if (now - _disconnect_since_ms >= DISCONNECT_DEBOUNCE_MS) {
          release(_active, TLOG_APP_SESSION_END_LOST);
          _disconnect_since_ms = 0;
        }
      } else {
        _disconnect_since_ms = 0;
      }
      return 0;
    }

    // Idle: poll every enabled transport; first frame wins and locks now.
    for (int i = 0; i < _count; i++) {
      if (!_transports[i].iface->isEnabled()) continue;
      size_t n = _transports[i].iface->checkRecvFrame(dest, transportRecvLimit(i, max_len));
      if (n > 0) {
        lockOn(i);
        _activity++;   // beebo: RX frame from app
        return n;
      }
    }
    return 0;
  }

private:
  uint32_t _activity = 0;   // beebo: TX+RX frame counter for transport-activity LED
  // beebo: debounce for transportConnected() flapping (see checkRecvFrame) -- 0
  // means "not currently observing a disconnect reading". A real, sustained
  // disconnect (cable pulled) still gets caught, just DISCONNECT_DEBOUNCE_MS
  // slower; a transient one-poll blip (e.g. HWCDC's `connected` flag
  // dropping momentarily on a USB_SERIAL_JTAG_INTR_BUS_RESET the peripheral
  // itself self-recovers from) no longer immediately tears down and
  // discards an in-flight session over it -- confirmed via a raw
  // (asyncio-free) pyserial repro that a reply frame's declared length and
  // actually-delivered length can mismatch (168 declared, 104 delivered,
  // remainder never arriving) with the loss traced to exactly this
  // immediate-release path, not to any host-side code.
  uint32_t _disconnect_since_ms = 0;
  static const uint32_t DISCONNECT_DEBOUNCE_MS = 300;
};
