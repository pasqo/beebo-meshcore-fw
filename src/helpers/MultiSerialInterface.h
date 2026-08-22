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

  struct Sub {
    BaseSerialInterface* iface;
    ConnectedFn connected_fn;
    PowerFn power_fn;
    bool exclusive;
    uint8_t type;   // TLOG_XPORT_* — stable id for logging (order-independent)
  };
  Sub _subs[MULTI_TRANSPORT_MAX];
  int _count = 0;
  int _active = -1;          // locked transport, or -1
  bool _enabled = false;

  bool subConnected(int i) const {
    return _subs[i].connected_fn ? _subs[i].connected_fn() : _subs[i].iface->isConnected();
  }
  // Never ask a sub for more than the caller's buffer holds: a sub's own max
  // (e.g. WiFi's OTA_CHUNK_SIZE) can exceed max_len while unlocked, and dest is
  // only guaranteed to be max_len bytes.
  size_t subRecvLimit(int i, size_t max_len) const {
    size_t m = _subs[i].iface->getMaxRecvFrameSize();
    return m < max_len ? m : max_len;
  }
  void lockOn(int i) {
    transport_log.log(TLOG_MULTI_LOCK, _subs[i].type);
    _active = i;
    if (!_subs[i].exclusive) return;
    for (int j = 0; j < _count; j++) {
      if (j == i || !_subs[j].exclusive) continue;
      _subs[j].iface->disable();
      if (_subs[j].power_fn) _subs[j].power_fn(false);
    }
  }
  void release(int prev) {
    transport_log.log(TLOG_MULTI_RELEASE, _subs[prev].type);
    // Reset the released sub's own byte-parser state: a session can end
    // mid-frame (host closes the port between test runs), leaving the
    // parser expecting the rest of a frame that's never coming. Left alone,
    // the sub's *next* session has its opening bytes consumed as that
    // phantom frame's payload instead of parsed as new commands -- a
    // desync that persists (and can cascade) until the stray byte count
    // happens to complete on its own.
    _subs[prev].iface->disable();
    _subs[prev].iface->enable();
    _active = -1;

    // Every other sub went unpolled -- checkRecvFrame() is called only on
    // whichever sub is _active -- for as long as this session held the
    // lock, so anything sitting in a byte-stream sub's hardware RX buffer
    // (a connection attempt that never got a reply) is stale by
    // construction. Drop it now rather than letting the next poll cycle
    // misread it as a fresh command. Covers the released sub too, since
    // disable()/enable() above resets parser state but not the underlying
    // hardware buffer.
    for (int i = 0; i < _count; i++) _subs[i].iface->discardStaleRx();

    if (!_enabled) return;
    if (!_subs[prev].exclusive) return;
    for (int i = 0; i < _count; i++) {
      if (i == prev || !_subs[i].exclusive) continue;
      if (_subs[i].power_fn) _subs[i].power_fn(true);
      _subs[i].iface->enable();
    }
  }

public:
  void addInterface(BaseSerialInterface* iface, ConnectedFn connected_fn = nullptr,
                    PowerFn power_fn = nullptr, bool exclusive = true, uint8_t type = 0) {
    if (_count < MULTI_TRANSPORT_MAX) {
      _subs[_count].iface = iface;
      _subs[_count].connected_fn = connected_fn;
      _subs[_count].power_fn = power_fn;
      _subs[_count].exclusive = exclusive;
      _subs[_count].type = type;
      _count++;
    }
  }

  // BaseSerialInterface methods
  void enable() override {
    _enabled = true;
    _active = -1;
    for (int i = 0; i < _count; i++) _subs[i].iface->enable();
  }
  void disable() override {
    _enabled = false;
    for (int i = 0; i < _count; i++) _subs[i].iface->disable();
    _active = -1;
  }
  void disconnectActive() override {
    if (_active < 0) return;
    transport_log.log(TLOG_MULTI_DISCONNECT, _subs[_active].type);
    int prev = _active;
    _subs[prev].iface->disable();
    _subs[prev].iface->enable();
    release(prev);
  }
  bool isEnabled() const override { return _enabled; }

  // Stable TLOG_XPORT_* type of the locked transport, or 0 if idle.
  uint8_t activeTransportType() const { return _active >= 0 ? _subs[_active].type : 0; }

  // beebo: exclusive subs are BLE/TCP (mutually exclusive with each other for
  // RF coexistence, see class comment above); non-exclusive subs are USB. So
  // "any exclusive sub currently powered" / "any non-exclusive sub currently
  // powered" is exactly the BLE-or-TCP / USB power-draw split, independent of
  // which one (if any) currently holds the locked session.
  bool is24GUp() const override {
    for (int i = 0; i < _count; i++) {
      if (_subs[i].exclusive && _subs[i].iface->isEnabled()) return true;
    }
    return false;
  }
  bool isUsbUp() const override {
    for (int i = 0; i < _count; i++) {
      if (!_subs[i].exclusive && _subs[i].iface->isEnabled()) return true;
    }
    return false;
  }

  // beebo: monotonic count of frames exchanged with the companion app (TX + RX),
  // used by main.cpp loop() to flash the status LED on transport activity.
  uint32_t activityCount() const { return _activity; }

  bool isConnected() const override {
    return _active >= 0 && subConnected(_active);
  }
  bool isWriteBusy() const override {
    return _active >= 0 ? _subs[_active].iface->isWriteBusy() : false;
  }
  size_t writeFrame(const uint8_t src[], size_t len) override {
    if (_active < 0) return 0;
    _activity++;   // beebo: TX frame to app -> transport-activity flash
    return _subs[_active].iface->writeFrame(src, len);
  }

  size_t getMaxRecvFrameSize() const override {
    return _active >= 0 ? _subs[_active].iface->getMaxRecvFrameSize() : MAX_FRAME_SIZE;
  }

  size_t getMaxSendFrameSize() const override {
    return _active >= 0 ? _subs[_active].iface->getMaxSendFrameSize() : MAX_FRAME_SIZE;
  }

  bool lastRecvWasText() const override {
    return _active >= 0 ? _subs[_active].iface->lastRecvWasText() : false;
  }

  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override {
    if (!_enabled) return 0;

    if (_active >= 0) {
      // Whichever transport is active -- exclusive or not -- holds the
      // session until it releases on its own (disconnect debounce below),
      // never preempted by an incoming connection on another transport. A
      // second connection attempt just goes unpolled until then -- see
      // release()'s discardStaleRx() call for why bytes that pile up on an
      // unpolled sub in the meantime are safe to drop. The `exclusive` flag
      // only governs power-management coexistence (lockOn()/release()
      // above): USB stays powered and enabled regardless of who else is
      // locked in, so it remains available as a monitor/debug conduit, but
      // it is never displaced mid-session by BLE/WiFi locking in, and it
      // never displaces them either.
      //
      // Locked: poll only the active transport.
      size_t n = _subs[_active].iface->checkRecvFrame(dest, max_len);
      if (n > 0) { _activity++; _disconnect_since_ms = 0; return n; }   // beebo: RX frame from app

      if (!subConnected(_active)) {
        uint32_t now = millis();
        if (_disconnect_since_ms == 0) _disconnect_since_ms = now;
        if (now - _disconnect_since_ms >= DISCONNECT_DEBOUNCE_MS) {
          transport_log.log(TLOG_MULTI_LOST, _subs[_active].type);
          release(_active);
          _disconnect_since_ms = 0;
        }
      } else {
        _disconnect_since_ms = 0;
      }
      return 0;
    }

    // Idle: poll every enabled transport; first frame wins and locks now.
    for (int i = 0; i < _count; i++) {
      if (!_subs[i].iface->isEnabled()) continue;
      size_t n = _subs[i].iface->checkRecvFrame(dest, subRecvLimit(i, max_len));
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
  // beebo: debounce for subConnected() flapping (see checkRecvFrame) -- 0
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
