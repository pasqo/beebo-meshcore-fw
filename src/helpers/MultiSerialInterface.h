#pragma once

#include "BaseSerialInterface.h"
#include "TransportLog.h"
#include "DebugLog.h"

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

  // beebo: which single transport, if any, owns the app-level command/
  // response conversation right now, and whether the arbitration engine is
  // running at all. Distinct from link state (BtpState/TransportState,
  // Beebo.h): the session lock must never gate a link, only decide whose
  // frames get dispatched. See plans/TRANSPORT_STATE_MACHINE.md.
  //
  // SESSION_DISABLED is a real state here, not a separate _enabled bool --
  // it changes checkRecvFrame()'s own dispatch (nothing polled while
  // disabled), and MultiSerialInterface's enabled-ness never gates any
  // sub-transport's real link power (driveBtp()/driveUsb() own that
  // independently), so this is a session-scope fact, not a link-scope one.
  //
  // No fourth "disconnect pending" state: unlike BTP_BLE_PENDING/
  // BTP_TCP_BACKOFF (Beebo.h), which can persist for many ticks waiting on
  // a real condition/timer, a disconnect request always resolves on the
  // very next tick (Beebo.cpp already guards disconnectActive() behind
  // !isWriteBusy()) -- never observable as the current state between two
  // ticks, so it's a same-tick input (REQ_DISCONNECT below), not a state.
  enum SessionState : uint8_t {
    SESSION_DISABLED,   // the known init state -- no transport serviced, no session possible
    SESSION_IDLE,        // enabled, no locked transport -- poll every enabled link for a first frame
    SESSION_ACTIVE,       // one link locked -- poll it for real app frames; poll every
                           // OTHER enabled link too, and actively evict it if it reports
                           // itself connected (never hand it real frames, never lock in)
  };
  SessionState _state = SESSION_DISABLED;
  int _active = -1;   // accompanying data (which transport), valid only while SESSION_ACTIVE

  // beebo: the only thing enable()/disable()/disconnectActive() are allowed
  // to write -- matches driveBtp()'s ble_on/tcp_on inputs, just as a field
  // instead of a parameter since checkRecvFrame()'s virtual signature can't
  // be extended. Consumed (and cleared) only inside checkRecvFrame().
  enum Request : uint8_t { REQ_NONE, REQ_ENABLE, REQ_DISABLE, REQ_DISCONNECT };
  Request _pending_request = REQ_NONE;

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
  // beebo: TLOG_APP_SESSION_START is logged by each transport itself, at the
  // very first moment of its own accept (SerialWifiInterface logs it as the
  // first statement of its own accept-a-new-client branch, ahead of its own
  // TCP new client/session ON; SerialBLEInterface logs it alongside its own
  // GATT link-up event) -- not here, for those two. This function only runs
  // once a first frame has already been parsed (the arbitration decision),
  // which is strictly later than the connection that produced it; logging
  // the start marker here as well would either double it up for them or (an
  // earlier, broken attempt at this) defer it a further tick past the
  // transport's own connect-time events instead of preceding them.
  //
  // USB has no accept-style event of its own to hook (no discrete "new
  // client" moment -- Serial is just available or not), so it's the one
  // exception: logged right here, gated to only the USB transport type so
  // it can never double up with WiFi/BLE's own self-logged version above.
  //
  // Call only from checkRecvFrame()'s switch (private -- compiler-enforced).
  // Returns the resulting state (SESSION_ACTIVE) -- like driveBtp()'s
  // teardownBleThen_(), doesn't write _state itself; the switch assigns
  // the return value into its own local `next`.
  SessionState lockOn(int i) {
    if (_transports[i].type == TLOG_XPORT_USB) {
      transport_log.log(TLOG_APP_SESSION_START, TLOG_XPORT_USB);
    }
    _active = i;
    if (_transports[i].exclusive) {
      for (int j = 0; j < _count; j++) {
        if (j == i || !_transports[j].exclusive) continue;
        _transports[j].iface->disable();
        if (_transports[j].power_fn) _transports[j].power_fn(false);
      }
    }
    return SESSION_ACTIVE;
  }
  // beebo: `end_reason` is logged exactly once here, not by the caller --
  // every caller already knows the real reason a session is ending
  // (the REQ_DISCONNECT input, or the LOST-timeout branch below), so
  // logging both that reason AND a generic "released" here too produced
  // two SESSION END lines back to back for every real session end.
  // Defaults to TLOG_APP_SESSION_END_RELEASED for a hypothetical future
  // caller with no more specific reason to give.
  //
  // Call only from checkRecvFrame()'s switch, same as lockOn() -- and
  // returns the resulting state (SESSION_IDLE) the same way, rather than
  // writing _state itself. Only ever called while _state == SESSION_ACTIVE
  // (REQ_DISABLE has its own dedicated forced-teardown path and never
  // calls this), so the sibling re-enable loop below is always reachable.
  SessionState release(int prev, uint8_t end_reason = TLOG_APP_SESSION_END_RELEASED) {
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
    //
    // DualModeSerialInterface::resetParserState() now folds in that same
    // hardware-buffer cleanup itself, but only ever discards bytes it can
    // prove belong to the just-abandoned parse (bounded by the declared
    // frame length, or up to a text line's own '\n' terminator) -- see its
    // own comment for the exact per-state rules. Every other
    // BaseSerialInterface subclass's resetParserState() is still
    // software-state-only (SerialBLEInterface, SerialWifiInterface) since
    // this bug is specific to a byte-stream transport with no framing at
    // the hardware level.
    _transports[prev].iface->resetParserState();
    _active = -1;

    if (_transports[prev].exclusive) {
      for (int i = 0; i < _count; i++) {
        if (i == prev || !_transports[i].exclusive) continue;
        if (_transports[i].power_fn) _transports[i].power_fn(true);
        _transports[i].iface->enable();
      }
    }
    return SESSION_IDLE;
  }

  // Call only from checkRecvFrame()'s switch, from SESSION_IDLE/
  // SESSION_ACTIVE (never SESSION_DISABLED -- already a no-op there).
  // Forceful, immediate teardown -- no graceful release() bookkeeping (no
  // END_* log, no discardStaleRx(), no sibling re-enable): matches
  // disable()'s own deliberately forceful semantics. Returns SESSION_DISABLED
  // rather than writing _state itself, same as lockOn()/release().
  SessionState forceDisable() {
    if (_active >= 0) {
      // beebo: same debug-link exposure as release()'s own resetParserState()
      // call -- skip it if the active transport is USB and the debug link is
      // enabled on it, so a raw-control byte mid-exchange isn't blindly
      // consumed. Rarely reached in practice (the one real caller,
      // CMD_FACTORY_RESET, only gets here on a formatFileSystem() failure --
      // the success path reboots before the next checkRecvFrame() tick ever
      // processes the queued REQ_DISABLE), but guarded anyway in case a
      // debug session happens to be live when it is.
      if (!(_transports[_active].type == TLOG_XPORT_USB && debug_log.isEnabled())) {
        _transports[_active].iface->resetParserState();
      }
      _active = -1;
    }
    for (int i = 0; i < _count; i++) _transports[i].iface->disable();
    return SESSION_DISABLED;
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
  //
  // beebo: only checkRecvFrame() writes _state/_active -- the three entry
  // points below just deposit a request, unconditionally, with no state
  // checks of their own; the FSM decides applicability. Same single-writer
  // discipline driveBtp() (Beebo.cpp) holds for _btp_state.
  void enable() override {
    _pending_request = REQ_ENABLE;
  }
  // REQ_DISABLE is deliberately forceful (applies from any state, no
  // graceful wait) -- CMD_FACTORY_RESET (Beebo.cpp) doesn't need a clean
  // release(), it's tearing down for an imminent reboot.
  void disable() override {
    _pending_request = REQ_DISABLE;
  }
  void disconnectActive() override {
    _pending_request = REQ_DISCONNECT;
  }
  bool isEnabled() const override {
    return _state != SESSION_DISABLED;
  }

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
  // Not FSM state/output -- pure telemetry, never read back into any
  // transition decision, incremented from both the RX path below (inside
  // the FSM) and the TX path in writeFrame() (outside it, but a genuinely
  // independent data flow, not a second place deciding session ownership).
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

  // beebo: link state (is a transport's radio/interface powered/listening,
  // BtpState/TransportState in Beebo.h) and session state (SessionState,
  // above -- which single transport owns the app-level command/response
  // conversation) are two distinct state machines: the session lock must
  // never gate a link. Every enabled transport, owner or not, gets its own
  // checkRecvFrame() called every tick; a non-owner's poll result is only
  // ever allowed to trigger eviction, never dispatch or lock in. See
  // plans/TRANSPORT_STATE_MACHINE.md's "Session arbitration state machine".
  //
  // A non-owner link that reports itself connected is forcibly evicted --
  // it cannot be trusted to reject itself: none of the three transports
  // have any concept of session ownership, only their own local
  // connected/not-connected state, so left alone a stray connect attempt
  // would either sit unconsumed forever (WiFi's listen backlog) or complete
  // for real (a BLE central's GATT link, accepted unconditionally by the
  // stack regardless of polling). resetParserState() is reused as the
  // eviction hook on every transport: for WiFi/BLE it also drops the live
  // peer (client.stop()/pServer->disconnect()), for USB there's no
  // connection object to drop, just parser/buffer state to resync.
  //
  // USB is the one exception to "evict on isConnected()": DualModeSerialInterface::
  // isConnected() is a 30s idle-liveness inference (USB_IDLE_TIMEOUT_MS), not
  // "a real peer is here" -- true for the whole window after enable()/any byte,
  // including its own boot. Evicting on that reset USB's own parser every tick
  // for 30s straight while it was legitimately trying to become the winner
  // (hardware-verified 2026-09-03, bunch-mc: a storm of eviction events racing
  // its own first connect). USB is only evicted if it actually delivers a real
  // command frame while non-owner -- see plans/TRANSPORT_STATE_MACHINE.md's
  // "Non-owner eviction, per transport".
  //
  // Call only from checkRecvFrame()'s switch. Runs only against a
  // non-owner index, evicting whichever sub-transport it's given.
  void pollAndEvictIfConnected(int i, uint8_t scratch[], size_t scratch_len) {
    size_t n = _transports[i].iface->checkRecvFrame(scratch, scratch_len);
    bool stray = (_transports[i].type == TLOG_XPORT_USB) ? (n > 0) : transportConnected(i);
    if (stray) {
      transport_log.log(TLOG_APP_SESSION_EVICTED, _transports[i].type);
      _transports[i].iface->resetParserState();
    }
  }

  // beebo: a pure switch, matching driveBtp()'s actual shape exactly --
  // every request check lives inside its relevant case body (repeated per
  // case, same as driveBtp()'s own !ble_on/!tcp_on checks), never hoisted
  // above the switch as a precondition. forceDisable() is the one action
  // genuinely needed from multiple cases, factored out the same way
  // driveBtp() factors out teardownBleThen_(). "Gather" is polling itself
  // (checkRecvFrame() on an enabled link has real side effects, e.g. WiFi's
  // accept-a-new-client), not a separate side-effect-free read, so it's
  // folded into each case's own logic.
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override {
    uint8_t scratch[MAX_FRAME_SIZE];
    size_t result = 0;
    SessionState next = _state;

    switch (_state) {
      case SESSION_DISABLED: {
        if (_pending_request == REQ_ENABLE) {
          _pending_request = REQ_NONE;
          for (int i = 0; i < _count; i++) {
            if (_transports[i].type == TLOG_XPORT_USB && debug_log.isEnabled())
              continue;
            _transports[i].iface->discardStaleRx();
          }
          next = SESSION_IDLE;
        } else if (_pending_request != REQ_NONE) {
          _pending_request = REQ_NONE;
        }
        break;
      }

      case SESSION_IDLE: {
        if (_pending_request == REQ_DISABLE) {
          _pending_request = REQ_NONE;
          next = forceDisable();
          break;
        }
        // REQ_ENABLE (already enabled) / REQ_DISCONNECT (nothing to
        // disconnect) mean nothing here -- same drain-if-inapplicable
        // reasoning as SESSION_DISABLED above.
        if (_pending_request == REQ_ENABLE || _pending_request == REQ_DISCONNECT) {
          _pending_request = REQ_NONE;
        }

        // Poll every enabled transport; first frame wins and locks in.
        // Whichever ones don't produce that frame still get evicted if they
        // report themselves connected, so a stray connect gets rejected
        // even with no session owner yet -- except USB (n is already known
        // 0 here), see pollAndEvictIfConnected()'s own comment on why
        // isConnected() is the wrong signal for it.
        int winner = -1;
        for (int i = 0; i < _count; i++) {
          if (!_transports[i].iface->isEnabled()) continue;
          if (winner < 0) {
            size_t n = _transports[i].iface->checkRecvFrame(dest, transportRecvLimit(i, max_len));
            if (n > 0) { winner = i; result = n; continue; }
            if (_transports[i].type != TLOG_XPORT_USB && transportConnected(i)) {
              transport_log.log(TLOG_APP_SESSION_EVICTED, _transports[i].type);
              _transports[i].iface->resetParserState();
            }
          } else {
            pollAndEvictIfConnected(i, scratch, sizeof(scratch));
          }
        }
        if (winner >= 0) {
          next = lockOn(winner);
          _activity++;   // beebo: RX frame from app
        }
        break;
      }

      case SESSION_ACTIVE: {
        // Captured before anything below can call release() (which clears
        // _active) -- a stale post-release owner (e.g. reading _active
        // after it's already -1) would let the "skip owner" check below
        // match nothing, double-polling the transport just released this
        // same tick, right after its parser was just reset. Its own
        // checkRecvFrame() then parses fresh bytes into a blank parser
        // state; if that returns a real frame, USB's own eviction rule
        // (pollAndEvictIfConnected(), n > 0) would reset it again --
        // splitting a legitimate frame mid-stream. Hardware-verified
        // 2026-09-03 (bunch-mc): binary bytes misread as text, "ERR:
        // unknown command", garbage -- the same FrameParser desync class
        // documented in kbase/CLI_INTERACTIVE_SESSION.md, reintroduced by
        // this stale-index bug.
        int owner = _active;

        if (_pending_request == REQ_DISABLE) {
          _pending_request = REQ_NONE;
          next = forceDisable();
          break;
        }
        if (_pending_request == REQ_ENABLE) {
          _pending_request = REQ_NONE;   // already enabled
        }
        // REQ_DISCONNECT wins over polling the owner for one more frame:
        // the one caller (Beebo.cpp) only requests this after
        // CMD_APP_DISCONNECT was already received as a frame, so anything
        // still buffered this tick is trailing noise, not a real command --
        // and checking it first means it can't be starved by a client that
        // keeps delivering frames with no idle gap.
        if (_pending_request == REQ_DISCONNECT) {
          _pending_request = REQ_NONE;
          next = release(owner, TLOG_APP_SESSION_END_DISCONNECT);
          _disconnect_since_ms = 0;
          break;
        }

        // Poll the owner for real app frames.
        size_t n = _transports[owner].iface->checkRecvFrame(dest, max_len);
        if (n > 0) {
          _activity++;   // beebo: RX frame from app
          _disconnect_since_ms = 0;
          result = n;
        } else if (!transportConnected(owner)) {
          uint32_t now = millis();
          if (_disconnect_since_ms == 0) _disconnect_since_ms = now;
          if (now - _disconnect_since_ms >= DISCONNECT_DEBOUNCE_MS) {
            next = release(owner, TLOG_APP_SESSION_END_LOST);
            _disconnect_since_ms = 0;
          }
        } else {
          _disconnect_since_ms = 0;
        }

        // Every other enabled link stays serviced too -- never displaces
        // the owner (no lockOn() here), just kept from silently accepting
        // a stray connection while starved. Skips `owner`, the real index
        // captured above, regardless of whether release() ran this tick.
        for (int i = 0; i < _count; i++) {
          if (i == owner || !_transports[i].iface->isEnabled()) continue;
          pollAndEvictIfConnected(i, scratch, sizeof(scratch));
        }
        break;
      }
    }

    // beebo: mirrors TLOG_XPORT_LINK_VAR_BTP_STATE/_USB_STATE (TransportLog.h) --
    // logged here, at the point of the actual write, rather than via the
    // external poll-based _checkTransportStateChanges() those use (Beebo.cpp
    // has no access to this private field to poll it that way). Reuses the
    // same TLOG_XPORT_VAR_CHANGED event/detail encoding for a consistent
    // decode path (xportlog.py).
    if (next != _state) {
      int32_t detail = TLOG_XPORT_VAR_SESSION_STATE | ((_state & 0xFF) << 8) | ((next & 0xFF) << 16);
      transport_log.log(TLOG_XPORT_VAR_CHANGED, detail);
    }
    _state = next;
    return result;
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
