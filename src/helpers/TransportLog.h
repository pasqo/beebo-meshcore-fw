#pragma once

#include <stdint.h>
#include <string.h>
#include "DebugLog.h"

// Ring sized to capture a full interactive session (e.g. a measurement sweep
// of ~40 commands = ~80 cmd recv/done events) for post-mortem fetch. Each
// event is 9 bytes on the wire; the ring is fetched paginated (see serialize).
// 1024 events * 12 bytes (in-memory, padded) = 12KB static/.bss -- bumped
// from 256 since a long-running interactive/test session's own command
// traffic (each CLI round trip logs a CMD_RECV/CMD_DONE pair) can wrap a
// 256-deep ring within a handful of commands, evicting real transport
// events (e.g. TLOG_WIFI_STA_GOT_IP) before they can be fetched. Confirmed
// affordable against a real device's live free heap (`bench status`, not
// just the raw 512KB SRAM figure): 62KB free before this bump, 53KB after.
#define TLOG_MAX_EVENTS 1024

// beebo: the app-level companion session, transport-agnostic (whichever of
// BLE/USB/TCP wins the MultiSerialInterface lock) -- named APP_SESSION_* to
// stay distinct from TLOG_WIFI_SESSION_ON/OFF below (that pair is the raw
// TCP socket's own connected state, one layer down: a socket can go through
// several ON/OFF cycles, or none at all if the TCP layer already dropped
// out, without ever mapping to a real APP_SESSION_START/END here).
#define TLOG_APP_SESSION_START            1
#define TLOG_APP_SESSION_END_RELEASED     2
#define TLOG_APP_SESSION_END_DISCONNECT   3
#define TLOG_APP_SESSION_END_LOST         4
// 5 (TLOG_WIFI_ENABLE) and 6 (TLOG_WIFI_DISABLE) retired 2026-08-31 -- fully
// duplicated by the generic TLOG_XPORT_LINK_VAR_WIFI_IFACE_ENABLED tracking
// (fires on the exact same enable()/disable() transition, every tick, no
// dedicated log call needed).
#define TLOG_WIFI_CLIENT_NEW  7   // detail = (remote_port << 1) | (deviceConnected ? 1 : 0) at accept time
#define TLOG_WIFI_SESSION_ON  8   // detail = the now-live client's remote port
#define TLOG_WIFI_SESSION_OFF 9   // detail = SO_ERROR read from the socket just before it was stop()'d (0 = clean peer FIN, no pending error; nonzero = an errno, e.g. ETIMEDOUT from the keepalive probes below timing out, or ECONNRESET from a peer RST)
// 10 (TLOG_WIFI_POWER_ON) and 11 (TLOG_WIFI_POWER_OFF) retired 2026-08-31 --
// never had a call site (dead since introduction). WiFi radio power is
// already visible via TLOG_XPORT_LINK_VAR_WL_STATUS transitioning off its 255
// (uninitialized) sentinel -- see TLOG_BLE_POWER_ON/OFF below for why BLE
// needed a real dedicated pair instead.
#define TLOG_CMD_RECV        12   // detail = (cmd_frame[0]<<8)|cmd_frame[1] for CMD_BEEBO/CMD_GET_STATS (their second byte is a real sub-id), else just cmd_frame[0] (companion frame received)
#define TLOG_CMD_DONE        13   // detail = same (cmd<<8)|sub encoding as TLOG_CMD_RECV (handler returned)
#define TLOG_WIFI_STA_DISCONNECTED 14   // detail = disconnect reason code
#define TLOG_WIFI_STA_GOT_IP       15   // station (re)associated and got an IP; detail = the IPv4 address, packed MSB-first (octet1<<24 | octet2<<16 | octet3<<8 | octet4)
#define TLOG_BLE_CONNECT           16   // BLE GATT link up (onConnect callback)
#define TLOG_BLE_DISCONNECT        17   // BLE GATT link down (onDisconnect callback)
#define TLOG_DEBUGLOG_READ         18   // marker: debuglog was fetched (boundary)
// 19 retired 2026-09-01 (was TLOG_COEX_PREFER_WIFI, esp_coex_preference_set()
// after a BLE teardown) -- wrong framing: that API arbitrates airtime
// between two *simultaneously* active radios, which BLE/TCP's enforced
// mutual exclusion here guarantees never happens, and it didn't fix the
// bug it was aimed at anyway (BUGS.md 2026-08-31). The real fix was
// reordering applyTransportConfig() to a teardown-pass-then-bring-up-pass
// shape; see that function's own comment.
#define TLOG_WIFI_CLIENT_REJECTED  20   // a second peer's TCP connect was accepted at the OS level (WiFiServer's backlog) while a live session was already locked in -- rejected instead of preempting it; detail = the rejected client's remote port
#define TLOG_CLOCK_SET             21   // RTC epoch changed via CMD_SET_DEVICE_TIME or the text-CLI "time" command; detail = new epoch seconds, so a reader can re-anchor every earlier event's millis() offset against the old epoch and every later one against the new
// 22 retired 2026-08-30 (was TLOG_XPORT_STATE, a one-shot packed-bitfield
// boot snapshot) -- folded into TLOG_XPORT_VAR_CHANGED below, which now
// covers boot too (see that event's own comment for how).
#define TLOG_XPORT_VAR_CHANGED     23   // one tracked variable's value, or a change to it -- see TLOG_XPORT_VAR_* below for detail's (id, old, new) layout
// beebo: periodic low-level WiFi health sample, gated to every
// WIFI_HEALTH_SAMPLE_MS while WiFi is up (loopTransports()) -- unlike
// TLOG_XPORT_VAR_CHANGED's change-triggered vars above, none of which
// moved during the TCP-reachability-degrades-after-a-live-switch bug
// (BUGS.md 2026-08-31): the failure is invisible to every high-level
// state flag Beebo already tracks, so this samples one level lower
// (free heap, RSSI, channel) to catch a silent degradation those flags
// don't see. WiFiServer exposes no public listening-socket fd, so this
// can't read the listening socket's own SO_ERROR the way
// TLOG_WIFI_SESSION_OFF already does for a live client's -- heap/RSSI
// are what's actually reachable without patching the third-party
// arduino-esp32 framework.
// detail: bits 0-15 = free heap in KB (uint16), bits 16-23 = RSSI dBm
// (int8, two's complement), bits 24-31 = WiFi channel (uint8).
#define TLOG_WIFI_HEALTH           24
// beebo: BT controller status (esp_bt_controller_status_t: 0=IDLE,
// 1=INITED, 2=ENABLED) read right before WiFi bring-up starts in
// applyTransportConfig(), only when a BLE teardown preceded it in the
// same switch -- confirms whether BLEDevice::deinit() actually left the
// controller IDLE (as it's supposed to) or stuck INITED/ENABLED, which
// would mean it's still holding the shared RF path when WiFi comes up.
// Added chasing the same TCP-reachability bug as TLOG_WIFI_HEALTH.
#define TLOG_BT_CONTROLLER_STATUS  25
// beebo: connected BLE central's own link-layer address (esp_bd_addr_t, 6
// bytes) -- BLE's counterpart to TLOG_WIFI_STA_GOT_IP, so a trace identifies
// *which* peer is on the link the same way an IP does for TCP. 48 bits
// doesn't fit in one int32 detail, so it's two events logged back-to-back
// (same millis tick) right alongside TLOG_BLE_CONNECT: HI carries the first
// 2 bytes (bda[0..1]) packed MSB-first in the low 16 bits, LO carries the
// last 4 bytes (bda[2..5]) packed MSB-first, matching TLOG_WIFI_STA_GOT_IP's
// own octet packing.
#define TLOG_BLE_CLIENT_ADDR_HI    27
#define TLOG_BLE_CLIENT_ADDR_LO    28
// beebo: periodic low-level BLE health sample, mirroring TLOG_WIFI_HEALTH --
// triggered every BLE_HEALTH_SAMPLE_MS while the radio is enabled
// (loopTransports() -> SerialBLEInterface::requestHealthSample()), same
// gating as TLOG_WIFI_HEALTH's own _wifi_up (not a live app session).
// Heap-only: RSSI is deliberately never read (always logged as 127,
// SerialBLEInterface's BLE_RSSI_UNAVAILABLE) -- an earlier version issued
// an async esp_ble_gap_read_rssi() here whenever a central was connected,
// which raced applyTransportConfig()'s BLE teardown (an outstanding HCI
// command during deinitRadio()) and reproduced as a hang + watchdog reboot
// switching back to TCP after BLE (BUGS.md). See requestHealthSample()'s
// own comment for the full race.
// detail: bits 0-15 = free heap in KB (uint16), bits 16-23 = RSSI dBm
// (int8, two's complement; always 127 -- see above).
#define TLOG_BLE_HEALTH            29
// beebo: MultiSerialInterface's session FSM forcibly dropped a non-owner
// link that reported itself connected while another transport already held
// the session lock (see plans/TRANSPORT_STATE_MACHINE.md's "Session
// arbitration state machine") -- a stray TCP client accepted by WiFi, a
// stray BLE central completing a GATT connect, or a genuine framed/text app
// command arriving on non-owner USB. detail = TLOG_XPORT_* of the evicted
// transport.
#define TLOG_APP_SESSION_EVICTED   30
// beebo: logged as the very first thing setup() does (main.cpp), before
// Serial.begin() -- transport_log.log() only touches RAM/millis(), no
// Serial dependency, so this captures reset_reason from the earliest
// possible point rather than waiting on any transport to come up.
// detail = esp_reset_reason_t. See TransportLog::replayTo()'s own
// comment for how this (and everything else logged before a client
// attaches) ever reaches the host despite predating any live connection.
#define TLOG_BOOT_START            31
// 26 (TLOG_WIFI_LISTEN_ENABLED) retired 2026-09-02 -- fully subsumed by
// TLOG_XPORT_LINK_VAR_WIFI_LISTENING: checkSerialInterface() (and the
// SerialWifiInterface::checkRecvFrame() dead-listener guard inside it)
// always runs before loopTransports()'s _checkTransportStateChanges() in
// the same loop() tick, so any transition this event would have reported
// is already caught and logged as "XPORT wifi.listening 0 -> 1" by the
// generic var-change tracker on the same tick -- confirmed via a real
// capture where both fired 3ms apart for the same underlying change.
// 24/25 never assigned -- an earlier attempt at dedicated BLE radio-power
// events (initRadio()/deinitRadio()) was reverted 2026-08-31: _ble_up (and
// its own TLOG_XPORT_VAR_BLE_UP tracking below) already transitions in
// lockstep with every initRadio()/deinitRadio() call, so a discrete event
// pair would have been an immediate duplicate, the same overlap just
// removed from WIFI_ENABLE/DISABLE/POWER_ON/OFF above.

// beebo: TLOG_XPORT_VAR_CHANGED's detail packs one variable's transition:
// bits 0-7 = var id (TLOG_XPORT_VAR_* below), bits 8-15 = old value (0xFF =
// no previous value -- see below), bits 16-23 = new value (each an 8-bit
// int, plenty for every value here -- booleans are 0/1, wl_status is 0-6,
// active is 0-3). Beebo::loopTransports() re-checks every one of these each
// tick and logs a line the instant any of them differs from its last-known
// value -- no need to wait for a session boundary or guess which moment to
// snapshot; whatever changed shows up on its own, whenever it actually
// happens. TLOG_XPORT_LINK_VAR_ACTIVE transitioning 0 <-> nonzero *is* session
// start/end -- no separate tracking needed for that.
//
// The same mechanism also produces the boot baseline: Beebo::_last_xport_var
// is seeded to -1 (0xFF once masked into the 8-bit old-value field) for
// every var before the very first check, so that first check logs every
// var's actual boot value as a "changed from 0xFF" line -- i.e. reads as
// "this is its value" rather than a real transition (CLI side treats
// old==0xFF as "no previous value" and renders just the new value). One
// harness for both boot state and every later change, instead of two
// separate event shapes to keep in sync (2026-08-30, replaced an earlier
// two-shape design -- a one-shot packed-bitfield snapshot plus this event --
// after the former proved awkward to position correctly and redundant with
// this one; see git log for that history).
#define TLOG_XPORT_LINK_VAR_WIFI_IFACE_ENABLED    0
#define TLOG_XPORT_LINK_VAR_WIFI_IFACE_CONNECTED  1
#define TLOG_XPORT_LINK_VAR_WIFI_LISTENING        2
#define TLOG_XPORT_LINK_VAR_BLE_IFACE_ENABLED     3
#define TLOG_XPORT_LINK_VAR_BLE_IFACE_CONNECTED   4
#define TLOG_XPORT_LINK_VAR_USB_IFACE_ENABLED     5
#define TLOG_XPORT_LINK_VAR_USB_IFACE_CONNECTED   6
#define TLOG_XPORT_LINK_VAR_MULTI_ENABLED         7
#define TLOG_XPORT_LINK_VAR_MULTI_CONNECTED       8
// 9 (was WIFI_STARTED), 11 (was WIFI_NEEDS_RECONNECT), 14 (was BLE_UP), 17
// (was USB_UP) retired 2026-09-03 -- BLE and TCP share one physical 2.4GHz
// radio and were previously two independent per-transport flag sets kept
// in sync by caller-ordering convention, which is exactly the class of bug
// that produced the TCP-unreachable-after-a-live-BLE-switch failure
// (BUGS.md 2026-09-02). Replaced by ONE state variable per
// plans/TRANSPORT_STATE_MACHINE.md: TLOG_XPORT_LINK_VAR_BTP_STATE below carries
// Beebo::BtpState directly, so there is no enum value meaning "BLE and TCP
// both up" -- not representable, not merely avoided.
// beebo: BTP_*_PENDING (Beebo::BtpState) is a real state, not a side flag
// -- entered instead of tearing a radio down immediately whenever a live
// app session still sits on it, exited only once that session ends (see
// Beebo.h's own comment). This is what fully replaced both
// TLOG_XPORT_VAR_TRANSPORT_SWITCH_PENDING and
// TLOG_XPORT_VAR_WIFI_CREDS_RECONNECT_PENDING (both retired 2026-09-03,
// were ids 12/25) -- neither a separate flag nor a separate var to track.
#define TLOG_XPORT_LINK_VAR_BTP_STATE             9   // Beebo::BtpState: 0=OFF, 1=BLE_STARTING, 2=BLE_UP, 3=BLE_PENDING, 4=TCP_STARTING, 5=TCP_UP, 6=TCP_BACKOFF, 7=TCP_PENDING
#define TLOG_XPORT_LINK_VAR_BLE_ADDED             13
// 16 (was USB_ADDED) retired 2026-09-03, folded into USB_STATE below.
#define TLOG_XPORT_LINK_VAR_USB_STATE             16   // Beebo::TransportState: 0=OFF, 1=UP, 2=PENDING
#define TLOG_XPORT_LINK_VAR_WL_STATUS             20   // wl_status_t (WiFi.status())
#define TLOG_XPORT_LINK_VAR_ACTIVE                23   // serial_interface.activeTransportType()
// beebo: MultiSerialInterface::SessionState -- unlike every other var above,
// logged inline from inside checkRecvFrame() itself (the private field
// isn't reachable for Beebo.cpp's usual external _checkTransportStateChanges()
// poll), not tracked in Beebo::_last_xport_var[] and so not bound by
// TLOG_XPORT_VAR_COUNT below. Reuses the same TLOG_XPORT_VAR_CHANGED
// event/detail encoding for a consistent decode path (xportlog.py).
#define TLOG_XPORT_VAR_SESSION_STATE         24   // MultiSerialInterface::SessionState: 0=DISABLED, 1=IDLE, 2=ACTIVE
#define TLOG_XPORT_VAR_COUNT                 26   // array size for Beebo::_last_xport_var[]

// Stable transport type ids, logged as the `detail` of MULTI_* events so the
// transport is identifiable regardless of registration order (which varies with
// which transports are enabled).
#define TLOG_XPORT_BLE  1
#define TLOG_XPORT_USB  2
#define TLOG_XPORT_TCP  3

// beebo: live echo of every TransportLog::log() call to Serial, named
// TRANSPORT_DEBUG_LOGGING for consistency with the other per-subsystem
// debug macros (WIFI_DEBUG_LOGGING, BLE_DEBUG_LOGGING, ...). Prints raw
// (type, detail) rather than a decoded name -- unlike beebo/src/beebo/
// debuglog.py's TLOG_NAMES table, there's no ARDUINO-side name lookup to
// keep in sync here, and this is a live low-level trace read with
// TransportLog.h's own TLOG_*/TLOG_XPORT_VAR_* defines open, not a
// polished report. See kbase/DEBUGGING.md's tier-3 note on why this only
// coexists cleanly with a companion session on BLE/TCP, not USB -- it
// writes to the same Serial/USB-CDC wire usb_interface parses.
#if TRANSPORT_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define TRANSPORT_DEBUG_PRINTLN(F, ...) Serial.printf("XPORT: " F "\n", ##__VA_ARGS__)
#else
  #define TRANSPORT_DEBUG_PRINTLN(...) {}
#endif

struct TransportEvent {
  uint32_t millis;
  uint8_t  type;
  int32_t  detail;
};

class TransportLog {
  TransportEvent _buf[TLOG_MAX_EVENTS];
  uint16_t _head = 0;
  uint16_t _count = 0;

public:
  // `detail` widened from a single byte to 4 bytes (still 0 by default) so
  // TLOG_WIFI_STA_GOT_IP can carry a full packed IPv4 address; every other
  // event type keeps using just the low byte, the upper 3 bytes stay zero.
  void log(uint8_t type, int32_t detail = 0) {
#if ARDUINO
    _buf[_head] = { (uint32_t)::millis(), type, detail };
#else
    _buf[_head] = { 0, type, detail };
#endif
    _head = (_head + 1) % TLOG_MAX_EVENTS;
    if (_count < TLOG_MAX_EVENTS) _count++;

    TRANSPORT_DEBUG_PRINTLN("type=%u detail=%ld", (unsigned)type, (long)detail);
    debug_log.logEvent(type, detail);
  }

  uint16_t count() const { return _count; }

  // beebo: re-emits every event currently in the ring through
  // DebugLog::logEvent(), oldest first -- called once, from
  // Beebo::checkSerialInterface()'s BEEBO_RAW_SUB_DEBUG_LOG_ENABLE
  // handling, on the transition into enabled. This is how a boot-time
  // event (TLOG_BOOT_START, or anything else logged before a client ever
  // attached -- see writeFrameBestEffort()'s own no-ring-buffer comment
  // in DualModeSerialInterface.cpp for why a *live* push that early is
  // simply lost) still reaches a `--debug`/`--debug-boot` host: nothing
  // needs to reach the wire before the host is listening, since the ring
  // already held it and this walks it again once the host actually can
  // receive it. Same (type, detail) wire shape as a live event -- the
  // host-side decode table (debuglog.py's TLOG_NAMES) needs no separate
  // "replay" case, it just sees a burst of ordinary-looking events with
  // old millis() timestamps.
  void replayTo(DebugLog& dl) const {
    uint16_t logical_start = (_count < TLOG_MAX_EVENTS) ? 0 : _head;
    for (uint16_t j = 0; j < _count; j++) {
      uint16_t idx = (logical_start + j) % TLOG_MAX_EVENTS;
      dl.logEvent(_buf[idx].type, _buf[idx].detail);
    }
  }

  // Serialize a page of events (9 bytes each) starting at logical index
  // `offset` (0 = oldest). Writes the total event count to *total so the
  // caller can paginate. Returns the number of bytes written (n_events * 9).
  int serialize(uint8_t *dest, size_t max_len, uint16_t offset, uint16_t *total) const {
    *total = _count;
    if (offset >= _count) return 0;

    const int per_event = 9;
    int avail = max_len / per_event;
    int remaining = _count - offset;
    int n = remaining < avail ? remaining : avail;

    // Oldest logical event is at index 0 (not yet wrapped) or _head (wrapped).
    uint16_t logical_start = (_count < TLOG_MAX_EVENTS) ? 0 : _head;

    int pos = 0;
    for (int j = 0; j < n; j++) {
      uint16_t idx = (logical_start + offset + j) % TLOG_MAX_EVENTS;
      memcpy(&dest[pos], &_buf[idx].millis, 4); pos += 4;
      dest[pos++] = _buf[idx].type;
      memcpy(&dest[pos], &_buf[idx].detail, 4); pos += 4;
    }
    return pos;
  }
};

extern TransportLog transport_log;
