#pragma once

#include <stdint.h>
#include <string.h>
#include "BaseSerialInterface.h"

// beebo: unified debug-event subsystem -- one ring (this file, `debug_ring`,
// was `TransportLog`/`transport_log`) and one live USB push mechanism (was
// the separate DebugLog.h/.cpp, folded in here) behind two macro families:
//
// - RLOGH/M/L(type, detail) -- a structured (type, detail) event, always
//   live-pushed over USB whenever the debug link is enabled, and also
//   appended to the ring *unless* severity is Low -- L is link-only, same
//   as DLOGH/M/L below, never occupies a ring slot. `type` is a hand-picked,
//   sometimes-reused-on-purpose per-call-site tag (e.g.
//   RLOG_ID_XSESSION_INIT/_CHANGE is logged from a different file than every
//   other RLOG_ID_XPORT_*/RLOG_ID_XLINK_* id) that doubles as the live
//   frame's id.
// - DLOGH/M/L(id, fmt, ...) -- a free-text printf-style event, live-pushed
//   over USB whenever the debug link is enabled, but never stored in the
//   ring (which only ever holds fixed-size structured records).
//
// Severity is a compile-time gate, not just a runtime label: H is always
// compiled in; M/L only exist when DEBUG_LOG_VERBOSE is defined (1) for the
// build, otherwise every RLOGM/L/DLOGM/L call site compiles to nothing at
// all -- e.g. RLOG_ID_WIFI_HEALTH/RLOG_ID_BLE_HEALTH (both Low) cost nothing in a
// regular build. Both families are unconditionally live-pushed to USB while
// the debug link is enabled -- the ring is a separate, always-on record of
// H/M-severity events only, for post-mortem fetch.
//
// Ring sized to capture a full interactive session (e.g. a measurement sweep
// of ~40 commands = ~80 cmd recv/done events) for post-mortem fetch. Each
// event is 9 bytes on the wire; the ring is fetched paginated (see serialize).
// 512 events * ~13 bytes (in-memory, padded) = ~6.5KB static/.bss.
#define RLOG_MAX_EVENTS 512

// beebo: the app-level companion session, transport-agnostic (whichever of
// BLE/USB/TCP wins the MultiSerialInterface lock) -- named APP_SESSION_* to
// stay distinct from RLOG_ID_WIFI_SESSION_ON/OFF below (that pair is the raw
// TCP socket's own connected state, one layer down: a socket can go through
// several ON/OFF cycles, or none at all if the TCP layer already dropped
// out, without ever mapping to a real APP_SESSION_START/END here).
// GEN_RLOG_NAMES_START -- tools/gen_debug_names.py generates
// beebo/src/beebo/_debug_names_gen.py's RLOG_NAMES from every
// `#define RLOG_ID_<NAME> <id>` in this bracketed region (name = <NAME>, i.e.
// the RLOG_ID_ prefix stripped) -- never hand-edit that generated file. Only
// the actual event-type ids belong in this block; the later
// RLOG_ID_XPORT_LINK_*/RLOG_ID_XPORT_BLE/USB/TCP blocks are a different id
// space (sub-ids packed inside a RLOG_ID_XPORT_*/RLOG_ID_XLINK_*/
// RLOG_ID_XSESSION_* detail, and transport-type tags) and stay outside it.
#define RLOG_ID_APP_SESSION_START            1
#define RLOG_ID_APP_SESSION_END_RELEASED     2
#define RLOG_ID_APP_SESSION_END_DISCONNECT   3
#define RLOG_ID_APP_SESSION_END_LOST         4
// 5 (RLOG_ID_WIFI_ENABLE) and 6 (RLOG_ID_WIFI_DISABLE) retired 2026-08-31 -- fully
// duplicated by the generic RLOG_ID_XPORT_LINK_WIFI_IFACE_ENABLED tracking
// (fires on the exact same enable()/disable() transition, every tick, no
// dedicated log call needed).
#define RLOG_ID_WIFI_CLIENT_NEW  7   // detail = (remote_port << 1) | (deviceConnected ? 1 : 0) at accept time
#define RLOG_ID_WIFI_SESSION_ON  8   // detail = the now-live client's remote port
#define RLOG_ID_WIFI_SESSION_OFF 9   // detail = SO_ERROR read from the socket just before it was stop()'d (0 = clean peer FIN, no pending error; nonzero = an errno, e.g. ETIMEDOUT from the keepalive probes below timing out, or ECONNRESET from a peer RST)
// 10 (RLOG_ID_WIFI_POWER_ON) and 11 (RLOG_ID_WIFI_POWER_OFF) retired 2026-08-31 --
// never had a call site (dead since introduction). WiFi radio power is
// already visible via RLOG_ID_XPORT_LINK_WL_STATUS transitioning off its 255
// (uninitialized) sentinel -- see RLOG_ID_BLE_POWER_ON/OFF below for why BLE
// needed a real dedicated pair instead.
#define RLOG_ID_CMD_RECV        12   // detail = (cmd_frame[0]<<8)|cmd_frame[1] for CMD_BEEBO/CMD_GET_STATS (their second byte is a real sub-id), else just cmd_frame[0] (companion frame received)
#define RLOG_ID_CMD_DONE        13   // detail = same (cmd<<8)|sub encoding as RLOG_ID_CMD_RECV (handler returned)
#define RLOG_ID_WIFI_STA_DISCONNECTED 14   // detail = disconnect reason code
#define RLOG_ID_WIFI_STA_GOT_IP       15   // station (re)associated and got an IP; detail = the IPv4 address, packed MSB-first (octet1<<24 | octet2<<16 | octet3<<8 | octet4)
#define RLOG_ID_BLE_CONNECT           16   // BLE GATT link up (onConnect callback)
#define RLOG_ID_BLE_DISCONNECT        17   // BLE GATT link down (onDisconnect callback)
#define RLOG_ID_DEBUGLOG_READ         18   // marker: debuglog was fetched (boundary)
// 19 retired 2026-09-01 (was RLOG_ID_COEX_PREFER_WIFI, esp_coex_preference_set()
// after a BLE teardown) -- wrong framing: that API arbitrates airtime
// between two *simultaneously* active radios, which BLE/TCP's enforced
// mutual exclusion here guarantees never happens, and it didn't fix the
// bug it was aimed at anyway (BUGS.md 2026-08-31). The real fix was
// reordering applyTransportConfig() to a teardown-pass-then-bring-up-pass
// shape; see that function's own comment.
#define RLOG_ID_WIFI_CLIENT_REJECTED  20   // a second peer's TCP connect was accepted at the OS level (WiFiServer's backlog) while a live session was already locked in -- rejected instead of preempting it; detail = the rejected client's remote port
#define RLOG_ID_CLOCK_SET             21   // RTC epoch changed via CMD_SET_DEVICE_TIME or the text-CLI "time" command; detail = new epoch seconds, so a reader can re-anchor every earlier event's millis() offset against the old epoch and every later one against the new
// 22, 23 retired -- folded into RLOG_ID_XPORT_INIT/_CHANGE below.
// beebo: periodic low-level WiFi health sample, gated to every
// WIFI_HEALTH_SAMPLE_MS while WiFi is up (loopTransports()) -- unlike
// RLOG_ID_XPORT_CHANGE's change-triggered vars above, none of which
// moved during the TCP-reachability-degrades-after-a-live-switch bug
// (BUGS.md 2026-08-31): the failure is invisible to every high-level
// state flag Beebo already tracks, so this samples one level lower
// (free heap, RSSI, channel) to catch a silent degradation those flags
// don't see. WiFiServer exposes no public listening-socket fd, so this
// can't read the listening socket's own SO_ERROR the way
// RLOG_ID_WIFI_SESSION_OFF already does for a live client's -- heap/RSSI
// are what's actually reachable without patching the third-party
// arduino-esp32 framework. Low severity -- periodic and high-volume.
// detail: bits 0-15 = free heap in KB (uint16), bits 16-23 = RSSI dBm
// (int8, two's complement), bits 24-31 = WiFi channel (uint8).
#define RLOG_ID_WIFI_HEALTH           24
// beebo: BT controller status (esp_bt_controller_status_t: 0=IDLE,
// 1=INITED, 2=ENABLED) read right before WiFi bring-up starts in
// applyTransportConfig(), only when a BLE teardown preceded it in the
// same switch -- confirms whether BLEDevice::deinit() actually left the
// controller IDLE (as it's supposed to) or stuck INITED/ENABLED, which
// would mean it's still holding the shared RF path when WiFi comes up.
// Added chasing the same TCP-reachability bug as RLOG_ID_WIFI_HEALTH.
#define RLOG_ID_BT_CONTROLLER_STATUS  25
// beebo: connected BLE central's own link-layer address (esp_bd_addr_t, 6
// bytes) -- BLE's counterpart to RLOG_ID_WIFI_STA_GOT_IP, so a trace identifies
// *which* peer is on the link the same way an IP does for TCP. 48 bits
// doesn't fit in one int32 detail, so it's two events logged back-to-back
// (same millis tick) right alongside RLOG_ID_BLE_CONNECT: HI carries the first
// 2 bytes (bda[0..1]) packed MSB-first in the low 16 bits, LO carries the
// last 4 bytes (bda[2..5]) packed MSB-first, matching RLOG_ID_WIFI_STA_GOT_IP's
// own octet packing.
#define RLOG_ID_BLE_CLIENT_ADDR_HI    27
#define RLOG_ID_BLE_CLIENT_ADDR_LO    28
// beebo: periodic low-level BLE health sample, mirroring RLOG_ID_WIFI_HEALTH --
// triggered every BLE_HEALTH_SAMPLE_MS while the radio is enabled
// (loopTransports() -> SerialBLEInterface::requestHealthSample()), same
// gating as RLOG_ID_WIFI_HEALTH's own _wifi_up (not a live app session). Low
// severity, same reasoning as RLOG_ID_WIFI_HEALTH.
// Heap-only: RSSI is deliberately never read (always logged as 127,
// SerialBLEInterface's BLE_RSSI_UNAVAILABLE) -- an earlier version issued
// an async esp_ble_gap_read_rssi() here whenever a central was connected,
// which raced applyTransportConfig()'s BLE teardown (an outstanding HCI
// command during deinitRadio()) and reproduced as a hang + watchdog reboot
// switching back to TCP after BLE (BUGS.md). See requestHealthSample()'s
// own comment for the full race.
// detail: bits 0-15 = free heap in KB (uint16), bits 16-23 = RSSI dBm
// (int8, two's complement; always 127 -- see above).
#define RLOG_ID_BLE_HEALTH            29
// beebo: MultiSerialInterface's session FSM forcibly dropped a non-owner
// link that reported itself connected while another transport already held
// the session lock (see plans/TRANSPORT_STATE_MACHINE.md's "Session
// arbitration state machine") -- a stray TCP client accepted by WiFi, a
// stray BLE central completing a GATT connect, or a genuine framed/text app
// command arriving on non-owner USB. detail = RLOG_ID_XPORT_* of the evicted
// transport.
#define RLOG_ID_APP_SESSION_EVICTED   30
// beebo: logged as the very first thing setup() does (main.cpp), before
// Serial.begin() -- RLOGH() only touches RAM/millis(), no Serial
// dependency, so this captures reset_reason from the earliest possible
// point rather than waiting on any transport to come up. detail =
// esp_reset_reason_t. See DebugRing::replayRing()'s own comment for how
// this (and everything else logged before a client attaches) ever reaches
// the host despite predating any live connection.
#define RLOG_ID_BOOT_START            31
// beebo: high-level boot-progress checkpoints, each logged once as its
// section of setup() (main.cpp) completes -- lets a boot that hangs or
// resets partway be placed against the last checkpoint it reached, without
// needing a live JTAG session attached at the moment it happens. detail =
// millis() at that checkpoint for RADIO/STORAGE/TRANSPORTS/COMPLETE.
#define RLOG_ID_BOOT_RADIO_READY       40   // radio_init() succeeded
#define RLOG_ID_BOOT_STORAGE_READY     41   // SPIFFS + DataStore mounted
#define RLOG_ID_BOOT_TRANSPORTS_READY  42   // Beebo::beginTransports() returned
#define RLOG_ID_BOOT_COMPLETE          43   // setup() finished, app marked valid
// beebo: one tracked variable's boot value (INIT) or a later change to it
// (CHANGE) -- see RLOG_ID_XPORT_VAR_* below for detail's (id, old, new)
// layout; old is RLOG_XPORT_VAR_NO_PREV_VALUE (0xFF) on INIT. The three
// real FSMs (BtpState/TransportState/SessionState) get their own
// RLOG_ID_XLINK_*/RLOG_ID_XSESSION_* pairs instead, sharing the same var id
// space and detail packing.
#define RLOG_ID_XPORT_INIT      34
#define RLOG_ID_XPORT_CHANGE    35
// beebo: same INIT/CHANGE split for BtpState (BLE/TCP's shared-radio state
// machine) and TransportState (USB's own, Beebo::_usb_state) --
// xportlog.py's _detail_str() special-cases those two var ids to render
// the named enum value.
#define RLOG_ID_XLINK_INIT            36
#define RLOG_ID_XLINK_CHANGE          37
// beebo: same INIT/CHANGE split for MultiSerialInterface::SessionState --
// logged inline from checkRecvFrame() (the private field isn't reachable
// from Beebo.cpp's poll), not via _checkTransportStateChanges().
#define RLOG_ID_XSESSION_INIT         38
#define RLOG_ID_XSESSION_CHANGE       39
// GEN_RLOG_NAMES_END
// 22, 26 retired -- subsumed by RLOG_ID_XPORT_LINK_WIFI_LISTENING.
// 24/25 never assigned.

// beebo: RLOG_ID_XPORT_INIT/_CHANGE (and RLOG_ID_XLINK_*/RLOG_ID_XSESSION_*)
// detail packs one variable's transition: bits 0-7 = var id
// (RLOG_ID_XPORT_LINK_* below), bits 8-15 = old value (0xFF = no previous value),
// bits 16-23 = new value (each an 8-bit int). Beebo::loopTransports()
// re-checks every one of these each tick and logs INIT (first check,
// _last_xport_var seeded to -1) or CHANGE (every value differs from the
// stored one) accordingly. RLOG_ID_XPORT_LINK_ACTIVE transitioning
// 0 <-> nonzero *is* session start/end -- no separate tracking needed.
#define RLOG_ID_XPORT_LINK_WIFI_IFACE_ENABLED    0
#define RLOG_ID_XPORT_LINK_WIFI_IFACE_CONNECTED  1
#define RLOG_ID_XPORT_LINK_WIFI_LISTENING        2
#define RLOG_ID_XPORT_LINK_BLE_IFACE_ENABLED     3
#define RLOG_ID_XPORT_LINK_BLE_IFACE_CONNECTED   4
#define RLOG_ID_XPORT_LINK_USB_IFACE_ENABLED     5
#define RLOG_ID_XPORT_LINK_USB_IFACE_CONNECTED   6
#define RLOG_ID_XPORT_LINK_MULTI_ENABLED         7
#define RLOG_ID_XPORT_LINK_MULTI_CONNECTED       8
// 9, 11, 14, 17 retired -- BLE/TCP collapsed into one BtpState variable
// (RLOG_ID_XPORT_LINK_BTP_STATE), since the two radios share one 2.4GHz
// path and can never legitimately be up independently.
// beebo: BTP_*_PENDING (Beebo::BtpState) is a real state, not a side flag
// -- entered instead of tearing a radio down immediately while a live app
// session still sits on it, exited once that session ends (Beebo.h).
#define RLOG_ID_XPORT_LINK_BTP_STATE             9   // Beebo::BtpState: 0=OFF, 1=BLE_STARTING, 2=BLE_UP, 3=BLE_PENDING, 4=TCP_STARTING, 5=TCP_UP, 6=TCP_BACKOFF, 7=TCP_PENDING
#define RLOG_ID_XPORT_LINK_BLE_ADDED             13
// 16 retired -- folded into USB_STATE below.
#define RLOG_ID_XPORT_LINK_USB_STATE             16   // Beebo::TransportState: 0=OFF, 1=UP, 2=PENDING
#define RLOG_ID_XPORT_LINK_WL_STATUS             20   // wl_status_t (WiFi.status())
#define RLOG_ID_XPORT_LINK_ACTIVE                23   // serial_interface.activeTransportType()
// beebo: MultiSerialInterface::SessionState -- not tracked in
// Beebo::_last_xport_var[], so not bound by RLOG_XPORT_VAR_COUNT below.
#define RLOG_ID_XPORT_LINK_SESSION_STATE         24   // MultiSerialInterface::SessionState: 0=DISABLED, 1=IDLE, 2=ACTIVE
#define RLOG_XPORT_VAR_COUNT                 26   // array size for Beebo::_last_xport_var[]

// Stable transport type ids, logged as the `detail` of MULTI_* events so the
// transport is identifiable regardless of registration order (which varies with
// which transports are enabled).
#define RLOG_ID_XPORT_BLE  1
#define RLOG_ID_XPORT_USB  2
#define RLOG_ID_XPORT_TCP  3

// beebo: live echo of every RLOGH/M/L() call to Serial, named
// TRANSPORT_DEBUG_LOGGING for consistency with the other per-subsystem
// debug macros (WIFI_DEBUG_LOGGING, BLE_DEBUG_LOGGING, ...). Prints raw
// (type, detail) rather than a decoded name -- unlike beebo/src/beebo/
// xportlog.py's RLOG_NAMES table, there's no ARDUINO-side name lookup to
// keep in sync here, and this is a live low-level trace read with this
// file's own RLOG_ID_*/RLOG_ID_XPORT_VAR_* defines open, not a polished report.
// See kbase/DEBUGGING.md's tier-3 note on why this only coexists cleanly
// with a companion session on BLE/TCP, not USB -- it writes to the same
// Serial/USB-CDC wire usb_interface parses.
#if TRANSPORT_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define TRANSPORT_DEBUG_PRINTLN(F, ...) Serial.printf("XPORT: " F "\n", ##__VA_ARGS__)
#else
  #define TRANSPORT_DEBUG_PRINTLN(...) {}
#endif

// beebo: severity is a real, transmitted field (so a live `--debug` reader
// can see/filter by it), but its main job is compile-time: M/L call sites
// only exist in a DEBUG_LOG_VERBOSE build (see RLOGH/M/L/DLOGH/M/L macros below)
// -- in a normal build they cost nothing, not even the call.
#define DLOG_SEV_H 0
#define DLOG_SEV_M 1
#define DLOG_SEV_L 2

// beebo: DLOGH/M/L(id, fmt, ...) call-site ids, named the same way RLOG_ID_*
// (above) already names every RLOGH/M/L call site -- so a decoded live line
// shows a meaningful name (e.g. "WIFI_TORN_DOWN") instead of a bare number.
// Defined here, not per-.cpp-file, so the id space stays centrally visible
// and collision-free (see DEBUG_LOG_ENABLE's own desc in protocol.yaml for
// the wire shape these travel in).
// GEN_DLOG_NAMES_START -- tools/gen_debug_names.py generates
// beebo/src/beebo/_debug_names_gen.py's DLOG_NAMES from every
// `#define DLOG_ID_<NAME> <id>` in this bracketed region (name = <NAME>,
// i.e. the DLOG_ID_ prefix stripped) -- never hand-edit that generated file.
#define DLOG_ID_WIFI_LISTENER_REBUILD      1   // SerialWifiInterface.cpp: listening socket found dead, rebuilding it
#define DLOG_ID_BLE_TORN_DOWN              2   // Beebo.cpp: BLE radio deinit complete, heap snapshot
#define DLOG_ID_WIFI_TORN_DOWN             3   // Beebo.cpp: WiFi radio deinit complete, heap-capability snapshot
#define DLOG_ID_WIFI_BRINGUP_AFTER_BLE     4   // Beebo.cpp: WiFi bring-up right after a BLE teardown in the same switch, BT controller status snapshot
#define DLOG_ID_ACK_TABLE_MATCH            5   // BeeboCompanion.cpp processAck(): expected_ack_table[] match found, about to push PUSH_CODE_SEND_CONFIRMED
#define DLOG_ID_ACK_TABLE_WRITE_RESULT     6   // BeeboCompanion.cpp processAck(): writeFrame() return value for the PUSH_CODE_SEND_CONFIRMED push
#define DLOG_ID_ACK_CONNECTIONS_FALLBACK   7   // BeeboCompanion.cpp processAck(): no expected_ack_table[] match, falling through to checkConnectionsAck()
#define DLOG_ID_ACK_NO_MATCH               8   // BeeboCompanion.cpp processAck(): neither expected_ack_table[] nor checkConnectionsAck() matched
// 9-99 reserved for future non-trace DLOGH/M/L call sites.
// beebo: BEEBO_USB_RXTX_TRACE (DualModeSerialInterface.cpp) opt-in trace ids.
#define DLOG_ID_USB_RX_TRACE               100   // one byte read off the wire, with the parser state it landed in
#define DLOG_ID_USB_TX_TRACE               101   // one writeFrame() call sending a frame out
#define DLOG_ID_USB_RX_DISCARD_STALE       102   // discardStaleRx(): bytes drained from a newly-(re)polled sub's stale hardware RX buffer
#define DLOG_ID_USB_RX_RESET_PARSER        103   // resetParserState(): mid-command parser state discarded at session end
#define DLOG_ID_USB_RX_PENDING_RAW_MARKER  104   // hasPendingRawMarker(): peek result while resolving a possible raw-control marker byte
#define DLOG_ID_USB_RX_BODY                105   // checkRecvFrame()'s MODE_FRAMED_BODY read progress
// GEN_DLOG_NAMES_END

#ifndef DEBUG_LOG_VERBOSE
#define DEBUG_LOG_VERBOSE 0
#endif

struct DebugEvent {
  uint32_t millis;
  uint8_t  type;
  int32_t  detail;
  uint8_t  severity;   // in-RAM only -- serialize()'s 9-byte wire shape omits it, see that method's own comment
  // beebo: __FILE__/__LINE__ from the RLOG call site, kept only so
  // replayRing() can report the real origin instead of a synthetic
  // "replay:0" -- in-RAM only, never on the wire (serialize()'s 9-byte
  // shape doesn't carry it either, same as severity above). `file` is a
  // string-literal pointer (always the same address for a given call
  // site, valid for the process lifetime) -- no copy needed.
  const char* file;
  uint16_t line;
};

class DebugRing {
  DebugEvent _buf[RLOG_MAX_EVENTS];
  uint16_t _head = 0;
  uint16_t _count = 0;

  BaseSerialInterface* _serial = nullptr;
  uint8_t _resp_code = 0;
  uint8_t _log_sub_id = 0;
  uint8_t _rlog_sub_id = 0;
  bool _enabled = false;

public:
  void attach(BaseSerialInterface* serial, uint8_t resp_code,
              uint8_t log_sub_id, uint8_t rlog_sub_id) {
    _serial = serial;
    _resp_code = resp_code;
    _log_sub_id = log_sub_id;
    _rlog_sub_id = rlog_sub_id;
  }
  void setEnabled(bool enabled) { _enabled = enabled; }
  bool isEnabled() const { return _enabled; }

  // RLOGH/M/L: appended to the ring unless severity is Low (L is link-only,
  // like DLOGH/M/L); always live-pushed over USB (RESP_CODE_BEEBO/DEBUG_TLOG
  // frame) whenever the debug link is enabled, regardless of severity.
  void logRing(const char* file, int line, uint8_t type, uint8_t severity, int32_t detail = 0);

  // DLOGH/M/L: never touches the ring (fixed-size records can't hold
  // arbitrary text) -- only live-pushed over USB (RESP_CODE_BEEBO/DEBUG_LOG
  // frame) whenever the debug link is enabled.
  void logLink(const char* file, int line, uint16_t id, uint8_t severity, const char* fmt, ...) __attribute__((format(printf, 6, 7)));

  uint16_t count() const { return _count; }

  // beebo: re-emits every event currently in the ring as a live DEBUG_TLOG
  // push, oldest first -- called once, from Beebo::checkSerialInterface()'s
  // BEEBO_RAW_SUB_DEBUG_LOG_ENABLE handling, on the transition into enabled.
  // This is how a boot-time event (RLOG_ID_BOOT_START, or anything else logged
  // before a client ever attached -- see writeFrameBestEffort()'s own
  // no-ring-buffer comment in DualModeSerialInterface.cpp for why a *live*
  // push that early is simply lost) still reaches a `--debug` host: nothing
  // needs to reach the wire before the host is listening, since the ring
  // already held it and this walks it again once the host actually can
  // receive it.
  void replayRing() const {
    uint16_t logical_start = (_count < RLOG_MAX_EVENTS) ? 0 : _head;
    for (uint16_t j = 0; j < _count; j++) {
      uint16_t idx = (logical_start + j) % RLOG_MAX_EVENTS;
      pushRlogFrame(_buf[idx].file, _buf[idx].line, _buf[idx].type, _buf[idx].severity, _buf[idx].detail, _buf[idx].millis);
    }
  }

  // Serialize a page of events (9 bytes each) starting at logical index
  // `offset` (0 = oldest). Writes the total event count to *total so the
  // caller can paginate. Returns the number of bytes written (n_events * 9).
  // beebo: this 9-byte-per-event wire shape (millis:4 + type:1 + detail:4,
  // no severity) predates severity and is left unchanged -- `beebo monitor
  // transport`'s GET_STATS/STATS_TYPE_TRANSPORT paging protocol stays wire-
  // compatible; severity is only ever transmitted on the *live* DEBUG_TLOG
  // push (logRing()/replayRing()), not this offline fetch.
  int serialize(uint8_t *dest, size_t max_len, uint16_t offset, uint16_t *total) const {
    *total = _count;
    if (offset >= _count) return 0;

    const int per_event = 9;
    int avail = max_len / per_event;
    int remaining = _count - offset;
    int n = remaining < avail ? remaining : avail;

    // Oldest logical event is at index 0 (not yet wrapped) or _head (wrapped).
    uint16_t logical_start = (_count < RLOG_MAX_EVENTS) ? 0 : _head;

    int pos = 0;
    for (int j = 0; j < n; j++) {
      uint16_t idx = (logical_start + offset + j) % RLOG_MAX_EVENTS;
      memcpy(&dest[pos], &_buf[idx].millis, 4); pos += 4;
      dest[pos++] = _buf[idx].type;
      memcpy(&dest[pos], &_buf[idx].detail, 4); pos += 4;
    }
    return pos;
  }

private:
  // beebo: shared header both live frame kinds write --
  // [resp_code:1][sub_id:1][millis:4 LE][id:2 LE][severity:1][line:2 LE]
  // [file_len:1][file], 12 bytes fixed + file_len bytes. Returns the
  // position past the file field, or 0 if there wasn't even room for the
  // fixed header (caller must bail out without sending). base_len is
  // clamped to what fits in the caller's remaining budget (avail_after,
  // e.g. detail's fixed 4 bytes for pushRlogFrame()) so a very long file
  // path degrades to a truncated name instead of starving the caller's
  // own trailing fields.
  static size_t writeHeader(uint8_t* out, size_t cap, size_t avail_after,
                             uint8_t resp_code, uint8_t sub_id, uint16_t id,
                             uint8_t severity, int line, const char* file,
                             uint32_t ms) {
    if (cap < 12) return 0;
    out[0] = resp_code;
    out[1] = sub_id;
    memcpy(&out[2], &ms, 4);
    memcpy(&out[6], &id, 2);
    out[8] = severity;
    uint16_t line16 = (uint16_t)line;
    memcpy(&out[9], &line16, 2);

    const char* base = strrchr(file, '/');
    base = base ? base + 1 : file;
    size_t base_len = strlen(base);
    size_t room = (cap >= 12 + avail_after) ? cap - 12 - avail_after : 0;
    if (base_len > room) base_len = room;
    if (base_len > 255) base_len = 255;

    out[11] = (uint8_t)base_len;
    memcpy(&out[12], base, base_len);
    return 12 + base_len;
  }

  void pushRlogFrame(const char* file, int line, uint16_t id, uint8_t severity,
                      int32_t detail, uint32_t ms) const {
    // beebo: no isConnected() gate -- DualModeSerialInterface::isConnected()
    // is an unconditional `return true` stub (no real way to detect a host-
    // side close on the native USB-Serial-JTAG peripheral, see that file's
    // own comment), so it never actually prevented a push into the void.
    // writeFrameBestEffort() below is what makes an unread push cheap now,
    // not this check -- confirmed on real hardware that relying on
    // isConnected() here let every push retry-block the main loop for up to
    // ZERO_WRITE_GIVEUP_MS (3s) whenever nothing was draining the USB TX
    // side, stalling completely unrelated traffic (BLE/TCP included) on
    // every single event while armed.
    if (!_enabled || !_serial) return;

    uint8_t out[64];   // plenty for header + a file basename + the 4-byte detail
    size_t pos = writeHeader(out, sizeof(out), 4, _resp_code, _rlog_sub_id, id, severity, line, file, ms);
    if (pos == 0) return;

    memcpy(&out[pos], &detail, 4);
    pos += 4;

    const_cast<DebugRing*>(this)->_serial->writeFrameBestEffort(out, pos);
  }
};

extern DebugRing debug_ring;

// beebo: RLOGH/DLOGH are always compiled in; RLOGM/L and DLOGM/L only
// exist in a DEBUG_LOG_VERBOSE build -- in a normal build those call sites
// vanish entirely (not even evaluated), so a hot/high-volume Low-severity
// site (RLOG_ID_WIFI_HEALTH, RLOG_ID_BLE_HEALTH) costs nothing by default.
#define RLOGH(type, ...) debug_ring.logRing(__FILE__, __LINE__, type, DLOG_SEV_H, ##__VA_ARGS__)
#define DLOGH(id, fmt, ...) debug_ring.logLink(__FILE__, __LINE__, id, DLOG_SEV_H, fmt, ##__VA_ARGS__)

#if DEBUG_LOG_VERBOSE
#define RLOGM(type, ...) debug_ring.logRing(__FILE__, __LINE__, type, DLOG_SEV_M, ##__VA_ARGS__)
#define RLOGL(type, ...) debug_ring.logRing(__FILE__, __LINE__, type, DLOG_SEV_L, ##__VA_ARGS__)
#define DLOGM(id, fmt, ...) debug_ring.logLink(__FILE__, __LINE__, id, DLOG_SEV_M, fmt, ##__VA_ARGS__)
#define DLOGL(id, fmt, ...) debug_ring.logLink(__FILE__, __LINE__, id, DLOG_SEV_L, fmt, ##__VA_ARGS__)
#else
#define RLOGM(type, ...) do {} while (0)
#define RLOGL(type, ...) do {} while (0)
#define DLOGM(id, fmt, ...) do {} while (0)
#define DLOGL(id, fmt, ...) do {} while (0)
#endif

// beebo: BEEBO_RAW_SUB_DEBUG_LOG_ENABLE=1 lives here (not generated from
// protocol.yaml, since this raw-marker layer is deliberately distinct from
// -- and below -- the CMD_BEEBO/sub_id wire format that generator covers).
// BEEBO_RAW_SUB_KEEPALIVE=2 is a fire-and-forget no-op that lets a client
// keep an otherwise-idle USB session's liveness timer alive without
// touching this enable/disable state -- see connect.py's periodic
// keepalive during `beebo -i`.
#define BEEBO_RAW_SUB_DEBUG_LOG_ENABLE 1
#define BEEBO_RAW_SUB_KEEPALIVE 2
