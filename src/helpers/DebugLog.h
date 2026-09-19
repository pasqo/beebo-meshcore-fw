#pragma once

#include <stdint.h>
#include <string.h>
#include "BaseSerialInterface.h"
#include "MonRing.h"

// beebo: unified debug-event subsystem -- one live push mechanism (was the
// separate DebugLog.h/.cpp, folded in here) behind two macro families:
//
// - RLOGH/M(type, detail) -- a structured (type, detail) event, forwarded to
//   MonRing's MON_DEBUG kind (see _debug_sink below) whenever a sink is
//   wired up -- MON_DEBUG carries file:line and is the sole record of these,
//   both for post-mortem fetch/replay (GET_MONRING/beginMlogReplay()) and
//   live (`_usb_raw.py`'s `_fmt_mlog_line()` renders it). This file used to
//   also keep its own ring + live DEBUG_TLOG push + replay-on-enable for
//   RLOG (retired 2026-09-11 once MON_DEBUG covered the same ground -- see
//   git history for the removed `serialize()`/`beginReplay()`/
//   `replayStep()`/`pushRlogFrame()` machinery).
//   `type` is a hand-picked, sometimes-reused-on-purpose per-call-site tag
//   (e.g. RLOG_ID_XSESSION_INIT/_CHANGE is logged from a different file than
//   every other RLOG_ID_XPORT_*/RLOG_ID_XLINK_* id) that doubles as
//   MON_DEBUG's own id. RLOGL doesn't exist -- MON_DEBUG structurally can't
//   represent Low severity (DebugRecord.type only encodes H vs M), so every
//   former RLOGL call site was converted to DLOGL (free text) instead.
// - DLOGH/M/L(id, fmt, ...) -- a free-text printf-style event, live-pushed
//   over physical USB *and* over whichever transport currently holds the
//   companion session, if any and if not USB itself (see attach()'s own
//   comment) whenever the debug link is enabled. Never stored anywhere
//   (fixed-size MonRing records can't hold arbitrary text) -- live-only,
//   with no post-mortem fetch/replay of its own.
//
// Severity is a compile-time gate, not just a runtime label: H is always
// compiled in; M/L only exist when DEBUG_LOG_VERBOSE is defined (1) for the
// build, otherwise every RLOGM/DLOGM/L call site compiles to nothing at
// all -- e.g. DLOG_ID_WIFI_HEALTH/DLOG_ID_BLE_HEALTH (both Low) cost nothing in a
// regular build.

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
// 12 (RLOG_ID_CMD_RECV) and 13 (RLOG_ID_CMD_DONE) retired 2026-09-11 --
// converted to DLOG_ID_CMD_RECV/DLOG_ID_CMD_DONE (RLOGL/DEBUG_TLOG retired).
#define RLOG_ID_WIFI_STA_DISCONNECTED 14   // detail = disconnect reason code
#define RLOG_ID_WIFI_STA_GOT_IP       15   // station (re)associated and got an IP; detail = the IPv4 address, packed MSB-first (octet1<<24 | octet2<<16 | octet3<<8 | octet4)
// beebo: BLE session-FSM bring-up sequence, one grouped id instead of a
// separate top-level RLOG_ID per step -- see checkRecvFrame()'s
// notify_ready gate -- detail bits 0-7 = sub-id (RLOG_ID_BLE_HANDSHAKE_*
// below), bits 8-31 = sub-id-specific payload, same shape as
// RLOG_ID_XPORT_INIT/_CHANGE's var-id packing further below. Teardown
// (RLOG_ID_BLE_DISCONNECT) stays its own top-level id -- this group is
// bring-up only.
#define RLOG_ID_BLE_HANDSHAKE         16
#define RLOG_ID_BLE_DISCONNECT        17   // BLE GATT link down (onDisconnect callback)
// 18 (RLOG_ID_DEBUGLOG_READ) retired 2026-09-11 -- was a read-boundary
// marker for the now-removed STATS_TYPE_TRANSPORT offline fetch.
// 19 retired 2026-09-01 (was RLOG_ID_COEX_PREFER_WIFI, esp_coex_preference_set()
// after a BLE teardown) -- wrong framing: that API arbitrates airtime
// between two *simultaneously* active radios, which BLE/TCP's enforced
// mutual exclusion here guarantees never happens, and it didn't fix the
// bug it was aimed at anyway. The real fix was
// reordering applyTransportConfig() to a teardown-pass-then-bring-up-pass
// shape; see that function's own comment.
#define RLOG_ID_WIFI_CLIENT_REJECTED  20   // a second peer's TCP connect was accepted at the OS level (WiFiServer's backlog) while a live session was already locked in -- rejected instead of preempting it; detail = the rejected client's remote port
#define RLOG_ID_CLOCK_SET             21   // RTC epoch (re)established -- via CMD_SET_DEVICE_TIME/the text-CLI "time" command, or Beebo::initMonRing()'s boot-time anchor capture; detail = the epoch seconds now in effect, so a reader can re-anchor every earlier event's millis() offset against the old epoch (or none, if this is the first) and every later one against the new
// 22, 23 retired -- folded into RLOG_ID_XPORT_INIT/_CHANGE below.
// 24 (RLOG_ID_WIFI_HEALTH) retired 2026-09-11 -- converted to
// DLOG_ID_WIFI_HEALTH (RLOGL/DEBUG_TLOG retired).
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
// (same millis tick) right alongside RLOG_ID_BLE_HANDSHAKE's CONNECT step: HI carries the first
// 2 bytes (bda[0..1]) packed MSB-first in the low 16 bits, LO carries the
// last 4 bytes (bda[2..5]) packed MSB-first, matching RLOG_ID_WIFI_STA_GOT_IP's
// own octet packing.
#define RLOG_ID_BLE_CLIENT_ADDR_HI    27
#define RLOG_ID_BLE_CLIENT_ADDR_LO    28
// 29 (RLOG_ID_BLE_HEALTH) retired 2026-09-11 -- converted to
// DLOG_ID_BLE_HEALTH (RLOGL/DEBUG_TLOG retired).
// MultiSerialInterface's session FSM forcibly dropped a non-owner
// link that reported itself connected while another transport already held
// the session lock -- a stray TCP client accepted by WiFi, a
// stray BLE central completing a GATT connect, or a genuine framed/text app
// command arriving on non-owner USB. detail = RLOG_ID_XPORT_* of the evicted
// transport.
#define RLOG_ID_APP_SESSION_EVICTED   30
// beebo: logged as the very first thing setup() does (main.cpp), before
// Serial.begin() -- RLOGH() only touches RAM/millis(), no Serial
// dependency, so this captures reset_reason from the earliest possible
// point rather than waiting on any transport to come up. detail =
// esp_reset_reason_t. See DebugLog::replayRing()'s own comment for how
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
// beebo: BLE RSSI async-request tracing -- reintroduces esp_ble_gap_read_rssi()
// (see RLOG_ID_BLE_HEALTH's own comment for why it was removed: an earlier
// version raced it against applyTransportConfig()'s BLE teardown and
// reproduced as a hang + watchdog reboot), purely to observe via the ring
// how often a teardown actually lands while a read is still outstanding --
// not a fix (no wait/cancel yet), just instrumentation to size the problem
// before deciding whether/how to build one. REQUESTED/COMPLETE bracket one
// read's lifetime (detail: RSSI dBm, int8 two's complement, for COMPLETE
// only -- REQUESTED's detail is unused/0); TEARDOWN_WHILE_INFLIGHT fires
// from SerialBLEInterface::deinitRadio() when it runs while a read is still
// outstanding (detail unused/0, just a marker). High severity (not Low, like
// RLOG_ID_BLE_HEALTH) so TEARDOWN_WHILE_INFLIGHT is always ring-captured,
// even in a non-verbose build -- this is exactly the anomaly this
// instrumentation exists to catch.
#define RLOG_ID_BLE_RSSI_REQUESTED               44
#define RLOG_ID_BLE_RSSI_COMPLETE                45
#define RLOG_ID_BLE_RSSI_TEARDOWN_WHILE_INFLIGHT  46
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
// beebo: radio_driver.recvRaw() got an RX interrupt but _radio->readData()
// itself failed (CRC mismatch, length mismatch, or another RadioLib error)
// -- the packet is dropped and counted in n_recv_errors (RadioLibWrappers.cpp),
// but that counter alone can't distinguish a bad-CRC reception (real RF
// noise/interference) from any other RadioLib-level failure. detail packs
// four signed bytes (bits 0-7 = LSB): RadioLib error code (e.g.
// RADIOLIB_ERR_CRC_MISMATCH = -6), RSSI dBm, SNR dB (rounded to the nearest
// integer -- the radio reports it with sub-dB precision, but a whole dB is
// enough for this diagnostic), and the current noise floor dBm -- all as
// int8 two's complement.
#define RLOG_ID_RADIO_RECV_ERROR      47
// 48 (RLOG_ID_CPU_SNAPSHOT) retired 2026-09-11 -- converted to
// DLOG_ID_CPU_SNAPSHOT (RLOGL/DEBUG_TLOG retired; see kbase/
// CPU_UTILIZATION.md for the 1-minute busy/idle snapshot itself).
// beebo: this device's own local BLE link-layer address (esp_bd_addr_t, 6
// bytes) -- BLE's counterpart to RLOG_ID_BLE_CLIENT_ADDR_HI/LO, but for
// the advertising side rather than a connecting peer, so a scan/connect
// (e.g. `beebo -b scan`) can be matched against a specific device's
// address without a physical serial console. Logged once, right after
// BLEDevice::init() in initRadio() -- i.e. whenever BLE comes up (boot
// with BLE enabled, or a live "set ble on"), not on every advertising
// restart. Same HI/LO split and packing as RLOG_ID_BLE_CLIENT_ADDR_HI/LO:
// HI carries the first 2 bytes (bda[0..1]) packed MSB-first in the low 16
// bits, LO carries the last 4 bytes (bda[2..5]) packed MSB-first.
#define RLOG_ID_BLE_LOCAL_ADDR_HI     49
#define RLOG_ID_BLE_LOCAL_ADDR_LO     50
// beebo: fired once per Beebo::applyClockSync() call with a nonzero SECS
// (a pure read logs nothing) -- detail = the exact signed ms delta this
// call classified against (> 0 = was behind, < 0 = was ahead, magnitude
// <= CLOCK_SYNC_WINDOW_MS = already in sync, nothing corrected). _user[0]
// is one of RLOG_CLOCK_SRC_SYNC (an explicit client request -- session-
// based, RTT-compensated) or RLOG_CLOCK_SRC_KEEPALIVE (BEEBO_RAW_SUB_TIME_SYNC's
// session-less background keepalive, debug_link.py) -- the two have very
// different accuracy, so don't compare their deltas as if they were the
// same measurement.
#define RLOG_ID_CLOCK_SYNC  51
// beebo: boot-time sub-checkpoints inside Beebo::begin(), between
// RLOG_ID_BOOT_STORAGE_READY and RLOG_ID_BOOT_TRANSPORTS_READY -- added to
// break down a 2026-09-09 report of a multi-second gap in that span
// (SPIFFS reads via loadRoleState() were the leading suspect, but never
// measured directly). detail = millis() at each point, same convention as
// the other RLOG_ID_BOOT_* events.
#define RLOG_ID_BOOT_ROLE_STATE_LOADED 53   // both loadRoleState() calls done
#define RLOG_ID_BOOT_ROLE_BEGIN_DONE   54   // beginCompanion()/beginRepeater() done
// beebo: NVS rtc_ts read/write tracing (see
// kbase/RTC_RESET_RECOVERY.md) -- added to make the "Behavior across
// reset kinds" cases directly observable instead of inferred from CLOCK_SET
// alone: a stale post-reset clock is either a genuinely lost RTC (no PULL
// logged at all this boot -- see ESP32RTCClock::begin()'s live-clock
// plausibility check) or a PULL that restored an old PUSH because nothing
// refreshed rtc_ts in between. detail = the epoch seconds value in both
// cases.
#define RLOG_ID_CLOCK_NVM_PUSH  55   // setRebootRTCTime() wrote rtc_ts
#define RLOG_ID_CLOCK_NVM_PULL  56   // ESP32RTCClock::begin() restored rtc_ts on ESP_RST_POWERON
// beebo: unconditionally logs exactly what time() returned at the very top
// of ESP32RTCClock::begin(), before any correction/restore touches it --
// direct evidence of whether a given reset actually left the RTC-backed
// clock intact (a real, recent epoch) or genuinely reset it (~0), instead
// of inferring it from RLOG_ID_CLOCK_SET's post-correction value or the
// plausibility check's own output. detail = the raw epoch seconds read.
#define RLOG_ID_CLOCK_RTC  57
// GEN_RLOG_NAMES_END
// 22, 26 retired -- subsumed by RLOG_ID_XPORT_LINK_WIFI_LISTENING.
// 24/25 never assigned.
// 58 was briefly RLOG_ID_USB_QUEUE_DROP (a standalone MON_DEBUG event for
// DebugLog's own USB retry-queue drops), replaced same-day by a plain
// DebugLog::_usb_queue_drop_count folded into link_tx_queue_full alongside
// BLE/WiFi's own send-queue-full counters (Beebo::appendLinkQueueDropEvents()/
// GET_MONRING header) -- one queue-full-drop reporting mechanism, not two.
// 57 was briefly a standalone RLOG_ID_CLOCK_SRC event 2026-09-12, replaced
// same-day by packing the value into RLOG_ID_CLOCK_SET's own record instead
// (see that event's comment and MonRing.h's DebugRecord._user) before ever
// shipping -- now reassigned to RLOG_ID_CLOCK_LIVE_AT_BOOT above.

// beebo: RLOG_ID_CLOCK_SET's DebugRecord._user[0] -- states *why* the
// epoch in `detail` is what it is, since that alone doesn't distinguish
// "this is just the live RTC that was never touched" from a genuine NVS
// restore/correction (the exact confusion a real user hit 2026-09-12
// watching a plain reflash's CLOCK_SET with no explanation). _user[1..2]
// (little-endian uint16) is the new epoch's own ms fraction (0-999) --
// `detail` (SECS) alone is only whole-second precision, same as RTCClock
// itself, so this is what lets a reader recover the full ms-precision
// timestamp actually anchored; 0 for
// a plain seconds-only correction. _user[3..4] is unused/reserved -- an
// earlier design carried the clock's own pre-correction value there so a
// reader could see the delta without cross-referencing the nearest
// preceding RLOG_ID_CLOCK_SYNC, but every real correction path logs that
// RLOG_ID_CLOCK_SYNC (with the exact ms-precision delta) immediately
// before its own RLOG_ID_CLOCK_SET, making the duplicate encoding
// redundant. _user[0] is one of:
#define RLOG_CLOCK_SRC_RTC      0   // live RTC counter survived the reset untouched (ESP_RST_UNKNOWN/SW, or a plausible-live ESP_RST_POWERON -- see kbase/CLOCK_DRIFT_COMPENSATION.md)
#define RLOG_CLOCK_SRC_NVM      1   // restored from the persisted rtc_ts (genuine cold-boot ESP_RST_POWERON, RLOG_ID_CLOCK_NVM_PULL logged alongside)
#define RLOG_CLOCK_SRC_FALLBACK 2   // no rtc_ts ever persisted -- fell back to the fixed placeholder date (RTC_FALLBACK_EPOCH)
#define RLOG_CLOCK_SRC_SYNC      3   // set by an explicit client command (CMD_SET_DEVICE_TIME, "time ", "clock sync")
#define RLOG_CLOCK_SRC_KEEPALIVE 4   // BEEBO_RAW_SUB_TIME_SYNC's session-less background keepalive (debug_link.py), not an explicit client request -- see RLOG_ID_CLOCK_SYNC's own comment

// beebo: RLOG_ID_BLE_HANDSHAKE sub-ids (detail bits 0-7) -- bits 8-31 are
// this sub-id's own payload, interpreted per step below. CONNECT has none
// (the GATT link coming up is the event); SECURITY_REQUEST/PASSKEY_REQUEST
// likewise (BLESecurityCallbacks -- PASSKEY_REQUEST only fires on a fresh,
// non-bonded pairing, absent on a silent bonded reconnect); AUTH_COMPLETE
// packs bit8=success, bits16-23=fail_reason (HCI code) if failed;
// CCCD_WRITE packs bit8=notifications now enabled -- the client's TX-
// characteristic "enable notifications" descriptor write, the step whose
// timing (or absence, right after a fresh pair) is what this group
// exists to trace; see checkRecvFrame()'s notify_ready gate.
#define RLOG_ID_BLE_HANDSHAKE_CONNECT           0   // onConnect() -- GATT link up
#define RLOG_ID_BLE_HANDSHAKE_SECURITY_REQUEST  1   // onSecurityRequest() -- central asked to elevate/pair
#define RLOG_ID_BLE_HANDSHAKE_PASSKEY_REQUEST   2   // onPassKeyRequest()
#define RLOG_ID_BLE_HANDSHAKE_AUTH_COMPLETE     3   // onAuthenticationComplete()
#define RLOG_ID_BLE_HANDSHAKE_CCCD_WRITE        4   // client wrote the TX characteristic's CCCD (0x2902)

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
#define DLOG_ID_BOOT_ROLE_NAME             9   // Beebo.cpp, right after RLOG_ID_BOOT_ROLE_BEGIN_DONE: this board/node's own identity at boot -- board.name (physical board alias, BeeboBoardPrefs.h's board_name), the live role's node.name (NodePrefs.h's node_name), and which role (companion/repeater) actually came up
// 10-99 reserved for future non-trace DLOGH/M/L call sites.
// beebo: BEEBO_USB_RXTX_TRACE (DualModeSerialInterface.cpp) opt-in trace ids.
#define DLOG_ID_USB_RX_TRACE               100   // one byte read off the wire, with the parser state it landed in
#define DLOG_ID_USB_TX_TRACE               101   // one writeFrame() call sending a frame out
#define DLOG_ID_USB_RX_DISCARD_STALE       102   // discardStaleRx(): bytes drained from a newly-(re)polled sub's stale hardware RX buffer
#define DLOG_ID_USB_RX_RESET_PARSER        103   // resetParserState(): mid-command parser state discarded at session end
#define DLOG_ID_USB_RX_BODY                105   // checkRecvFrame()'s MODE_FRAMED_BODY read progress
// beebo: converted from RLOGL 2026-09-11 (RLOGL/DEBUG_TLOG retired --
// MON_DEBUG can't represent Low severity, and DLOG's plain live-only push
// already covers exactly the same ground with no host-side decode table to
// keep in sync).
#define DLOG_ID_CMD_RECV                   110   // Beebo.cpp checkSerialInterface(): command received, message = "cmd=0x%04x" ((cmd<<8)|sub, hex)
#define DLOG_ID_CMD_DONE                   111   // Beebo.cpp checkSerialInterface(): command handler returned, same id encoding as DLOG_ID_CMD_RECV
#define DLOG_ID_CPU_SNAPSHOT               112   // Beebo.cpp: 1-minute busy/idle snapshot for a live `--debug`/`-d` session, message = "radio=N%% link=N%% idle=N%%"
#define DLOG_ID_WIFI_HEALTH                113   // Beebo.cpp: periodic low-level WiFi health sample, message = "heap=NKB rssi=NdBm ch=N"
#define DLOG_ID_BLE_HEALTH                 114   // SerialBLEInterface.cpp: periodic low-level BLE health sample, message = "heap=NKB"
// GEN_DLOG_NAMES_END

#ifndef DEBUG_LOG_VERBOSE
#define DEBUG_LOG_VERBOSE 0
#endif

class DebugLog {
  // beebo: bounded best-effort retry queue for pushToTargets()'s _usb
  // target -- writeFrameBestEffort()'s own availableForWrite() gate
  // (DualModeSerialInterface.cpp) can refuse a write outright when the
  // USB CDC TX ring (CONFIG_TINYUSB_CDC_TX_BUFSIZE=64 bytes on this core,
  // no Arduino-level API to raise it -- see that file's own comment) is
  // momentarily full under back-to-back live pushes, e.g. an rx MLOG
  // record immediately followed by the tx MLOG record it triggered.
  // Confirmed on real hardware 2026-09-18: the second push silently
  // vanished with no retry and no fault record, while unrelated tx
  // pushes elsewhere in the same session transmitted fine -- pure
  // buffer-timing, not a logic drop. This queue holds what didn't fit
  // the first time and retries it, one attempt per loop() tick
  // (retryUsbQueue(), driven from Beebo.cpp's checkSerialInterface()),
  // instead of losing it outright. Deliberately scoped to the _usb
  // target only -- the _serial (companion-session) target already skips
  // its write under isWriteBusy() (pushToTargets(), above) rather than
  // attempting and losing it, so it doesn't share this failure mode.
  //
  // Bounded on both axes: at most QUEUE_CAP frames pending (oldest-first
  // FIFO; once full, a new push is dropped rather than evicting the
  // oldest, so what's already queued keeps its place), and each frame
  // gets at most MAX_RETRY_TICKS attempts before being dropped. The
  // *tick* ceiling (MAX_RETRY_TICKS) is a real, permanent escape hatch,
  // not a capacity concern -- writeFrameBestEffort() also refuses
  // (returns 0) whenever the link's `_lastWasText` is true, a policy
  // refusal (don't inject binary framing after a legacy text-mode client
  // just spoke on the same physical link) this queue can't tell apart
  // from "buffer full" via the return value alone, so a frame that can
  // genuinely never be delivered must still eventually give up.
  //
  // The *depth* ceiling (QUEUE_CAP), by contrast, exists purely to keep
  // this a fixed-size embedded struct, not because backpressure should
  // ever actually reach it in practice: USB throughput is orders of
  // magnitude faster than the LoRa airtime that paces how quickly new
  // events can even be generated (an rx/tx/route/debug burst is bounded
  // by packets-per-second on the radio, not anything USB-side), so a
  // generous static depth here is, for any realistic traffic pattern,
  // equivalent to "never actually drains empty into overflow" -- real
  // backpressure (wait for the next tick, then write), not a small
  // shock absorber sized to be blown through under normal load. Only a
  // genuinely stuck peripheral (the still-open HWCDC TX-hang class,
  // plans/ARDUINO_ESP32_CORE_UPGRADE.md) or a policy-refused frame
  // sitting at the queue head blocking everything behind it should ever
  // realistically fill this.
  static const uint8_t USB_QUEUE_CAP = 64;
  static const uint16_t USB_QUEUE_MAX_RETRY_TICKS = 50;
  struct QueuedUsbFrame {
    uint8_t data[200];   // matches logLink()'s own out[200] cap -- the largest frame this class ever pushes
    uint8_t len = 0;
    uint16_t attempts = 0;
  };
  QueuedUsbFrame _usb_queue[USB_QUEUE_CAP];
  uint8_t _usb_queue_head = 0;
  uint8_t _usb_queue_count = 0;

  // beebo: lifetime count of frames lost because _usb_queue itself was
  // already full (or a frame too big for it ever arrived) -- the one drop
  // this class still can't paper over. Surfaced the same way BLE/WiFi's
  // own send_queue-full drops are (Beebo::appendLinkQueueDropEvents()/
  // GET_MONRING's link_tx_queue_full header field both fold this in
  // alongside ble_interface/wifi_interface's counters), not as an
  // independent MON_DEBUG event -- one queue-full-drop counter mechanism,
  // not two competing ones.
  uint32_t _usb_queue_drop_count = 0;

  void enqueueUsb(const uint8_t* out, size_t pos) {
    if (_usb_queue_count >= USB_QUEUE_CAP || pos > sizeof(QueuedUsbFrame::data)) {
      _usb_queue_drop_count++;
      return;
    }
    uint8_t tail = (uint8_t)((_usb_queue_head + _usb_queue_count) % USB_QUEUE_CAP);
    memcpy(_usb_queue[tail].data, out, pos);
    _usb_queue[tail].len = (uint8_t)pos;
    _usb_queue[tail].attempts = 0;
    _usb_queue_count++;
  }

  BaseSerialInterface* _serial = nullptr;
  BaseSerialInterface* _usb = nullptr;
  uint8_t _resp_code = 0;
  uint8_t _log_sub_id = 0;
  // beebo: enabling is tracked per requesting path, not one shared bool --
  // the raw session-less USB tap (BEEBO_RAW_SUB_DEBUG_LOG_ENABLE) and the
  // session-owning opcode (BEEBO_CMD_DEBUG_LOG_ENABLE, arriving over
  // whichever transport currently holds the companion session) are
  // independent clients that can each be live or not. logLink() routes to
  // _usb only when _usb_enabled and to _serial only when _session_enabled,
  // so a `beebo -d` session tapping raw USB never also gets mirrored onto
  // an unrelated BLE/WiFi companion session (and vice versa) -- see that
  // function's own comment.
  bool _usb_enabled = false;
  bool _session_enabled = false;

  // MLOG (live MonRing relay) is enabled
  // independently of DLOG above -- same per-path (raw USB tap vs.
  // session) split, same reasoning (a raw USB tap must never also land on
  // an unrelated BLE/WiFi companion session, see attach()'s own comment).
  bool _usb_mlog_enabled = false;
  bool _session_mlog_enabled = false;
  uint8_t _mlog_sub_id = 0;

  // Forward of every H/M-severity RLOG event to MonRing's MON_DEBUG
  // kind -- a plain function
  // pointer, not a hardcoded MonRing/Beebo reference, so this
  // shared-location file (fw/src/helpers/) stays decoupled from
  // multi_role's own Beebo::monring (private, and not every board target
  // that could include this file necessarily has one) -- same "caller
  // supplies a callback" shape as MonRing::LiveSink. nullptr by default, so
  // nothing changes for any build that never calls setDebugSink() -- but
  // MON_DEBUG is now the *only* place an RLOG event goes (this file no
  // longer keeps its own ring/replay for it), so a build that never wires
  // this sink loses RLOG entirely, not just its MonRing mirror.
  // beebo: `user` is always a valid pointer to 5 bytes (logRing() zero-fills
  // its own local array when the caller passes none), never nullptr --
  // simpler for every DebugSink implementation than also handling a null
  // case.
  using DebugSink = void (*)(uint8_t type, uint8_t severity, int32_t detail, uint32_t ms,
                             const char* file, int line, const uint8_t user[5]);
  DebugSink _debug_sink = nullptr;

public:
  // beebo: `serial` is the MultiSerialInterface aggregator (Beebo::_serial/
  // serial_interface) -- its writeFrame()/writeFrameBestEffort() forward to
  // whichever sub-transport (BLE/WiFi-TCP/USB) currently holds the companion
  // session, reached only while a companion session itself enabled the
  // stream (setSessionEnabled(), the TCP_DEBUG_STREAM capability -- e.g.
  // gatto, which has no USB at all). `usb` is the fixed physical USB
  // interface (Beebo's usb_interface), reached only while the session-less
  // raw USB debug tap enabled it (setUsbEnabled(), BEEBO_RAW_SUB_DEBUG_LOG_ENABLE,
  // checkSerialInterface()) -- see pushToTargets() for the routing. Each
  // path pushes only to the target that actually asked for it, so a raw USB
  // tap's stream never also gets mirrored onto an unrelated live BLE/WiFi
  // companion session, which would otherwise compete with
  // that session's own command-reply traffic for the same shallow
  // send_queue and drop real app frames.
  void attach(BaseSerialInterface* serial, BaseSerialInterface* usb, uint8_t resp_code,
              uint8_t log_sub_id, uint8_t mlog_sub_id = 0) {
    _usb = usb;
    _serial = serial;
    _resp_code = resp_code;
    _log_sub_id = log_sub_id;
    _mlog_sub_id = mlog_sub_id;
  }
  void setUsbEnabled(bool enabled) { _usb_enabled = enabled; }
  void setSessionEnabled(bool enabled) { _session_enabled = enabled; }
  bool isUsbEnabled() const { return _usb_enabled; }
  bool isSessionEnabled() const { return _session_enabled; }
  bool isEnabled() const { return _usb_enabled || _session_enabled; }

  // MLOG -- independent DLOG/RLOG vs
  // MLOG enable state per path, same reasoning as _usb_enabled/
  // _session_enabled above.
  void setUsbMlogEnabled(bool enabled) { _usb_mlog_enabled = enabled; }
  void setSessionMlogEnabled(bool enabled) { _session_mlog_enabled = enabled; }
  bool isUsbMlogEnabled() const { return _usb_mlog_enabled; }
  bool isSessionMlogEnabled() const { return _session_mlog_enabled; }
  bool isMlogEnabled() const { return _usb_mlog_enabled || _session_mlog_enabled; }

  // RLOGH/M: forwarded to MonRing's MON_DEBUG kind via _debug_sink (see that
  // member's own comment) -- the sole delivery, live and for post-mortem
  // fetch alike (GET_MONRING). No ring/replay/offline-fetch of its own
  // anymore (retired, along with RLOGL -- see this file's own top comment).
  // beebo: `user`, when non-null, points at 5 bytes copied verbatim into
  // MonRing's DebugRecord._user -- meaning defined per RLOG_ID_* (see that
  // field's own comment), not a general-purpose payload. Almost every
  // call site passes none (defaults to all-zero); RLOG_ID_CLOCK_SET is the
  // first to use it.
  void logRing(const char* file, int line, uint8_t type, uint8_t severity, int32_t detail = 0,
               const uint8_t* user = nullptr);
  void setDebugSink(DebugSink sink) { _debug_sink = sink; }

  // DLOGH/M/L: never persisted anywhere (fixed-size MonRing records can't
  // hold arbitrary text) -- only live-pushed to physical USB *and* whichever
  // transport (if any, if not USB) holds the session (RESP_CODE_BEEBO/
  // DEBUG_LOG frame) whenever the debug link is enabled -- see attach()'s
  // own comment.
  void logLink(const char* file, int line, uint16_t id, uint8_t severity, const char* fmt, ...) __attribute__((format(printf, 6, 7)));

private:
  // beebo: logLink()'s (DLOG's) live DEBUG_LOG frame header --
  // [anchor_epoch_sec:4][anchor_millis:4][anchor_ms_frac:2][file_len:1][file],
  // 22 bytes fixed + file_len bytes. anchor_epoch_sec/anchor_millis/
  // anchor_ms_frac are the *current* shared time anchor (MonRing.h's
  // g_time_anchor_epoch_sec/_millis/_ms_frac, set by
  // Beebo::startMonRing()/an explicit RTC correction) -- carried on every
  // DLOG frame so a client can resolve `millis` to absolute wall-clock time
  // directly, with no dependency on MonRing's own wire protocol (MLOG)
  // being enabled at all. 0/0 (epoch_sec/millis) means the anchor isn't
  // known yet (real epoch is never 0 in practice) -- a client falls back to
  // boot-relative duration in that case. anchor_ms_frac is the sub-second
  // component of anchor_epoch_sec from a millisecond-precision clock
  // sync; 0 when the anchor is still a
  // plain seconds-only boot-time/RTC capture.
  static size_t writeHeader(uint8_t* out, size_t cap, size_t avail_after,
                             uint8_t resp_code, uint8_t sub_id, uint16_t id,
                             uint8_t severity, int line, const char* file,
                             uint32_t ms) {
    if (cap < 22) return 0;
    out[0] = resp_code;
    out[1] = sub_id;
    memcpy(&out[2], &ms, 4);
    memcpy(&out[6], &id, 2);
    out[8] = severity;
    uint16_t line16 = (uint16_t)line;
    memcpy(&out[9], &line16, 2);
    memcpy(&out[11], &g_time_anchor_epoch_sec, 4);
    memcpy(&out[15], &g_time_anchor_millis, 4);
    memcpy(&out[19], &g_time_anchor_ms_frac, 2);

    const char* base = strrchr(file, '/');
    base = base ? base + 1 : file;
    size_t base_len = strlen(base);
    size_t room = (cap >= 22 + avail_after) ? cap - 22 - avail_after : 0;
    if (base_len > room) base_len = room;
    if (base_len > 255) base_len = 255;

    out[21] = (uint8_t)base_len;
    memcpy(&out[22], base, base_len);
    return 22 + base_len;
  }

  // beebo: routes one already-serialized frame to whichever target(s)
  // actually asked for the live stream -- _usb only if the raw USB tap
  // enabled it (BEEBO_RAW_SUB_DEBUG_LOG_ENABLE), _serial only if a
  // companion session enabled it (BEEBO_CMD_DEBUG_LOG_ENABLE), skipping
  // _serial when its locked transport is USB and usb_enabled already
  // covers the same physical wire (no double send). A companion session's
  // stream is best-effort against that session's own traffic -- skipped
  // outright (not queued) while its transport isWriteBusy(), so a burst of
  // debug events never takes the last send_queue slot a real app command
  // reply needs (see SerialBLEInterface::isWriteBusy()'s own comment on
  // why that slot matters); the raw USB tap has no such shared traffic to
  // protect, so it always pushes when armed. Takes explicit usb_enabled/
  // session_enabled flags rather than reading _usb_enabled/_session_enabled
  // directly -- DLOG/RLOG and MLOG are enabled
  // independently, each with their own pair of flags, but route through
  // this same targeting logic.
  void pushToTargets(const uint8_t* out, size_t pos, bool usb_enabled, bool session_enabled) const {
    if (usb_enabled && _usb) {
      DebugLog* self = const_cast<DebugLog*>(this);
      // beebo: same isWriteBusy()-gated-before-attempting methodology
      // checkSerialInterface()'s own paced chain already uses for every
      // other USB write (contacts/neighbors/advert-path/stats/monring
      // streaming, MLOG replay) -- skip the attempt outright when busy
      // instead of calling writeFrameBestEffort() only to have it fail
      // the identical availableForWrite() check internally. Also never
      // attempt a direct write while anything is already queued: writing
      // straight through here would let this frame reach the wire ahead
      // of an older one still waiting in _usb_queue, reordering the
      // stream -- queueing behind it instead preserves FIFO order (see
      // _usb_queue's own comment for why that queue exists at all).
      if (self->_usb_queue_count > 0 || self->_usb->isWriteBusy()) {
        self->enqueueUsb(out, pos);
      } else {
        size_t sent = self->_usb->writeFrameBestEffort(out, pos);
        if (sent < pos) self->enqueueUsb(out, pos);
      }
    }
    if (session_enabled && _serial) {
      BaseSerialInterface* serial = const_cast<DebugLog*>(this)->_serial;
      uint8_t active_type = serial->activeTransportType();
      bool same_wire_as_usb = usb_enabled && active_type == RLOG_ID_XPORT_USB;
      if (active_type != 0 && !same_wire_as_usb && !serial->isWriteBusy()) {
        serial->writeFrameBestEffort(out, pos);
      }
    }
  }

public:
  // beebo: queued-retry best-effort USB write, for callers outside this
  // class's own pushToTargets() (e.g. Beebo.cpp's session-less
  // BEEBO_RESP_RAW_ACK replies) that want the same "never silently drop
  // under momentary TX backpressure" protection pushMlogFrame() already
  // gets -- see _usb_queue's own comment for why a bare
  // writeFrameBestEffort() call isn't safe on this hardware. No-op if no
  // USB target is attached.
  void writeUsbQueued(const uint8_t* out, size_t pos) {
    if (!_usb) return;
    // beebo: same busy-gated-before-attempting, queue-preserves-order
    // methodology as pushToTargets()'s _usb branch -- see its own
    // comment for why both checks matter.
    if (_usb_queue_count > 0 || _usb->isWriteBusy()) {
      enqueueUsb(out, pos);
      return;
    }
    size_t sent = _usb->writeFrameBestEffort(out, pos);
    if (sent < pos) enqueueUsb(out, pos);
  }

  // MLOG live push -- wired as
  // MonRing's LiveSink, so it fires from MonRing::_store() for every record
  // actually appended (and, during a disabled->enabled replay, once per
  // record already resident in the ring). Wire frame: [resp_code:1]
  // [mlog_sub_id:1][MonRecord:16 bytes, verbatim] -- the same 16 bytes
  // GET_MONRING's page download already uses, so the host reuses that
  // decoder rather than a second one (design decision 2).
  void pushMlogFrame(const MonRecord &rec) const {
    if (!isMlogEnabled() || (!_serial && !_usb)) return;
    uint8_t out[2 + sizeof(MonRecord)];
    out[0] = _resp_code;
    out[1] = _mlog_sub_id;
    memcpy(&out[2], &rec, sizeof(MonRecord));
    pushToTargets(out, sizeof(out), _usb_mlog_enabled, _session_mlog_enabled);
  }

  // beebo: drains at most one queued _usb retry frame per call, oldest
  // first -- call once per loop() tick (Beebo.cpp's checkSerialInterface(),
  // unconditionally, since this is independent of that function's own
  // companion-session pacing). See _usb_queue's own comment for why this
  // exists and why it's bounded. No-op when the queue is empty or no USB
  // target is attached.
  void retryUsbQueue() {
    if (_usb_queue_count == 0 || !_usb) return;
    QueuedUsbFrame& f = _usb_queue[_usb_queue_head];
    size_t sent = _usb->writeFrameBestEffort(f.data, f.len);
    if (sent >= f.len) {
      _usb_queue_head = (uint8_t)((_usb_queue_head + 1) % USB_QUEUE_CAP);
      _usb_queue_count--;
    } else if (++f.attempts >= USB_QUEUE_MAX_RETRY_TICKS) {
      // beebo: genuinely gave up (see USB_QUEUE_CAP's own comment on
      // MAX_RETRY_TICKS being a real, permanent escape hatch, not a
      // capacity concern) -- counts as a drop the same as the queue
      // being full outright.
      _usb_queue_drop_count++;
      _usb_queue_head = (uint8_t)((_usb_queue_head + 1) % USB_QUEUE_CAP);
      _usb_queue_count--;
    }
  }

  // beebo: lifetime count of frames this class has dropped via
  // enqueueUsb()/retryUsbQueue() -- see _usb_queue_drop_count's own
  // comment. Read fresh (not mirrored into MonRing) by
  // Beebo::appendLinkQueueDropEvents() and the GET_MONRING header, folded
  // into link_tx_queue_full alongside ble_interface/wifi_interface's own
  // send-queue-full counters.
  uint32_t getUsbQueueDropCount() const { return _usb_queue_drop_count; }
};

extern DebugLog debug_log;

// beebo: RLOGH/DLOGH are always compiled in; RLOGM/L and DLOGM/L only
// exist in a DEBUG_LOG_VERBOSE build -- in a normal build those call sites
// vanish entirely (not even evaluated), so a hot/high-volume Low-severity
// site (RLOG_ID_WIFI_HEALTH, RLOG_ID_BLE_HEALTH) costs nothing by default.
#define RLOGH(type, ...) debug_log.logRing(__FILE__, __LINE__, type, DLOG_SEV_H, ##__VA_ARGS__)
#define DLOGH(id, fmt, ...) debug_log.logLink(__FILE__, __LINE__, id, DLOG_SEV_H, fmt, ##__VA_ARGS__)

#if DEBUG_LOG_VERBOSE
#define RLOGM(type, ...) debug_log.logRing(__FILE__, __LINE__, type, DLOG_SEV_M, ##__VA_ARGS__)
#define DLOGM(id, fmt, ...) debug_log.logLink(__FILE__, __LINE__, id, DLOG_SEV_M, fmt, ##__VA_ARGS__)
#define DLOGL(id, fmt, ...) debug_log.logLink(__FILE__, __LINE__, id, DLOG_SEV_L, fmt, ##__VA_ARGS__)
#else
#define RLOGM(type, ...) do {} while (0)
#define DLOGM(id, fmt, ...) do {} while (0)
#define DLOGL(id, fmt, ...) do {} while (0)
#endif

// beebo: BEEBO_RAW_SUB_DBG_ENABLE=1 lives here (not generated from
// protocol.yaml, since this raw-marker layer is deliberately distinct from
// -- and below -- the CMD_BEEBO/sub_id wire format that generator covers).
// sub_id 2 (formerly BEEBO_RAW_SUB_KEEPALIVE) retired 2026-09-13 -- never
// actually sent by the CLI (dead since introduction); BEEBO_RAW_SUB_TIME_SYNC
// already refreshes USB liveness identically (DualModeSerialInterface's own
// per-sub-id liveness check) while also doing real clock-sync work, so it
// fully subsumed this sub-id's only purpose. Never reassign sub_id 2.
// BEEBO_RAW_SUB_TIME_SYNC=3 carries a fixed 6-byte payload -- a 4-byte
// little-endian epoch-seconds value plus a 2-byte little-endian ms fraction
// (unlike BEEBO_RAW_SUB_DBG_ENABLE's single data byte -- see
// DualModeSerialInterface's per-sub-id payload length); the --debug link
// sends it periodically to keep the device clock corrected without a full
// authenticated CMD_SET_DEVICE_TIME session (see Beebo::applyClockSync()'s
// forward-only-correction rule, same as CMD_SET_DEVICE_TIME), doubling as
// this link's own liveness keepalive. The ms fraction matters here more
// than most callers: this keepalive is the only clock touch a long
// --debug session gets between connects (hourly).
#define BEEBO_RAW_SUB_DBG_ENABLE 1
#define BEEBO_RAW_SUB_TIME_SYNC 3

// DEBUG_LOG_ENABLE/BEEBO_RAW_SUB_DBG_ENABLE's payload byte is a
// bitmask, not a bare bool -- bit 0 is the
// original DLOG/RLOG enable (an old client sending bare 0/1 is unaffected),
// bit 1 is the new MLOG enable.
#define DEBUG_LOG_ENABLE_BIT_DLOG 0x01
#define DEBUG_LOG_ENABLE_BIT_MLOG 0x02
// beebo: suppresses MLOG's replay-on-enable backlog walk (MonRing::
// requestMlogReplay()) without touching live MLOG delivery -- for a
// session that only wants to watch from here on, not print the ring's
// existing history at connect time. Kept in sync by hand with
// _usb_raw.py's own DEBUG_LOG_ENABLE_BIT_NO_REPLAY.
#define DEBUG_LOG_ENABLE_BIT_NO_REPLAY 0x04
