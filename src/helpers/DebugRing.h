#pragma once

#include <stdint.h>
#include <string.h>
#include "BaseSerialInterface.h"

// beebo: unified debug-event subsystem -- one ring (this file, `debug_ring`,
// was `TransportLog`/`transport_log`) and one live push mechanism (was
// the separate DebugLog.h/.cpp, folded in here) behind two macro families:
//
// - RLOGH/M/L(type, detail) -- a structured (type, detail) event, always
//   live-pushed over physical USB *and* over whichever transport currently
//   holds the companion session, if any and if not USB itself (see
//   attach()'s own comment) whenever the debug link is enabled, and also
//   appended to the ring *unless* severity is Low -- L is link-only, same
//   as DLOGH/M/L below, never occupies a ring slot. `type`
//   is a hand-picked, sometimes-reused-on-purpose per-call-site tag (e.g.
//   RLOG_ID_XSESSION_INIT/_CHANGE is logged from a different file than every
//   other RLOG_ID_XPORT_*/RLOG_ID_XLINK_* id) that doubles as the live
//   frame's id.
// - DLOGH/M/L(id, fmt, ...) -- a free-text printf-style event, live-pushed
//   the same way whenever the debug link is enabled, but never stored in
//   the ring (which only ever holds fixed-size structured records).
//
// Severity is a compile-time gate, not just a runtime label: H is always
// compiled in; M/L only exist when DEBUG_LOG_VERBOSE is defined (1) for the
// build, otherwise every RLOGM/L/DLOGM/L call site compiles to nothing at
// all -- e.g. RLOG_ID_WIFI_HEALTH/RLOG_ID_BLE_HEALTH (both Low) cost nothing in a
// regular build. Both families are unconditionally live-pushed while
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
// beebo: BLE session-FSM bring-up sequence, one grouped id instead of a
// separate top-level RLOG_ID per step (added 2026-09-09 diagnosing a
// "Failed to fetch device info" timeout that only reproduced on the first
// connection after a fresh pair -- see BUGS.md and checkRecvFrame()'s
// notify_ready gate) -- detail bits 0-7 = sub-id (RLOG_ID_BLE_HANDSHAKE_*
// below), bits 8-31 = sub-id-specific payload, same shape as
// RLOG_ID_XPORT_INIT/_CHANGE's var-id packing further below. Teardown
// (RLOG_ID_BLE_DISCONNECT) stays its own top-level id -- this group is
// bring-up only.
#define RLOG_ID_BLE_HANDSHAKE         16
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
// (same millis tick) right alongside RLOG_ID_BLE_HANDSHAKE's CONNECT step: HI carries the first
// 2 bytes (bda[0..1]) packed MSB-first in the low 16 bits, LO carries the
// last 4 bytes (bda[2..5]) packed MSB-first, matching RLOG_ID_WIFI_STA_GOT_IP's
// own octet packing.
#define RLOG_ID_BLE_CLIENT_ADDR_HI    27
#define RLOG_ID_BLE_CLIENT_ADDR_LO    28
// beebo: periodic low-level BLE health sample, mirroring RLOG_ID_WIFI_HEALTH --
// triggered every BLE_HEALTH_SAMPLE_MS (loopTransports() ->
// SerialBLEInterface::requestHealthSample()), but gated on deviceConnected
// (a central actually connected), unlike RLOG_ID_WIFI_HEALTH's _wifi_up
// (not a live app session) -- BLE's own heap number never moves while
// just advertising with nobody connected, and rssi is unavailable until
// then anyway, so a sample logged in that state would be pure noise. Low
// severity, same reasoning as RLOG_ID_WIFI_HEALTH.
// detail's rssi field is still always 127 (SerialBLEInterface's
// BLE_RSSI_UNAVAILABLE) -- the real value only shows up separately, via
// RLOG_ID_BLE_RSSI_COMPLETE below, once its own async read completes; see
// RLOG_ID_BLE_RSSI_REQUESTED's own comment for why this event doesn't
// just read RSSI synchronously into the same detail the way WiFi does.
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
// beebo: 1-minute busy/idle snapshot for a live `--debug`/`-d` session
// (visual-only, coarse) -- see kbase/CPU_UTILIZATION.md. detail packs
// three unsigned 0-100 percentages (rescaled down from the live ~1s
// 0-10000 tier) into the low 3 bytes (no sign-extension needed, unlike
// RLOG_ID_RADIO_RECV_ERROR's signed dB values): bits 0-7 = exec_pct
// (rx_busy+tx_busy, clamped at 100), bits 8-15 = lx_busy, bits 16-23 =
// idle_pct. RLOGL (DLOG_SEV_L) only -- never persisted
// to the in-RAM debug ring retrievable later via GET_STATS/
// STATS_TYPE_TRANSPORT, genuinely live-only like the values it reports.
#define RLOG_ID_CPU_SNAPSHOT          48
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
// beebo: clock-drift compensation tracing (see kbase/CLOCK_DRIFT_COMPENSATION.md)
// -- added to pin down a discrepancy between the offset recorded right
// before a flash and the offset actually applied after it, rather than
// inferring both from before/after `beebo clock` readings. detail =
// the offset in seconds (curr - secs, always positive -- see
// beebo_recordClockDrift()'s own comment for why only an ahead-drift is
// ever recorded).
#define RLOG_ID_CLOCK_DRIFT_RECORDED  51
// beebo: logged from applyDriftOffset_() every time it actually subtracts
// (i.e. has_offset was true and the device_now_t > drift_offset guard
// passed) -- detail = the drift_off value it read from NVS and applied,
// so it can be compared directly against the nearest preceding
// RLOG_ID_CLOCK_DRIFT_RECORDED's detail across a reset.
#define RLOG_ID_CLOCK_DRIFT_APPLIED   52
// beebo: boot-time sub-checkpoints inside Beebo::begin(), between
// RLOG_ID_BOOT_STORAGE_READY and RLOG_ID_BOOT_TRANSPORTS_READY -- added to
// break down a 2026-09-09 report of a multi-second gap in that span
// (SPIFFS reads via loadRoleState() were the leading suspect, but never
// measured directly). detail = millis() at each point, same convention as
// the other RLOG_ID_BOOT_* events.
#define RLOG_ID_BOOT_ROLE_STATE_LOADED 53   // both loadRoleState() calls done
#define RLOG_ID_BOOT_ROLE_BEGIN_DONE   54   // beginCompanion()/beginRepeater() done
// GEN_RLOG_NAMES_END
// 55, 56, 57 retired 2026-09-09 -- folded into RLOG_ID_BLE_HANDSHAKE above.
// 22, 26 retired -- subsumed by RLOG_ID_XPORT_LINK_WIFI_LISTENING.
// 24/25 never assigned.

// beebo: RLOG_ID_BLE_HANDSHAKE sub-ids (detail bits 0-7) -- bits 8-31 are
// this sub-id's own payload, interpreted per step below. CONNECT has none
// (the GATT link coming up is the event); SECURITY_REQUEST/PASSKEY_REQUEST
// likewise (BLESecurityCallbacks -- PASSKEY_REQUEST only fires on a fresh,
// non-bonded pairing, absent on a silent bonded reconnect); AUTH_COMPLETE
// packs bit8=success, bits16-23=fail_reason (HCI code) if failed;
// CCCD_WRITE packs bit8=notifications now enabled -- the client's TX-
// characteristic "enable notifications" descriptor write, the step whose
// timing (or absence, right after a fresh pair) was the actual root cause
// behind the "Failed to fetch device info" investigation this group was
// added for; see checkRecvFrame()'s notify_ready gate and BUGS.md.
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
// 9-99 reserved for future non-trace DLOGH/M/L call sites.
// beebo: BEEBO_USB_RXTX_TRACE (DualModeSerialInterface.cpp) opt-in trace ids.
#define DLOG_ID_USB_RX_TRACE               100   // one byte read off the wire, with the parser state it landed in
#define DLOG_ID_USB_TX_TRACE               101   // one writeFrame() call sending a frame out
#define DLOG_ID_USB_RX_DISCARD_STALE       102   // discardStaleRx(): bytes drained from a newly-(re)polled sub's stale hardware RX buffer
#define DLOG_ID_USB_RX_RESET_PARSER        103   // resetParserState(): mid-command parser state discarded at session end
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
  BaseSerialInterface* _usb = nullptr;
  uint8_t _resp_code = 0;
  uint8_t _log_sub_id = 0;
  uint8_t _rlog_sub_id = 0;
  // beebo: enabling is tracked per requesting path, not one shared bool --
  // the raw session-less USB tap (BEEBO_RAW_SUB_DEBUG_LOG_ENABLE) and the
  // session-owning opcode (BEEBO_CMD_DEBUG_LOG_ENABLE, arriving over
  // whichever transport currently holds the companion session) are
  // independent clients that can each be live or not. pushRlogFrame()/
  // logLink() route to _usb only when _usb_enabled and to _serial only
  // when _session_enabled, so a `beebo -d` session tapping raw USB never
  // also gets mirrored onto an unrelated BLE/WiFi companion session (and
  // vice versa) -- see those functions' own comments.
  bool _usb_enabled = false;
  bool _session_enabled = false;

  bool _replay_active = false;
  uint16_t _replay_pos = 0;   // 0.._count-1, logical index of the next event replayStep() will push

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
  // companion session (found 2026-09-09 doing exactly that, competing with
  // that session's own command-reply traffic for the same shallow
  // send_queue and dropping real app frames -- see BUGS.md).
  void attach(BaseSerialInterface* serial, BaseSerialInterface* usb, uint8_t resp_code,
              uint8_t log_sub_id, uint8_t rlog_sub_id) {
    _usb = usb;
    _serial = serial;
    _resp_code = resp_code;
    _log_sub_id = log_sub_id;
    _rlog_sub_id = rlog_sub_id;
  }
  void setUsbEnabled(bool enabled) { _usb_enabled = enabled; }
  void setSessionEnabled(bool enabled) { _session_enabled = enabled; }
  bool isUsbEnabled() const { return _usb_enabled; }
  bool isSessionEnabled() const { return _session_enabled; }
  bool isEnabled() const { return _usb_enabled || _session_enabled; }

  // RLOGH/M/L: appended to the ring unless severity is Low (L is link-only,
  // like DLOGH/M/L); always live-pushed to physical USB *and* whichever
  // transport (if any, if not USB) holds the session (RESP_CODE_BEEBO/
  // DEBUG_TLOG frame) whenever the debug link is enabled, regardless of
  // severity -- see attach()'s own comment.
  void logRing(const char* file, int line, uint8_t type, uint8_t severity, int32_t detail = 0);

  // DLOGH/M/L: never touches the ring (fixed-size records can't hold
  // arbitrary text) -- only live-pushed to physical USB *and* whichever
  // transport (if any, if not USB) holds the session (RESP_CODE_BEEBO/
  // DEBUG_LOG frame) whenever the debug link is enabled -- see attach()'s
  // own comment.
  void logLink(const char* file, int line, uint16_t id, uint8_t severity, const char* fmt, ...) __attribute__((format(printf, 6, 7)));

  uint16_t count() const { return _count; }

  // beebo: re-emits every event currently in the ring as a live DEBUG_TLOG
  // push, oldest first -- armed once from Beebo::checkSerialInterface()'s
  // DEBUG_LOG_ENABLE handling (both the raw sub-frame USB path and the
  // session-owning BEEBO_CMD_DEBUG_LOG_ENABLE opcode), on the transition
  // into enabled, then drained one event per loop() tick from
  // checkSerialInterface()'s own paced-stream chain -- the same
  // !_serial->isWriteBusy()-gated pattern GET_NEIGHBORS/GET_MONRING/the
  // contacts iterator already use. This is how a boot-time event
  // (RLOG_ID_BOOT_START, or anything else logged before a client ever
  // attached -- see writeFrameBestEffort()'s own no-ring-buffer comment in
  // DualModeSerialInterface.cpp for why a *live* push that early is simply
  // lost) still reaches a `--debug` host: nothing needs to reach the wire
  // before the host is listening, since the ring already held it and this
  // walks it again once the host actually can receive it.
  //
  // Paced deliberately, not a single synchronous burst -- confirmed on real
  // hardware (`gatto`, 2026-09-07) that a tight unpaced loop calling
  // writeFrameBestEffort() for every ring event overflows
  // SerialWifiInterface's own send_queue (only FRAME_QUEUE_SIZE slots deep)
  // well before a full boot sequence's worth of events got out -- the
  // replay silently stopped after exactly as many events as the queue could
  // hold, with no indication anything had been dropped. USB doesn't hit
  // this (DualModeSerialInterface's writeFrameBestEffort() is a cheap,
  // genuinely non-blocking drop, no shallow intermediate queue involved),
  // but the ring can hold up to RLOG_MAX_EVENTS regardless of transport, so
  // pacing applies uniformly rather than only over TCP/BLE.
  bool isReplaying() const { return _replay_active; }

  void beginReplay() {
    _replay_active = (_count > 0);
    _replay_pos = 0;
  }

  // Push one more ring event (oldest-first) and advance the cursor.
  // Returns true while replay is still in progress (call again next tick,
  // once the caller's own !_serial->isWriteBusy() gate opens again), false
  // once done (nothing pushed this call).
  bool replayStep() {
    if (!_replay_active) return false;
    if (_replay_pos >= _count) {
      _replay_active = false;
      return false;
    }
    uint16_t logical_start = (_count < RLOG_MAX_EVENTS) ? 0 : _head;
    uint16_t idx = (logical_start + _replay_pos) % RLOG_MAX_EVENTS;
    pushRlogFrame(_buf[idx].file, _buf[idx].line, _buf[idx].type, _buf[idx].severity, _buf[idx].detail, _buf[idx].millis);
    _replay_pos++;
    if (_replay_pos >= _count) _replay_active = false;
    return _replay_active;
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

  // beebo: routes one already-serialized frame to whichever target(s)
  // actually asked for the live stream -- _usb only if the raw USB tap
  // enabled it (BEEBO_RAW_SUB_DEBUG_LOG_ENABLE), _serial only if a
  // companion session enabled it (BEEBO_CMD_DEBUG_LOG_ENABLE), skipping
  // _serial when its locked transport is USB and _usb_enabled already
  // covers the same physical wire (no double send). A companion session's
  // stream is best-effort against that session's own traffic -- skipped
  // outright (not queued) while its transport isWriteBusy(), so a burst of
  // debug events never takes the last send_queue slot a real app command
  // reply needs (see SerialBLEInterface::isWriteBusy()'s own comment on
  // why that slot matters); the raw USB tap has no such shared traffic to
  // protect, so it always pushes when armed.
  void pushToTargets(const uint8_t* out, size_t pos) const {
    if (_usb_enabled && _usb) {
      const_cast<DebugRing*>(this)->_usb->writeFrameBestEffort(out, pos);
    }
    if (_session_enabled && _serial) {
      BaseSerialInterface* serial = const_cast<DebugRing*>(this)->_serial;
      uint8_t active_type = serial->activeTransportType();
      bool same_wire_as_usb = _usb_enabled && active_type == RLOG_ID_XPORT_USB;
      if (active_type != 0 && !same_wire_as_usb && !serial->isWriteBusy()) {
        serial->writeFrameBestEffort(out, pos);
      }
    }
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
    if (!isEnabled() || (!_serial && !_usb)) return;

    uint8_t out[64];   // plenty for header + a file basename + the 4-byte detail
    size_t pos = writeHeader(out, sizeof(out), 4, _resp_code, _rlog_sub_id, id, severity, line, file, ms);
    if (pos == 0) return;

    memcpy(&out[pos], &detail, 4);
    pos += 4;

    pushToTargets(out, pos);
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
