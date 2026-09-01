#pragma once

#include <stdint.h>
#include "BaseSerialInterface.h"

// beebo: live, unsolicited debug-event push -- the DEBUG_LOG(fmt, ...)
// drop-in replacement for an ad-hoc Serial.printf(), plus a live mirror of
// every TransportLog event (see logEvent(), called from TransportLog::log()
// itself), delivered as properly framed companion-protocol events
// (RESP_CODE_BEEBO/DEBUG_LOG and RESP_CODE_BEEBO/DEBUG_TLOG, see
// protocol.yaml) always over USB (Beebo::usb_interface), independent of
// whichever transport (if any) currently holds the companion session --
// instead of raw text that collides with binary protocol bytes on shared
// USB (see kbase/DEBUGGING.md's USB/Serial-sharing constraint). No ring
// buffer for either kind -- each call either pushes immediately (while
// enabled and USB has something connected) or costs one cheap check
// (while not), same enable-once-then-stream shape as MonRing's own capture
// pump. Enabled/disabled two ways: BEEBO_CMD_DEBUG_LOG_ENABLE (`beebo
// --debug`, a real CMD_BEEBO command over whichever transport already
// holds the companion session), or DualModeSerialInterface::RAW_MARKER +
// BEEBO_RAW_SUB_DEBUG_LOG_ENABLE (`beebo dbglog`'s standalone raw write,
// see that class's own pollRawControl() comment for why it bypasses the
// companion-session layer entirely rather than reusing the CMD_BEEBO path).
// Not tied to any one session's lifetime -- stays enabled until explicitly
// disabled or the device reboots.
//
// A global singleton, same pattern as TransportLog.h's `transport_log` --
// needed since DEBUG_LOG() must be callable from any shared file (e.g.
// SerialWifiInterface.cpp), not just Beebo.cpp, without depending on the
// generated, multi_role-only BeeboProtocol.h. Beebo::begin() calls attach()
// once with usb_interface and the generated frame-prefix bytes for both
// response shapes so this file stays free of any beebo-protocol-specific
// constant.
// beebo: sub_id for the raw-marker control frame described above -- lives
// here, not generated from protocol.yaml, since this is a distinct, lower
// wire layer than CMD_BEEBO's sub-id space (protocol.yaml's PER_ROLE
// generator is specific to that layer) and beebo dbglog is currently the
// only raw-marker consumer. Room for more sub_ids here if that grows, per
// this file's own attach()/setEnabled() comment above.
#define BEEBO_RAW_SUB_DEBUG_LOG_ENABLE 1

class DebugLog {
public:
  void attach(BaseSerialInterface* serial, uint8_t resp_code,
              uint8_t log_sub_id, uint8_t tlog_sub_id) {
    _serial = serial;
    _resp_code = resp_code;
    _log_sub_id = log_sub_id;
    _tlog_sub_id = tlog_sub_id;
  }
  void setEnabled(bool enabled) { _enabled = enabled; }
  bool isEnabled() const { return _enabled; }

  // file is always __FILE__ (a full build-path); only the basename is sent,
  // to leave more of the frame budget for the actual message.
  void logf(const char* file, int line, const char* fmt, ...) __attribute__((format(printf, 4, 5)));

  // Live mirror of a TransportLog event -- same (type, detail) TransportLog
  // itself just recorded into its own ring, so the CLI can decode it with
  // the exact same TLOG_NAMES/_detail_str() table `beebo monitor transport`
  // already uses, no firmware-side text formatting needed.
  void logEvent(uint8_t type, int32_t detail);

private:
  BaseSerialInterface* _serial = nullptr;
  uint8_t _resp_code = 0;
  uint8_t _log_sub_id = 0;
  uint8_t _tlog_sub_id = 0;
  bool _enabled = false;
};

extern DebugLog debug_log;

#define DEBUG_LOG(fmt, ...) debug_log.logf(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
