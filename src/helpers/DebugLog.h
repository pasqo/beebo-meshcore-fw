#pragma once

#include <stdint.h>
#include "BaseSerialInterface.h"

// beebo: live, unsolicited debug-event push -- the DEBUG_LOG(fmt, ...)
// drop-in replacement for an ad-hoc Serial.printf(), plus a live mirror of
// every TransportLog event (see logEvent(), called from TransportLog::log()
// itself), delivered as properly framed companion-protocol events
// (RESP_CODE_BEEBO/DEBUG_LOG and RESP_CODE_BEEBO/DEBUG_TLOG, see
// protocol.yaml) over whichever transport currently holds the session --
// USB, BLE, or TCP, whichever MultiSerialInterface has locked in -- instead
// of raw text that collides with binary protocol bytes on shared USB (see
// kbase/DEBUGGING.md's USB/Serial-sharing constraint). No ring buffer for
// either kind -- each call either pushes immediately (while armed and a
// session is live) or costs one cheap bool check (while not), same
// arm-once-then-stream shape as MonRing's own capture pump. Armed/disarmed
// per session by BEEBO_CMD_DEBUG_LOG_ARM (`beebo --debug`); auto-disarmed on
// session end so a forgotten arm doesn't push forever into the void.
//
// A global singleton, same pattern as TransportLog.h's `transport_log` --
// needed since DEBUG_LOG() must be callable from any shared file (e.g.
// SerialWifiInterface.cpp), not just Beebo.cpp, without depending on the
// generated, multi_role-only BeeboProtocol.h. Beebo::begin() calls attach()
// once with the live serial interface and the generated frame-prefix bytes
// for both response shapes so this file stays free of any beebo-protocol-
// specific constant.
class DebugLog {
public:
  void attach(BaseSerialInterface* serial, uint8_t resp_code,
              uint8_t log_sub_id, uint8_t tlog_sub_id) {
    _serial = serial;
    _resp_code = resp_code;
    _log_sub_id = log_sub_id;
    _tlog_sub_id = tlog_sub_id;
  }
  void setArmed(bool armed) { _armed = armed; }
  bool isArmed() const { return _armed; }

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
  bool _armed = false;
};

extern DebugLog debug_log;

#define DEBUG_LOG(fmt, ...) debug_log.logf(__FILE__, __LINE__, fmt, ##__VA_ARGS__)
