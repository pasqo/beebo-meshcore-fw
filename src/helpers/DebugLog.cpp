#if ARDUINO
#include <Arduino.h>   // millis(), used by logRing()/logLink()
#endif
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "DebugLog.h"

// Single definition for the shared debug ring/link. Lives here (not in an
// application's main.cpp) so every firmware that links the transport helpers
// which log to it (esp32 SerialBLEInterface / SerialWifiInterface,
// MultiSerialInterface) resolves the symbol; builds that never reference it
// have the object stripped by linker GC.
DebugLog debug_log;

// beebo: the shared (epoch_sec, millis()) time anchor declared in
// MonRing.h -- defined here (not in MonRing.h, which is header-only) so
// both this file's DLOG live push and MonRing's own offset math link
// against the exact same storage. See MonRing.h's own comment for the
// full rationale.
uint32_t g_time_anchor_epoch_sec = 0;
uint16_t g_time_anchor_ms_frac = 0;
uint32_t g_time_anchor_millis = 0;

void DebugLog::logRing(const char* file, int line, uint8_t type, uint8_t severity, int32_t detail,
                        const uint8_t* user) {
#if ARDUINO
  uint32_t ms = (uint32_t)::millis();
#else
  uint32_t ms = 0;
#endif
  uint8_t user_buf[5] = {0, 0, 0, 0, 0};
  if (user != nullptr) memcpy(user_buf, user, sizeof(user_buf));
  // beebo: RLOGH/M only now -- RLOGL was retired 2026-09-11 (converted to
  // DLOGL at every call site; DebugRecord.type only encodes H vs M,
  // MonRing.h, so Low severity was never representable here anyway).
  if (_debug_sink) _debug_sink(type, severity, detail, ms, file, line, user_buf);

  TRANSPORT_DEBUG_PRINTLN("type=%u detail=%ld", (unsigned)type, (long)detail);
}

void DebugLog::logLink(const char* file, int line, uint16_t id, uint8_t severity, const char* fmt, ...) {
  // beebo: no isConnected() gate -- DualModeSerialInterface::isConnected()
  // is an unconditional `return true` stub (no real way to detect a host-
  // side close on the native USB-Serial-JTAG peripheral, see that file's
  // own comment), so it never actually prevented a push into the void.
  // writeFrameBestEffort() (via pushToTargets() below) is what makes an
  // unread push cheap now, not this check -- confirmed on real hardware
  // that relying on isConnected() here let every push retry-block the main
  // loop for up to ZERO_WRITE_GIVEUP_MS (3s) whenever nothing was draining
  // the USB TX side, stalling completely unrelated traffic (BLE/TCP
  // included) on every single event while armed.
  if (!isEnabled() || (!_serial && !_usb)) return;

  BaseSerialInterface* cap_source = _serial ? _serial : _usb;
  uint8_t out[200];
  size_t cap = cap_source->getMaxSendFrameSize();
  if (cap > sizeof(out)) cap = sizeof(out);
  if (cap < 20) return;   // no room for even the header

#if ARDUINO
  uint32_t ms = (uint32_t)::millis();
#else
  uint32_t ms = 0;
#endif
  size_t pos = writeHeader(out, cap, 0, _resp_code, _log_sub_id, id, severity, line, file, ms);
  if (pos == 0) return;

  char* text = (char*)&out[pos];
  size_t text_cap = cap - pos;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(text, text_cap, fmt, args);
  va_end(args);
  if (n < 0) n = 0;
  size_t msg_len = (size_t)n < text_cap ? (size_t)n : (text_cap > 0 ? text_cap - 1 : 0);

  pushToTargets(out, pos + msg_len, _usb_enabled, _session_enabled);
}
