#if ARDUINO
#include <Arduino.h>   // millis(), used by logRing()/logLink()
#endif
#include <stdarg.h>
#include "DebugRing.h"

// Single definition for the shared debug ring/link. Lives here (not in an
// application's main.cpp) so every firmware that links the transport helpers
// which log to it (esp32 SerialBLEInterface / SerialWifiInterface,
// MultiSerialInterface) resolves the symbol; builds that never reference it
// have the object stripped by linker GC.
DebugRing debug_ring;

void DebugRing::logRing(const char* file, int line, uint8_t type, uint8_t severity, int32_t detail) {
#if ARDUINO
  uint32_t ms = (uint32_t)::millis();
#else
  uint32_t ms = 0;
#endif
  // beebo: Low severity is link-only -- never written into the ring, same
  // as DLOGL/M/H never are (see logLink()). Only H/M actually occupy ring
  // slots; L still reaches a live `--debug` link exactly like any other
  // severity, via pushRlogFrame() below.
  if (severity != DLOG_SEV_L) {
    _buf[_head] = { ms, type, detail, severity, file, (uint16_t)line };
    _head = (_head + 1) % RLOG_MAX_EVENTS;
    if (_count < RLOG_MAX_EVENTS) _count++;
  }

  TRANSPORT_DEBUG_PRINTLN("type=%u detail=%ld", (unsigned)type, (long)detail);
  pushRlogFrame(file, line, (uint16_t)type, severity, detail, ms);
}

void DebugRing::logLink(const char* file, int line, uint16_t id, uint8_t severity, const char* fmt, ...) {
  // beebo: no isConnected() gate -- see pushRlogFrame()'s own comment on why.
  if (!_enabled || !_serial) return;

  uint8_t out[200];
  size_t cap = _serial->getMaxSendFrameSize();
  if (cap > sizeof(out)) cap = sizeof(out);
  if (cap < 16) return;   // no room for even the header

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

  _serial->writeFrameBestEffort(out, pos + msg_len);
}
