#include "DebugLog.h"
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

DebugLog debug_log;

void DebugLog::logf(const char* file, int line, const char* fmt, ...) {
  if (!_armed || !_serial) return;

  const char* base = strrchr(file, '/');
  base = base ? base + 1 : file;

  // beebo: frame = [resp_code:1][log_sub_id:1][millis:4 LE][line:2 LE][text],
  // capped to this transport's own send limit (BLE's 176B cap is the
  // tightest) so a long message truncates instead of never being sent.
  uint8_t out[200];
  size_t cap = _serial->getMaxSendFrameSize();
  if (cap > sizeof(out)) cap = sizeof(out);
  if (cap < 16) return;   // no room for even the header

  out[0] = _resp_code;
  out[1] = _log_sub_id;
  uint32_t ms = millis();
  memcpy(&out[2], &ms, 4);
  uint16_t line16 = (uint16_t)line;
  memcpy(&out[6], &line16, 2);

  char* text = (char*)&out[8];
  size_t text_cap = cap - 8;
  int n = snprintf(text, text_cap, "%s: ", base);
  if (n < 0) n = 0;
  if ((size_t)n >= text_cap) n = text_cap > 0 ? (int)text_cap - 1 : 0;

  va_list args;
  va_start(args, fmt);
  int n2 = vsnprintf(text + n, text_cap - n, fmt, args);
  va_end(args);
  if (n2 < 0) n2 = 0;
  size_t msg_len = (size_t)n + (size_t)((size_t)n2 < text_cap - n ? n2 : text_cap - n);

  _serial->writeFrame(out, 8 + msg_len);
}

void DebugLog::logEvent(uint8_t type, int32_t detail) {
  if (!_armed || !_serial) return;

  // beebo: frame = [resp_code:1][tlog_sub_id:1][millis:4 LE][type:1][detail:4 LE],
  // fixed 11 bytes -- always fits even the tightest (BLE) send cap, no
  // truncation logic needed the way logf()'s free-text frame needs.
  uint8_t out[11];
  out[0] = _resp_code;
  out[1] = _tlog_sub_id;
  uint32_t ms = millis();
  memcpy(&out[2], &ms, 4);
  out[6] = type;
  memcpy(&out[7], &detail, 4);

  _serial->writeFrame(out, sizeof(out));
}
