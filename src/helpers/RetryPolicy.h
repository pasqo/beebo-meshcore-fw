#pragma once
#include <stdint.h>

// beebo: a route-retry policy (kbase/ROUTE_RETRY.md) in one byte: the coding
// rates of the retries of a forward whose echo timed out, in order. "5|6" is
// two retries, the first at 4/5 and the second at 4/6; 0 is no retries.
//   bits 7..6  retry count, 0..3
//   bits 5..4  coding rate - 5 of retry 1
//   bits 3..2  coding rate - 5 of retry 2
//   bits 1..0  coding rate - 5 of retry 3
// The bits of the retries not taken stay 0, so each policy has one byte.
// Header-only, no Arduino dependency.
class RetryPolicy {
public:
  static const int MAX_RETRIES = 3;
  static const int CR_MIN = 5, CR_MAX = 8;

  static uint8_t count(uint8_t p) { return p >> 6; }
  // Coding rate 4/x of retry `attempt` (1..count); 0 when there is no such retry.
  static uint8_t crAt(uint8_t p, int attempt) {
    if (attempt < 1 || attempt > count(p)) return 0;
    return (uint8_t)(CR_MIN + ((p >> (6 - 2 * attempt)) & 3));
  }
  static bool valid(uint8_t p) {
    int n = count(p);
    if (n > MAX_RETRIES) return false;
    return (p & ~(0xFFu << (6 - 2 * n)) & 0x3F) == 0;   // the retries not taken are 0
  }
  // The policy of `n` coding rates (5..8); false on a bad count or rate.
  static bool pack(const uint8_t* cr, int n, uint8_t* out) {
    if (n < 1 || n > MAX_RETRIES) return false;
    uint8_t p = (uint8_t)(n << 6);
    for (int i = 0; i < n; i++) {
      if (cr[i] < CR_MIN || cr[i] > CR_MAX) return false;
      p |= (uint8_t)((cr[i] - CR_MIN) << (4 - 2 * i));
    }
    *out = p;
    return true;
  }

  // "0", "8" or "5|6|8" (spaces around numbers allowed) into `out`. Parsing
  // stops at '\0' or `stop` (the policy separator of a list) and sets *end there.
  static bool parse(const char* text, char stop, uint8_t* out, const char** end) {
    const char* p = text;
    while (*p == ' ') p++;
    if (*p == '0') {
      p++;
      while (*p == ' ') p++;
      if (*p != '\0' && *p != stop) return false;
      *out = 0;
      *end = p;
      return true;
    }
    uint8_t cr[MAX_RETRIES];
    int n = 0;
    while (true) {
      while (*p == ' ') p++;
      if (*p < '0' || *p > '9' || n >= MAX_RETRIES) return false;
      cr[n++] = (uint8_t)(*p++ - '0');
      while (*p == ' ') p++;
      if (*p == '|') { p++; continue; }
      if (*p != '\0' && *p != stop) return false;
      break;
    }
    *end = p;
    return pack(cr, n, out);
  }

  // "0", "8", "5|6" into buf (size at least 6); the length.
  static int format(uint8_t p, char* buf) {
    int len = 0;
    if (count(p) == 0) buf[len++] = '0';
    for (int a = 1; a <= count(p); a++) {
      if (a > 1) buf[len++] = '|';
      buf[len++] = (char)('0' + crAt(p, a));
    }
    buf[len] = '\0';
    return len;
  }
};
