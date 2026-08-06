#pragma once

#include <stdint.h>
#include <string.h>

// beebo: general-purpose command/span latency profiling ring.
//
// TransportLog captures connection/coexistence churn (link up/down, MULTI
// lock/release) with a 1-byte detail field and millis() resolution -- it
// answers "what happened, when". This ring answers "how long did it take":
// each event is a completed span, `id` identifying the call site (caller's
// choice of encoding -- companion command dispatch packs the two command
// bytes as (cmd<<8)|sub, so CMD_BEEBO sub-commands are distinguishable,
// unlike TransportLog's TLOG_CMD_* which only ever sees the outer byte) and
// `duration_us` its measured length, captured via the PROFILE_SCOPE RAII
// guard below so any call site -- not just command dispatch -- can be
// instrumented with one line.
//
// Ring sized for a full interactive session's worth of commands, same
// reasoning as TLOG_MAX_EVENTS; each event is 8 bytes on the wire, fetched
// paginated the same way (see serialize()).
#define PROFILE_MAX_EVENTS 128

struct ProfileEvent {
  uint32_t millis;
  uint16_t id;            // caller-defined call-site id
  uint16_t duration_us;   // saturates at 0xFFFF (65.535ms) -- spans longer
                           // than that just report the cap, not overflow
};

class ProfileLog {
  ProfileEvent _buf[PROFILE_MAX_EVENTS];
  uint16_t _head = 0;
  uint16_t _count = 0;

public:
  void log(uint16_t id, uint32_t duration_us) {
    uint16_t clamped = duration_us > 0xFFFF ? 0xFFFF : (uint16_t)duration_us;
#if ARDUINO
    _buf[_head] = { (uint32_t)::millis(), id, clamped };
#else
    _buf[_head] = { 0, id, clamped };
#endif
    _head = (_head + 1) % PROFILE_MAX_EVENTS;
    if (_count < PROFILE_MAX_EVENTS) _count++;
  }

  uint16_t count() const { return _count; }

  void clear() { _head = 0; _count = 0; }

  // Serialize a page of events (8 bytes each) starting at logical index
  // `offset` (0 = oldest). Writes the total event count to *total so the
  // caller can paginate. Returns the number of bytes written (n_events * 8).
  int serialize(uint8_t *dest, size_t max_len, uint16_t offset, uint16_t *total) const {
    *total = _count;
    if (offset >= _count) return 0;

    const int per_event = 8;
    int avail = max_len / per_event;
    int remaining = _count - offset;
    int n = remaining < avail ? remaining : avail;

    // Oldest logical event is at index 0 (not yet wrapped) or _head (wrapped).
    uint16_t logical_start = (_count < PROFILE_MAX_EVENTS) ? 0 : _head;

    int pos = 0;
    for (int j = 0; j < n; j++) {
      uint16_t idx = (logical_start + offset + j) % PROFILE_MAX_EVENTS;
      memcpy(&dest[pos], &_buf[idx].millis, 4); pos += 4;
      memcpy(&dest[pos], &_buf[idx].id, 2); pos += 2;
      memcpy(&dest[pos], &_buf[idx].duration_us, 2); pos += 2;
    }
    return pos;
  }
};

extern ProfileLog profile_log;

// RAII scope guard: times its own lifetime and logs the span to profile_log
// on destruction. #id should uniquely encode the call site (see Beebo.cpp's
// checkSerialInterface() for the (cmd<<8)|sub packing convention).
class ProfileScope {
  uint16_t _id;
  uint32_t _start_us;

public:
  explicit ProfileScope(uint16_t id) : _id(id), _start_us(_now()) {}
  ~ProfileScope() { profile_log.log(_id, _now() - _start_us); }

private:
  static uint32_t _now() {
#if ARDUINO
    return ::micros();
#else
    return 0;
#endif
  }
};

#define PROFILE_SCOPE_CONCAT_(a, b) a##b
#define PROFILE_SCOPE_CONCAT(a, b) PROFILE_SCOPE_CONCAT_(a, b)
#define PROFILE_SCOPE(id) ProfileScope PROFILE_SCOPE_CONCAT(__prof_scope_, __LINE__)(id)
