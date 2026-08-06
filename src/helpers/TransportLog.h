#pragma once

#include <stdint.h>
#include <string.h>

// Ring sized to capture a full interactive session (e.g. a measurement sweep
// of ~40 commands = ~80 cmd recv/done events) for post-mortem fetch. Each
// event is 6 bytes on the wire; the ring is fetched paginated (see serialize).
#define TLOG_MAX_EVENTS 256

#define TLOG_MULTI_LOCK       1
#define TLOG_MULTI_RELEASE    2
#define TLOG_MULTI_DISCONNECT 3
#define TLOG_MULTI_LOST       4
#define TLOG_WIFI_ENABLE      5
#define TLOG_WIFI_DISABLE     6
#define TLOG_WIFI_CLIENT_NEW  7
#define TLOG_WIFI_SESSION_ON  8
#define TLOG_WIFI_SESSION_OFF 9
#define TLOG_WIFI_POWER_ON   10
#define TLOG_WIFI_POWER_OFF  11
#define TLOG_CMD_RECV        12   // detail = command byte (companion frame received)
#define TLOG_CMD_DONE        13   // detail = command byte (handler returned)
#define TLOG_WIFI_STA_DISCONNECTED 14   // detail = disconnect reason code
#define TLOG_WIFI_STA_GOT_IP       15   // station (re)associated and got an IP
#define TLOG_BLE_CONNECT           16   // BLE GATT link up (onConnect callback)
#define TLOG_BLE_DISCONNECT        17   // BLE GATT link down (onDisconnect callback)
#define TLOG_DEBUGLOG_READ         18   // marker: debuglog was fetched (boundary)
#define TLOG_COEX_PREFER_WIFI      19   // esp_coex_preference_set(PREFER_WIFI); detail = esp_err_t

// Stable transport type ids, logged as the `detail` of MULTI_* events so the
// transport is identifiable regardless of registration order (which varies with
// which transports are enabled).
#define TLOG_XPORT_BLE  1
#define TLOG_XPORT_USB  2
#define TLOG_XPORT_TCP  3

struct TransportEvent {
  uint32_t millis;
  uint8_t  type;
  int8_t   detail;
};

class TransportLog {
  TransportEvent _buf[TLOG_MAX_EVENTS];
  uint16_t _head = 0;
  uint16_t _count = 0;

public:
  void log(uint8_t type, int8_t detail = 0) {
#if ARDUINO
    _buf[_head] = { (uint32_t)::millis(), type, detail };
#else
    _buf[_head] = { 0, type, detail };
#endif
    _head = (_head + 1) % TLOG_MAX_EVENTS;
    if (_count < TLOG_MAX_EVENTS) _count++;
  }

  uint16_t count() const { return _count; }

  // Serialize a page of events (6 bytes each) starting at logical index
  // `offset` (0 = oldest). Writes the total event count to *total so the
  // caller can paginate. Returns the number of bytes written (n_events * 6).
  int serialize(uint8_t *dest, size_t max_len, uint16_t offset, uint16_t *total) const {
    *total = _count;
    if (offset >= _count) return 0;

    const int per_event = 6;
    int avail = max_len / per_event;
    int remaining = _count - offset;
    int n = remaining < avail ? remaining : avail;

    // Oldest logical event is at index 0 (not yet wrapped) or _head (wrapped).
    uint16_t logical_start = (_count < TLOG_MAX_EVENTS) ? 0 : _head;

    int pos = 0;
    for (int j = 0; j < n; j++) {
      uint16_t idx = (logical_start + offset + j) % TLOG_MAX_EVENTS;
      memcpy(&dest[pos], &_buf[idx].millis, 4); pos += 4;
      dest[pos++] = _buf[idx].type;
      dest[pos++] = (uint8_t)_buf[idx].detail;
    }
    return pos;
  }
};

extern TransportLog transport_log;
