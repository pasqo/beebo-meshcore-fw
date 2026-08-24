#pragma once

#include <stdint.h>
#include <string.h>

// Ring sized to capture a full interactive session (e.g. a measurement sweep
// of ~40 commands = ~80 cmd recv/done events) for post-mortem fetch. Each
// event is 9 bytes on the wire; the ring is fetched paginated (see serialize).
// 1024 events * 12 bytes (in-memory, padded) = 12KB static/.bss -- bumped
// from 256 since a long-running interactive/test session's own command
// traffic (each CLI round trip logs a CMD_RECV/CMD_DONE pair) can wrap a
// 256-deep ring within a handful of commands, evicting real transport
// events (e.g. TLOG_WIFI_STA_GOT_IP) before they can be fetched. Confirmed
// affordable against a real device's live free heap (`bench status`, not
// just the raw 512KB SRAM figure): 62KB free before this bump, 53KB after.
#define TLOG_MAX_EVENTS 1024

// beebo: the app-level companion session, transport-agnostic (whichever of
// BLE/USB/TCP wins the MultiSerialInterface lock) -- named APP_SESSION_* to
// stay distinct from TLOG_WIFI_SESSION_ON/OFF below (that pair is the raw
// TCP socket's own connected state, one layer down: a socket can go through
// several ON/OFF cycles, or none at all if the TCP layer already dropped
// out, without ever mapping to a real APP_SESSION_START/END here).
#define TLOG_APP_SESSION_START            1
#define TLOG_APP_SESSION_END_RELEASED     2
#define TLOG_APP_SESSION_END_DISCONNECT   3
#define TLOG_APP_SESSION_END_LOST         4
#define TLOG_WIFI_ENABLE      5
#define TLOG_WIFI_DISABLE     6
#define TLOG_WIFI_CLIENT_NEW  7   // detail = (remote_port << 1) | (deviceConnected ? 1 : 0) at accept time
#define TLOG_WIFI_SESSION_ON  8   // detail = the now-live client's remote port
#define TLOG_WIFI_SESSION_OFF 9
#define TLOG_WIFI_POWER_ON   10
#define TLOG_WIFI_POWER_OFF  11
#define TLOG_CMD_RECV        12   // detail = (cmd_frame[0]<<8)|cmd_frame[1] for CMD_BEEBO/CMD_GET_STATS (their second byte is a real sub-id), else just cmd_frame[0] (companion frame received)
#define TLOG_CMD_DONE        13   // detail = same (cmd<<8)|sub encoding as TLOG_CMD_RECV (handler returned)
#define TLOG_WIFI_STA_DISCONNECTED 14   // detail = disconnect reason code
#define TLOG_WIFI_STA_GOT_IP       15   // station (re)associated and got an IP; detail = the IPv4 address, packed MSB-first (octet1<<24 | octet2<<16 | octet3<<8 | octet4)
#define TLOG_BLE_CONNECT           16   // BLE GATT link up (onConnect callback)
#define TLOG_BLE_DISCONNECT        17   // BLE GATT link down (onDisconnect callback)
#define TLOG_DEBUGLOG_READ         18   // marker: debuglog was fetched (boundary)
#define TLOG_COEX_PREFER_WIFI      19   // esp_coex_preference_set(PREFER_WIFI); detail = esp_err_t
#define TLOG_WIFI_CLIENT_REJECTED  20   // a second peer's TCP connect was accepted at the OS level (WiFiServer's backlog) while a live session was already locked in -- rejected instead of preempting it; detail = the rejected client's remote port

// Stable transport type ids, logged as the `detail` of MULTI_* events so the
// transport is identifiable regardless of registration order (which varies with
// which transports are enabled).
#define TLOG_XPORT_BLE  1
#define TLOG_XPORT_USB  2
#define TLOG_XPORT_TCP  3

struct TransportEvent {
  uint32_t millis;
  uint8_t  type;
  int32_t  detail;
};

class TransportLog {
  TransportEvent _buf[TLOG_MAX_EVENTS];
  uint16_t _head = 0;
  uint16_t _count = 0;

public:
  // `detail` widened from a single byte to 4 bytes (still 0 by default) so
  // TLOG_WIFI_STA_GOT_IP can carry a full packed IPv4 address; every other
  // event type keeps using just the low byte, the upper 3 bytes stay zero.
  void log(uint8_t type, int32_t detail = 0) {
#if ARDUINO
    _buf[_head] = { (uint32_t)::millis(), type, detail };
#else
    _buf[_head] = { 0, type, detail };
#endif
    _head = (_head + 1) % TLOG_MAX_EVENTS;
    if (_count < TLOG_MAX_EVENTS) _count++;
  }

  uint16_t count() const { return _count; }

  // Serialize a page of events (9 bytes each) starting at logical index
  // `offset` (0 = oldest). Writes the total event count to *total so the
  // caller can paginate. Returns the number of bytes written (n_events * 9).
  int serialize(uint8_t *dest, size_t max_len, uint16_t offset, uint16_t *total) const {
    *total = _count;
    if (offset >= _count) return 0;

    const int per_event = 9;
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
      memcpy(&dest[pos], &_buf[idx].detail, 4); pos += 4;
    }
    return pos;
  }
};

extern TransportLog transport_log;
