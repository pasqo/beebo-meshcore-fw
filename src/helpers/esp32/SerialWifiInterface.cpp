#include "SerialWifiInterface.h"
#include <WiFi.h>
#include <lwip/sockets.h>   // send(MSG_DONTWAIT) for the non-blocking queue drain
#include <string.h>   // memcpy
#include "../TransportLog.h"

void SerialWifiInterface::begin(int port) {
  _port = port;
  server.begin(port);
}

// ---------- public methods
void SerialWifiInterface::enable() {
  if (_isEnabled) return;
  transport_log.log(TLOG_WIFI_ENABLE);

  _isEnabled = true;
  clearBuffers();
  resetReceivedFrameHeader();
  if (_port > 0) server.begin(_port);
}

void SerialWifiInterface::disable() {
  transport_log.log(TLOG_WIFI_DISABLE, deviceConnected ? 1 : 0);
  _isEnabled = false;
  if (deviceConnected) {
    client.stop();
    deviceConnected = false;
  }
  clearBuffers();
  resetReceivedFrameHeader();
}

size_t SerialWifiInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_SEND_FRAME_SIZE) {
    WIFI_DEBUG_PRINTLN("writeFrame(), frame too big, len=%d\n", len);
    return 0;
  }

  if (deviceConnected && len > 0) {
    if (send_queue_len >= FRAME_QUEUE_SIZE) {
      WIFI_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
      _send_queue_full_count++;
      return 0;
    }

    send_queue[send_queue_len].len = len;  // add to send queue
    memcpy(send_queue[send_queue_len].buf, src, len);
    send_queue_len++;

    return len;
  }
  return 0;
}

bool SerialWifiInterface::isWriteBusy() const {
  // beebo: back-pressure the producer (e.g. the contacts-iterator loop in
  // Beebo.cpp, gated on !isWriteBusy() before enqueuing each frame) against
  // the actual TCP drain rate -- checkRecvFrame()'s send_queue drain stalls
  // on EAGAIN whenever the client's TCP window fills, but a stub that always
  // returned false let the producer keep enqueuing every loop tick regardless,
  // overflowing the 6-slot queue within a burst (e.g. GET_CONTACTS) instead of
  // pacing to it.
  return send_queue_len >= FRAME_QUEUE_SIZE - 1;
}

bool SerialWifiInterface::hasReceivedFrameHeader() {
  return received_frame_header.type != 0 && received_frame_header.length != 0;
}

void SerialWifiInterface::resetReceivedFrameHeader() {
  received_frame_header.type = 0;
  received_frame_header.length = 0;
}

void SerialWifiInterface::resetParserState() {
  // beebo: same buffer/parser reset disable()+enable() used to achieve as
  // a side effect, without the transport-log noise or the pointless
  // listen-socket teardown+rebind -- see BaseSerialInterface.h's own
  // comment on resetParserState(). _isEnabled is deliberately left alone:
  // this is only ever called on a sub that's currently the active,
  // already-enabled session, so it was never meaningfully "disabled".
  if (deviceConnected) {
    client.stop();
    deviceConnected = false;
  }
  clearBuffers();
  resetReceivedFrameHeader();
}

size_t SerialWifiInterface::checkRecvFrame(uint8_t dest[], size_t max_len) {
  // check if new client connected
  auto newClient = server.available();
  if (newClient) {
    if (deviceConnected) {
      // beebo: a genuinely live session is already locked in (e.g. a
      // phone app) -- WiFiServer's listen backlog (default 4, see
      // WiFiServer's own ctor) lets a second peer (the CLI, or the same
      // phone reconnecting) complete its TCP handshake at the OS level
      // independently of which client the app is currently servicing, so
      // unconditionally swapping to whatever server.available() hands
      // back here used to silently kill the live session out from under
      // it every time a second peer's connect raced in -- reject the new
      // one outright instead of preempting a session that's still alive.
      // beebo: detail = the rejected client's remote port -- lets a trace
      // distinguish a genuine second peer from a duplicate/retransmitted
      // accept of the SAME connection the live session is already on (same
      // port would show up twice), by comparing against the port
      // TLOG_WIFI_SESSION_ON logged for the live session.
      transport_log.log(TLOG_WIFI_CLIENT_REJECTED, newClient.remotePort());
      newClient.stop();
    } else {
      // beebo: the real first event of an app-level session -- logged as
      // the very first thing in the accept path, ahead of TCP new client/
      // session ON below and MultiSerialInterface::lockOn() (which only
      // runs later, once a first frame has actually been parsed).
      transport_log.log(TLOG_APP_SESSION_START, TLOG_XPORT_TCP);
      // beebo: detail packs the new client's remote port (bits 1+) with
      // whether deviceConnected was still true at accept time (bit 0) --
      // the latter flags the case this guard can't otherwise catch: the
      // OLD client's connected() had already gone false (so this wasn't a
      // rejected preemption), but deviceConnected itself hadn't been
      // updated yet. Same port-comparison use as TLOG_WIFI_CLIENT_REJECTED
      // above.
      transport_log.log(TLOG_WIFI_CLIENT_NEW, (newClient.remotePort() << 1) | (deviceConnected ? 1 : 0));
      deviceConnected = false;
      client.stop();
      clearBuffers();
      resetReceivedFrameHeader();

      client = newClient;
      client.setNoDelay(true);
    }
  }

  if (client.connected()) {
    if (!deviceConnected) {
      // beebo: detail = the newly-live client's remote port, so a later
      // TLOG_WIFI_CLIENT_NEW/REJECTED can be matched against the session
      // it raced with -- see those events' own comments above.
      transport_log.log(TLOG_WIFI_SESSION_ON, client.remotePort());
      deviceConnected = true;
    }
  } else if (deviceConnected) {
    client.stop();
    deviceConnected = false;
    clearBuffers();
    resetReceivedFrameHeader();
    transport_log.log(TLOG_WIFI_SESSION_OFF);
  }

  if (deviceConnected) {
    while (send_queue_len > 0) {   // drain send queue before receiving
      // Non-blocking send with partial-frame progress: _send_off spans the
      // 3-byte '>'+len header then the body. client.write() is not used here —
      // it blocks in a 1 s select/retry loop as soon as the TCP send buffer
      // fills (which back-to-back bulk drains do by design), stalling the mesh
      // loop and radio RX. send(MSG_DONTWAIT) writes what fits; a partly-sent
      // frame stays at the queue head and resumes next loop. The client
      // delimits by the length header, so any TCP segmentation is fine.
      int len = send_queue[0].len;
      uint8_t hdr[3];  // same header as serial interface so client can delimit frames
      hdr[0] = '>';
      hdr[1] = (len & 0xFF);  // LSB
      hdr[2] = (len >> 8);    // MSB
      int total = 3 + len;
      while (_send_off < total) {
        const uint8_t* src;
        int n;
        if (_send_off < 3) {
          src = &hdr[_send_off];
          n = 3 - _send_off;
        } else {
          src = &send_queue[0].buf[_send_off - 3];
          n = total - _send_off;
        }
        int res = send(client.fd(), src, n, MSG_DONTWAIT);
        if (res <= 0) break;   // buffer full (EAGAIN) or error; connected() catches errors next call
        _send_off += res;
        _last_write = millis();
      }
      if (_send_off < total) break;   // TCP send buffer full: resume this frame next loop
      _send_off = 0;
      send_queue_len--;
      for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
        send_queue[i] = send_queue[i + 1];
      }
    }

    // check if we are waiting for a frame header
    if(!hasReceivedFrameHeader()){

      // make sure we have received enough bytes for a frame header
      // 3 bytes frame header = (1 byte frame type) + (2 bytes frame length as unsigned 16-bit little endian)
      int frame_header_length = 3;
      if(client.available() >= frame_header_length){

          // read frame header
        client.readBytes(&received_frame_header.type, 1);
        client.readBytes((uint8_t*)&received_frame_header.length, 2);

      }

    }

    // check if we have received a frame header
    if(hasReceivedFrameHeader()){

      int frame_type = received_frame_header.type;
      int frame_length = received_frame_header.length;

      // skip frames that exceed the buffer the caller provided -- drains
      // incrementally across calls too (client.available() bytes only),
      // same reasoning as the accumulation path below: a bogus/oversized
      // length shouldn't ever block waiting for bytes that aren't there yet.
      if ((size_t)frame_length > max_len) {
        int available = client.available();
        int to_skip = frame_length - (int)_recv_body_len;
        if (to_skip > available) to_skip = available;
        uint8_t scratch[64];
        while (to_skip > 0) {
          int n = to_skip < (int)sizeof(scratch) ? to_skip : (int)sizeof(scratch);
          int got = client.read(scratch, n);
          if (got <= 0) break;
          _recv_body_len += got;
          to_skip -= got;
        }
        if (_recv_body_len >= (size_t)frame_length) {
          WIFI_DEBUG_PRINTLN("Skipped frame: length=%d exceeds max_len=%d", frame_length, (int)max_len);
          resetReceivedFrameHeader();
          _recv_body_len = 0;
        }
        return 0;
      }

      // skip frames that are not expected type
      // '<' is 0x3c which indicates a frame sent from app to radio
      if(frame_type != '<'){
        int available = client.available();
        int to_skip = frame_length - (int)_recv_body_len;
        if (to_skip > available) to_skip = available;
        uint8_t scratch[64];
        while (to_skip > 0) {
          int n = to_skip < (int)sizeof(scratch) ? to_skip : (int)sizeof(scratch);
          int got = client.read(scratch, n);
          if (got <= 0) break;
          _recv_body_len += got;
          to_skip -= got;
        }
        if (_recv_body_len >= (size_t)frame_length) {
          WIFI_DEBUG_PRINTLN("Skipped frame: type=0x%x is unexpected", frame_type);
          resetReceivedFrameHeader();
          _recv_body_len = 0;
        }
        return 0;
      }

      // beebo: accumulate the body across as many calls as it takes --
      // only ever reads bytes client.available() already reports ready
      // (client.read() with an available-bounded count never blocks), so a
      // frame larger than the TCP socket's own receive buffer still
      // completes over several checkRecvFrame() calls instead of
      // deadlocking (see _recv_body_buf's own comment in the header).
      int available = client.available();
      int want = frame_length - (int)_recv_body_len;
      if (want > available) want = available;
      if (want > 0) {
        int got = client.read(&_recv_body_buf[_recv_body_len], want);
        if (got > 0) _recv_body_len += got;
      }

      if (_recv_body_len < (size_t)frame_length) {
        return 0;   // still waiting on more bytes
      }

      memcpy(dest, _recv_body_buf, frame_length);

      // ready for next frame
      resetReceivedFrameHeader();
      _recv_body_len = 0;
      return frame_length;

    }
  }

  return 0;
}

bool SerialWifiInterface::isConnected() const {
  return deviceConnected;
}