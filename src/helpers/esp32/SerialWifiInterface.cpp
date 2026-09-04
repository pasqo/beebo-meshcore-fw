#include "SerialWifiInterface.h"
#include <WiFi.h>
#include <lwip/sockets.h>   // send(MSG_DONTWAIT) for the non-blocking queue drain
#include <string.h>   // memcpy
#include "../TransportLog.h"
#include "../DebugLog.h"

void SerialWifiInterface::begin(int port) {
  _port = port;
  server.begin(port);
}

// ---------- public methods
void SerialWifiInterface::enable() {
  if (_isEnabled) return;
  // beebo: no dedicated log call here -- TLOG_XPORT_LINK_VAR_WIFI_IFACE_ENABLED
  // (Beebo::_checkTransportStateChanges()) already captures this transition
  // every tick, retired 2026-08-31 as a duplicate (see TransportLog.h).

  _isEnabled = true;
  clearBuffers();
  resetReceivedFrameHeader();
  if (_port > 0) server.begin(_port);
}

void SerialWifiInterface::disable() {
  // beebo: see enable()'s comment -- TLOG_XPORT_LINK_VAR_WIFI_IFACE_ENABLED
  // already covers this transition; the deviceConnected-at-disable detail
  // this used to carry is reconstructable from TLOG_XPORT_LINK_VAR_WIFI_IFACE_CONNECTED
  // around the same timestamp if ever needed.
  _isEnabled = false;
  if (deviceConnected) {
    client.stop();
    deviceConnected = false;
  }
  // beebo: same _listening footgun rebind()'s own comment documents --
  // WiFiServer::begin() no-ops whenever _listening is already true, and
  // only end() clears it. A live tcp-off transport switch calls disable()
  // right before WiFi.mode(WIFI_OFF) tears the whole radio/netif down
  // (Beebo.cpp's applyTransportConfig()), but without this end() call
  // _listening stayed stuck true across that teardown -- a later
  // tcp-on's enable() -> server.begin(_port) then no-op'd, leaving
  // isListening()/WL_STATUS both reporting a healthy connection while the
  // actual listening socket was dead underneath, so no TCP client could
  // ever connect again. Found via hardware-testing a live tcp-off/tcp-on
  // cycle (2026-08-31).
  server.end();
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
  // beebo: refuse a second peer at the TCP/listen level instead of accepting
  // it into the app and rejecting it afterward. The prior approach (accept
  // via server.available(), then newClient.stop() when a session was already
  // live) was seen to correlate with the LIVE client's own session dying
  // shortly after a reject -- even though newClient.stop() never touches
  // `client` directly, suggesting a lower-level ESP32 WiFiClient/lwIP
  // interaction between two socket ops landing close together. Closing the
  // listening socket entirely while a session is live sidesteps that
  // interaction altogether: a stray connect attempt gets an immediate RST
  // from the OS and never reaches server.available() at all. Re-opened here,
  // at the top of the next call, once the session ends.
  if (!deviceConnected && !server && _isEnabled && _port > 0) {
    // beebo: covers both the deliberate reopen after Beebo's own session-
    // exclusivity close above, and an uncommanded listener death -- either
    // way, _checkTransportStateChanges() (Beebo.cpp) picks up the resulting
    // wifi.listening 0 -> 1 transition on this same loop() tick and logs it,
    // so no separate event is logged here (see TransportLog.h's retired
    // TLOG_WIFI_LISTEN_ENABLED entry).
    DEBUG_LOG("listening socket was dead, rebuilding");
    server.begin(_port);
  }

  // check if new client connected -- only reachable while server is
  // listening, i.e. while no session is live (see above).
  auto newClient = server.available();
  if (newClient) {
    if (deviceConnected) {
      // beebo: defensive fallback only -- shouldn't happen now that the
      // listening socket is closed for the duration of a live session, but
      // keep rejecting outright (never preempt) in case a lwIP race still
      // hands one back. detail = the rejected client's remote port.
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
      // beebo: TCP keepalive, tuned for a LAN peer rather than lwIP's
      // 2-hour internet-facing default -- the standard mechanism for
      // detecting a peer that vanishes without a clean FIN/RST (crash, WiFi
      // drop, cable pull, NAT mapping dropped): the OS itself probes and
      // reports failure through the same connected()/errno path already in
      // use, no app-level idle timer needed. This is what actually bounds
      // how long the listening socket can stay closed (see server.end()
      // below) if the live peer disappears silently -- without it, a
      // session stuck in that state would keep the port deaf indefinitely.
      // idle=10s, 3 probes 5s apart => a dead peer is detected within ~25s.
      { int enable = 1;
        client.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, (const void*)&enable, sizeof(enable));
        int keepidle = 10;
        client.setSocketOption(IPPROTO_TCP, TCP_KEEPIDLE, (const void*)&keepidle, sizeof(keepidle));
        int keepintvl = 5;
        client.setSocketOption(IPPROTO_TCP, TCP_KEEPINTVL, (const void*)&keepintvl, sizeof(keepintvl));
        int keepcnt = 3;
        client.setSocketOption(IPPROTO_TCP, TCP_KEEPCNT, (const void*)&keepcnt, sizeof(keepcnt));
      }
    }
  }

  if (client.connected()) {
    if (!deviceConnected) {
      // beebo: detail = the newly-live client's remote port, so a later
      // TLOG_WIFI_CLIENT_NEW/REJECTED can be matched against the session
      // it raced with -- see those events' own comments above.
      transport_log.log(TLOG_WIFI_SESSION_ON, client.remotePort());
      deviceConnected = true;
      server.end();   // beebo: stop accepting new peers for the duration of this session
    }
  } else if (deviceConnected) {
    // beebo: capture the socket's pending error (if any) before stop()
    // closes the fd -- distinguishes a clean peer-initiated FIN (0) from a
    // real error (e.g. ETIMEDOUT from the keepalive probes below timing
    // out, ECONNRESET from a peer RST) so a later beebo monitor transport
    // read can tell which one actually happened instead of guessing from
    // elapsed time alone.
    int sock_err = 0;
    socklen_t sock_err_len = sizeof(sock_err);
    getsockopt(client.fd(), SOL_SOCKET, SO_ERROR, &sock_err, &sock_err_len);
    client.stop();
    deviceConnected = false;
    clearBuffers();
    resetReceivedFrameHeader();
    transport_log.log(TLOG_WIFI_SESSION_OFF, sock_err);
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