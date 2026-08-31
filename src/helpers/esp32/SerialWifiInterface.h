#pragma once

#include "../BaseSerialInterface.h"
#include <WiFi.h>

class SerialWifiInterface : public BaseSerialInterface {
  bool deviceConnected;
  bool _isEnabled;
  unsigned long _last_write;
  unsigned long adv_restart_time;
  int _port;

  WiFiServer server;
  WiFiClient client;

  struct FrameHeader {
    uint8_t type;
    uint16_t length;
  };

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  // beebo: outbound frames may be larger under BULK_XFER (up to MAX_SEND_FRAME_SIZE),
  // so the send queue needs a wider len and buffer than the recv path. Equals the
  // recv Frame when BULK_XFER is off (MAX_SEND_FRAME_SIZE == MAX_FRAME_SIZE).
  struct SendFrame {
    uint16_t len;
    uint8_t buf[MAX_SEND_FRAME_SIZE];
  };

  FrameHeader received_frame_header;

  // beebo: accumulates a frame body across multiple checkRecvFrame() calls
  // instead of requiring the whole thing to already be sitting in
  // client.available() before touching any of it -- the old all-or-nothing
  // read silently depended on frame_length fitting inside ESP32 lwIP's
  // default TCP socket receive buffer (true at the old 4098-byte OTA frame
  // size, false once OTA_CHUNK_SIZE was tried at 8192: available() plateaus
  // below frame_length, the frame is never read, the TCP window closes, and
  // the transfer deadlocks). Sized OTA_FRAME_SIZE, the largest frame this
  // class ever advertises via getMaxRecvFrameSize().
  uint8_t _recv_body_buf[OTA_FRAME_SIZE];
  size_t _recv_body_len = 0;

  // beebo: bumped 4->6 (not 8 -- each send_queue slot is MAX_SEND_FRAME_SIZE
  // bytes, 2048 under BULK_XFER, so 8 overflows DRAM by ~5KB) -- absorbs a
  // short burst of outbound frames (several RX packets forwarded to the app
  // back-to-back, or a bulk page reply) without dropping via
  // link_tx_queue_full. Doesn't fix a genuinely slow/stalled client or WiFi
  // link, which still eventually fills a bigger queue too (see MonRing.h's
  // EVENT_LINK_TX_QUEUE_FULL).
  #define FRAME_QUEUE_SIZE  6
  int recv_queue_len;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  int send_queue_len;
  SendFrame send_queue[FRAME_QUEUE_SIZE];
  int _send_off;   // bytes of the head send frame (3-byte header + body) already written
  uint32_t _send_queue_full_count = 0;

  // beebo: the original eviction bug this class guards against (a live
  // session silently replaced by a second peer's connect racing in via the
  // listen backlog) is fully explained by the accept-race itself, not by
  // client.connected() blipping -- checkRecvFrame()'s `if (deviceConnected)`
  // guard alone is sufficient, since deviceConnected only clears once the
  // socket is genuinely gone. No debounce needed here.

  void clearBuffers() { recv_queue_len = 0; send_queue_len = 0; _send_off = 0; _recv_body_len = 0; }

protected:

public:
  SerialWifiInterface() : server(WiFiServer()), client(WiFiClient()) {
    deviceConnected = false;
    _isEnabled = false;
    _last_write = 0;
    send_queue_len = recv_queue_len = 0;
    _send_off = 0;
    received_frame_header.type = 0;
    received_frame_header.length = 0;
    _port = 0;
  }

  void begin(int port);

  // beebo: re-bind the listening socket to the current STA network
  // interface. enable()'s own `if (_isEnabled) return;` guard makes it a
  // no-op for this -- WiFi.disconnect()/WiFi.begin()/WiFi.reconnect() (the
  // live-creds-change and auto-reconnect paths in Beebo.cpp) never flip
  // _isEnabled, so enable() is never re-entered, and the WiFiServer bound
  // at the *previous* association is left orphaned: it keeps accepting a
  // TCP handshake at the IP layer (still routable, still ESTABLISHED) but
  // server.available() never hands the app a live client again. Call this
  // from the STA's got-IP handler after any reassociation, not just the
  // first one.
  //
  // A plain server.begin(_port) is not enough here: WiFiServer::begin()
  // no-ops whenever _listening is already true (WiFiServer.cpp), and a
  // reassociation never clears it -- only WiFiServer::end() does. Since the
  // listener sits open in steady state (only end()'ed for the duration of
  // a live session -- see checkRecvFrame()'s server.end() below), _listening
  // is almost always still true when a reassociation happens, so the old
  // begin()-only rebind() silently did nothing: the orphaned socket from
  // before the reassociation stayed in place, later handing back a
  // connection that accepts and exchanges one frame but is actually dead
  // underneath (confirmed via a captured transport-log trace: a TCP session
  // that ran CMD_APP_START through to CMD done, then dropped 3ms later --
  // never reaching the client). end() first forces WiFiServer to close the
  // old socket and open a fresh one against the current association.
  //
  // Skipped while deviceConnected: a live session already has the listener
  // closed for its own exclusivity guarantee (see checkRecvFrame()'s
  // comment on server.end() below) -- rebuilding it here would reopen the
  // port mid-session. checkRecvFrame()'s own top-of-function reopen logic
  // picks the listener back up once that session actually ends.
  void rebind() { if (_port > 0 && !deviceConnected) { server.end(); server.begin(_port); } }

  // beebo: raw listen-socket state (WiFiServer's own _listening, via its
  // operator bool()) -- distinct from isEnabled()/isConnected() below, and
  // exactly the piece of state rebind()'s no-op bug above turned out to
  // hinge on. Exposed for transport-state dumps (Beebo::_logTransportState()).
  bool isListening() { return (bool)server; }   // WiFiServer::operator bool() isn't const

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  void resetParserState() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;
  bool isWriteBusy() const override;

  size_t getMaxRecvFrameSize() const override { return OTA_FRAME_SIZE; }
  size_t getMaxSendFrameSize() const override { return MAX_SEND_FRAME_SIZE; }
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override;

  bool hasReceivedFrameHeader();
  void resetReceivedFrameHeader();

  // beebo: lifetime count of writeFrame() silently dropping a frame because
  // send_queue (FRAME_QUEUE_SIZE=6) was full. No recv-side equivalent -- WiFi
  // reads one frame directly off the TCP stream rather than through a fixed
  // recv_queue the way BLE's onWrite() does.
  uint32_t getSendQueueFullCount() const { return _send_queue_full_count; }
};

#if WIFI_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define WIFI_DEBUG_PRINT(F, ...) Serial.printf("WiFi: " F, ##__VA_ARGS__)
  #define WIFI_DEBUG_PRINTLN(F, ...) Serial.printf("WiFi: " F "\n", ##__VA_ARGS__)
#else
  #define WIFI_DEBUG_PRINT(...) {}
  #define WIFI_DEBUG_PRINTLN(...) {}
#endif
