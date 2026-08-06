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

  #define FRAME_QUEUE_SIZE  4
  int recv_queue_len;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  int send_queue_len;
  SendFrame send_queue[FRAME_QUEUE_SIZE];
  int _send_off;   // bytes of the head send frame (3-byte header + body) already written
  uint32_t _send_queue_full_count = 0;

  void clearBuffers() { recv_queue_len = 0; send_queue_len = 0; _send_off = 0; }

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

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;
  bool isWriteBusy() const override;

  size_t getMaxRecvFrameSize() const override { return OTA_CHUNK_SIZE; }
  size_t getMaxSendFrameSize() const override { return MAX_SEND_FRAME_SIZE; }
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override;

  bool hasReceivedFrameHeader();
  void resetReceivedFrameHeader();

  // beebo: lifetime count of writeFrame() silently dropping a frame because
  // send_queue (FRAME_QUEUE_SIZE=4) was full. No recv-side equivalent -- WiFi
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
