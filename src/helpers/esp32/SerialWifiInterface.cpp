#include "SerialWifiInterface.h"
#include <WiFi.h>
#include <lwip/sockets.h>   // send(MSG_DONTWAIT) for the non-blocking queue drain
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
  return false;
}

bool SerialWifiInterface::hasReceivedFrameHeader() {
  return received_frame_header.type != 0 && received_frame_header.length != 0;
}

void SerialWifiInterface::resetReceivedFrameHeader() {
  received_frame_header.type = 0;
  received_frame_header.length = 0;
}

size_t SerialWifiInterface::checkRecvFrame(uint8_t dest[], size_t max_len) {
  // check if new client connected
  auto newClient = server.available();
  if (newClient) {
    transport_log.log(TLOG_WIFI_CLIENT_NEW, deviceConnected ? 1 : 0);
    deviceConnected = false;
    client.stop();
    clearBuffers();
    resetReceivedFrameHeader();

    client = newClient;
    client.setNoDelay(true);
  }

  if (client.connected()) {
    if (!deviceConnected) {
      transport_log.log(TLOG_WIFI_SESSION_ON);
      deviceConnected = true;
    }
  } else {
    if (deviceConnected) {
      client.stop();
      deviceConnected = false;
      clearBuffers();
      resetReceivedFrameHeader();
      transport_log.log(TLOG_WIFI_SESSION_OFF);
    }
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

      // make sure we have received enough bytes for the required frame length
      int available = client.available();
      int frame_type = received_frame_header.type;
      int frame_length = received_frame_header.length;
      if(frame_length > available){
        WIFI_DEBUG_PRINTLN("Waiting for %d more bytes", frame_length - available);
        return 0;
      }

      // skip frames that exceed the buffer the caller provided
      if ((size_t)frame_length > max_len) {
        WIFI_DEBUG_PRINTLN("Skipping frame: length=%d exceeds max_len=%d", frame_length, (int)max_len);
        while (frame_length > 0) {
          uint8_t skip[1];
          int skipped = client.read(skip, 1);
          frame_length -= skipped;
        }
        resetReceivedFrameHeader();
        return 0;
      }

      // skip frames that are not expected type
      // '<' is 0x3c which indicates a frame sent from app to radio
      if(frame_type != '<'){
        WIFI_DEBUG_PRINTLN("Skipping frame: type=0x%x is unexpected", frame_type);
        while (frame_length > 0) {
          uint8_t skip[1];
          int skipped = client.read(skip, 1);
          frame_length -= skipped;
        }
        resetReceivedFrameHeader();
        return 0;
      }

      // read frame data to provided buffer
      client.readBytes(dest, frame_length);

      // ready for next frame
      resetReceivedFrameHeader();
      return frame_length;

    }
  }

  return 0;
}

bool SerialWifiInterface::isConnected() const {
  return deviceConnected;  //pServer != NULL && pServer->getConnectedCount() > 0;
}