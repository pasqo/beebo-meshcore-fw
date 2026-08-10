#pragma once

#include <Arduino.h>

#define MAX_FRAME_SIZE  176   // +4 for transport codes (region scoping)

// beebo: OTA_CHUNK_SIZE is the BEEBO_CMD_OTA_WRITE payload target for
// high-bandwidth transports (WiFi, USB) -- a multiple of the ESP32 flash
// sector size (4096) so esp_ota_write() calls land cleanly on sector
// boundaries instead of straddling two sectors on every chunk. First tried
// at 8192 before SerialWifiInterface::checkRecvFrame() accumulated frame
// bodies incrementally -- that all-or-nothing read strategy silently
// depended on the frame fitting inside ESP32 lwIP's default TCP socket
// receive buffer (true at 4098 bytes, false at 8194), deadlocking TCP OTA
// on the very first chunk. Now that it accumulates across calls (see that
// file's own comment), a bigger chunk here is safe to retry.
//
// 8192 measured 60kB/s -> 85kB/s over TCP with the fixed accumulation path
// (USB unaffected either way -- different class, its own ~47kB/s ceiling).
// 16384 was tried next and doesn't fit: these buffers (this file's
// callers' rx_buf/cmd_frame/_recv_body_buf, plus main.cpp's
// setRxBufferSize()) are static internal-DRAM allocations, not PSRAM --
// PlatformIO's build-time "RAM: 2097152 bytes" figure is misleading here,
// it's not the real internal-DRAM budget these live in. 16384 overflowed
// dram0_0_seg by ~9KB at link time. 8192 is the practical ceiling for this
// buffering approach without moving these buffers to PSRAM explicitly.
//
// The actual wire frame carries a 2-byte [CMD_BEEBO][BEEBO_CMD_OTA_WRITE]
// header ahead of that payload, so buffers/getMaxRecvFrameSize() must size
// for OTA_FRAME_SIZE, not OTA_CHUNK_SIZE -- the firmware then negotiates
// back chunk_size = getMaxRecvFrameSize() - 2, recovering the clean
// OTA_CHUNK_SIZE.
#define OTA_CHUNK_SIZE  8192
#define OTA_FRAME_SIZE  (OTA_CHUNK_SIZE + 2)

// beebo: BULK_XFER (opt-in via -D BULK_XFER) lets high-bandwidth transports
// (WiFi/USB) send larger response frames and stream bulk drains (e.g. the RX
// capture ring) back-to-back, cutting the request/response round-trips that
// otherwise dominate transfer time. Off by default: MAX_SEND_FRAME_SIZE stays
// MAX_FRAME_SIZE and every wire behaviour is unchanged. Tune BULK_XFER_MAX_TX
// (bytes per TX frame on a capable transport) here or per-env with -D.
#ifdef BULK_XFER
  #ifndef BULK_XFER_MAX_TX
    #define BULK_XFER_MAX_TX 2048
  #endif
  #define MAX_SEND_FRAME_SIZE  BULK_XFER_MAX_TX
#else
  #define MAX_SEND_FRAME_SIZE  MAX_FRAME_SIZE
#endif

class BaseSerialInterface {
protected:
  BaseSerialInterface() { }

public:
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void disconnectActive() { disable(); enable(); }
  virtual bool isEnabled() const = 0;

  virtual bool isConnected() const = 0;

  // beebo: TLOG_XPORT_* id of whichever sub-transport currently holds the
  // session, or 0 if none/not applicable. Only MultiSerialInterface tracks
  // this meaningfully; single-transport interfaces can ignore it.
  virtual uint8_t activeTransportType() const { return 0; }

  // beebo: independent power-draw correlates for BattRecord.flags -- unlike
  // activeTransportType(), BLE/TCP and USB can be up at the same time (USB is
  // registered non-exclusive), so these are two separate queries rather than
  // one enum. is24GUp() covers BLE/TCP (~75mA idle draw each); isUsbUp()
  // covers USB/Serial. Only MultiSerialInterface tracks these meaningfully;
  // single-transport interfaces can ignore them.
  virtual bool is24GUp() const { return false; }
  virtual bool isUsbUp() const { return false; }

  // beebo: true if the frame most recently returned by checkRecvFrame() came
  // in as an unframed text-CLI line rather than the binary <len><data>
  // envelope. Only DualModeSerialInterface can say yes -- every other
  // interface is binary-only, so the default is a safe "no". Lets a caller
  // that supports both (e.g. multi_role's MyMesh::checkSerialInterface)
  // dispatch on real transport state instead of guessing from frame content,
  // which can't be made collision-proof against future opcode IDs.
  virtual bool lastRecvWasText() const { return false; }

  // Maximum number of bytes this transport can deliver in a single incoming frame.
  // BLE is limited by MTU; WiFi and USB can handle large OTA chunks.
  virtual size_t getMaxRecvFrameSize() const { return MAX_FRAME_SIZE; }

  // beebo: max bytes this transport can send in one outgoing frame. Default
  // MAX_FRAME_SIZE (BLE, USB-serial); WiFi overrides upward under BULK_XFER.
  // Callers clamp bulk responses to this so a limited transport never gets an
  // over-sized frame. Equals MAX_FRAME_SIZE for everyone when BULK_XFER is off.
  virtual size_t getMaxSendFrameSize() const { return MAX_FRAME_SIZE; }

  virtual bool isWriteBusy() const = 0;
  virtual size_t writeFrame(const uint8_t src[], size_t len) = 0;
  // dest must point to a buffer of at least max_len bytes. max_len defaults
  // to MAX_FRAME_SIZE so pre-existing single-arg call sites are unaffected;
  // callers that want a wider bound (e.g. BULK_XFER) pass it explicitly.
  virtual size_t checkRecvFrame(uint8_t dest[], size_t max_len = MAX_FRAME_SIZE) = 0;
};
