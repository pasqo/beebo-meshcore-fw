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

// beebo: what kind of frame checkRecvFrame() just delivered -- decided
// fresh per frame, from the transport's own parser state, not guessed from
// frame content (opcode IDs aren't fully under this project's control,
// shared with upstream MeshCore, and could collide with any fixed
// byte-range heuristic). BINARY is the default/only kind for every
// transport except DualModeSerialInterface, which can also deliver TEXT
// (an unframed text-CLI line) or DEBUG (a session-less raw control
// sub-frame -- BEEBO_RAW_SUB_DBG_ENABLE/BEEBO_RAW_SUB_TIME_SYNC,
// DebugLog.h -- carried as a real 2-byte [sub_id][data] payload in dest,
// not an app command).
enum class RecvFrameType : uint8_t { BINARY, TEXT, DEBUG };

class BaseSerialInterface {
protected:
  BaseSerialInterface() { }

public:
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void disconnectActive() { disable(); enable(); }
  virtual bool isEnabled() const = 0;

  // beebo: drop whatever's sitting in this transport's hardware RX buffer
  // without touching enabled/parser state. For a byte-stream transport
  // (DualModeSerialInterface) that just stopped being polled -- either it
  // was the active sub and its session ended, or it sat locked out the
  // whole time another sub was active -- anything still buffered was never
  // parsed/acknowledged, so it's stale by construction. No-op default for
  // transports with no such buffer to leak stale bytes from.
  virtual void discardStaleRx() { }

  // beebo: reset per-session parser/buffer state (byte-parser position,
  // queued frames, ...) without a real enable/disable transition -- used by
  // MultiSerialInterface::release() purely to leave a released sub's parser
  // clean for its next session, distinct from an actual power/enabled-state
  // change. Deliberately not disable()+enable(): that also logs a real
  // transport disable/enable event and (for SerialWifiInterface) needlessly
  // tears down and rebinds the listen socket -- neither of which reflects
  // anything that actually happened. No-op default for transports with no
  // such state to reset.
  virtual void resetParserState() { }

  virtual bool isConnected() const = 0;

  // beebo: RLOG_ID_XPORT_* id of whichever sub-transport currently holds the
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

  // beebo: best-effort variant of writeFrame() for a caller that would
  // rather lose a push than block -- currently only DebugLog (see its own
  // comment). writeFrame()'s retry-until-sent behavior is the right
  // tradeoff for a real companion reply (losing bytes there corrupts an
  // in-flight exchange the client has no way to recover from), but wrong
  // for lossy, best-effort debug telemetry: retrying a doomed write for
  // seconds at a time blocks the single-threaded main loop that also
  // services every other transport, turning an unread debug stream into a
  // stall affecting completely unrelated traffic (confirmed on real
  // hardware -- see DebugLog.cpp's own comment). Default just forwards to
  // writeFrame(), so every interface keeps its current behavior unless it
  // overrides this; DualModeSerialInterface is the one that actually needs
  // to, since isConnected()/isWriteBusy() are both unconditional stubs on
  // it (documented in DualModeSerialInterface.cpp) and can't be used to
  // gate a push instead.
  virtual size_t writeFrameBestEffort(const uint8_t src[], size_t len) { return writeFrame(src, len); }
  // dest must point to a buffer of at least max_len bytes. max_len defaults
  // to MAX_FRAME_SIZE so pre-existing single-arg call sites are unaffected;
  // callers that want a wider bound (e.g. BULK_XFER) pass it explicitly.
  // type, if non-null, is set to this call's RecvFrameType whenever a frame
  // is actually returned (n > 0) -- every transport except
  // DualModeSerialInterface always sets it to BINARY. Defaulted to nullptr
  // so pre-existing two-arg call sites are unaffected.
  virtual size_t checkRecvFrame(uint8_t dest[], size_t max_len = MAX_FRAME_SIZE, RecvFrameType* type = nullptr) = 0;
};
