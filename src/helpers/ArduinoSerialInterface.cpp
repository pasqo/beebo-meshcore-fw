#include "ArduinoSerialInterface.h"

#define RECV_STATE_IDLE        0
#define RECV_STATE_HDR_FOUND   1
#define RECV_STATE_LEN1_FOUND  2
#define RECV_STATE_LEN2_FOUND  3

void ArduinoSerialInterface::enable() { 
  _isEnabled = true;
  _state = RECV_STATE_IDLE;
}
void ArduinoSerialInterface::disable() {
  _isEnabled = false;
}

bool ArduinoSerialInterface::isConnected() const { 
  return true;   // no way of knowing, so assume yes
}

bool ArduinoSerialInterface::isWriteBusy() const {
  return false;
}

// Stream::write(buf, size) is allowed to short-write (return less than
// requested) rather than block -- e.g. the USB CDC peripheral's own TX
// buffer being momentarily full under rapid back-to-back replies, with no
// setTxBufferSize call on this core's USBCDC class (see main.cpp's own
// comment on why). A short write here silently truncates the frame with no
// way for the host to recover: its own byte-parser just waits forever for
// bytes that were never sent, stalling the command for its full timeout.
// Retry the remainder until the whole buffer is out.
//
// beebo: same fix as DualModeSerialInterface.cpp's own writeAll() (see its
// comment for the full writeup, including the live wire-capture evidence
// for both failure modes below) -- CONFIG_TINYUSB_CDC_TX_BUFSIZE is only 64
// bytes on this core, so any non-trivial reply needs several retries here.
// Two fixes: (1) yield() on every iteration that still made progress, so a
// never-yielding busy-spin can't starve whatever drains the TX buffer of
// the CPU time it needs; (2) retry through a *zero*-byte return too instead
// of giving up on the very first one -- a zero return can be a momentary
// refusal, not necessarily a lost connection, and giving up immediately
// silently truncates the frame. This copy isn't on multi_role's own code
// path (Beebo.h uses DualModeSerialInterface, not this class) but is live
// for companion_radio and any other board still using ArduinoSerialInterface
// directly, so it gets the identical fix.
// beebo: see DualModeSerialInterface.cpp's own writeAll() comment for why
// this is 3000ms, not a short window -- same reasoning applies here.
static const uint32_t ZERO_WRITE_GIVEUP_MS = 3000;

static size_t writeAll(Stream* serial, const uint8_t* buf, size_t len) {
  size_t written = 0;
  uint32_t zero_since_ms = 0;
  while (written < len) {
    size_t n = serial->write(buf + written, len - written);
    if (n == 0) {
      uint32_t now = millis();
      if (zero_since_ms == 0) zero_since_ms = now;
      if (now - zero_since_ms >= ZERO_WRITE_GIVEUP_MS) break;  // sustained refusal -- treat as lost connection
      yield();
      continue;
    }
    zero_since_ms = 0;
    written += n;
    if (written < len) yield();  // let TX-draining work run before the next retry
  }
  return written;
}

size_t ArduinoSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    // frame is too big!
    return 0;
  }

  uint8_t hdr[3];
  hdr[0] = '>';
  hdr[1] = (len & 0xFF);  // LSB
  hdr[2] = (len >> 8);    // MSB

  if (writeAll(_serial, hdr, 3) < 3) return 0;
  return writeAll(_serial, src, len);
}

size_t ArduinoSerialInterface::checkRecvFrame(uint8_t dest[], size_t max_len) {
  while (_serial->available()) {
    int c = _serial->read();
    if (c < 0) break;

    switch (_state) {
      case RECV_STATE_IDLE:
        if (c == '<') {
          _state = RECV_STATE_HDR_FOUND;
        }
        break;
      case RECV_STATE_HDR_FOUND:
        _frame_len = (uint8_t)c;   // LSB
        _state = RECV_STATE_LEN1_FOUND;
        break;
      case RECV_STATE_LEN1_FOUND:
        _frame_len |= ((uint16_t)c) << 8;   // MSB
        rx_len = 0;
        _state = _frame_len > 0 ? RECV_STATE_LEN2_FOUND : RECV_STATE_IDLE;
        break;
      default:
        if (rx_len < max_len) {
          rx_buf[rx_len] = (uint8_t)c;   // rest of frame will be discarded if > max_len
        }
        rx_len++;
        if (rx_len >= _frame_len) {  // received a complete frame?
          if (_frame_len > max_len) _frame_len = max_len;    // truncate
          memcpy(dest, rx_buf, _frame_len);
          _state = RECV_STATE_IDLE;  // reset state, for next frame
          return _frame_len;
        }
    }
  }
  return 0;
}
