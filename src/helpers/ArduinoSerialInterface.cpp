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
// Retry the remainder until the whole buffer is out; this can only ever
// under-run if the peripheral itself stops accepting bytes entirely (already
// a lost-connection condition handled elsewhere), not from a transient
// backlog draining at normal USB speed.
static size_t writeAll(Stream* serial, const uint8_t* buf, size_t len) {
  size_t written = 0;
  while (written < len) {
    size_t n = serial->write(buf + written, len - written);
    if (n == 0) break;   // peripheral not accepting bytes at all -- give up
    written += n;
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
