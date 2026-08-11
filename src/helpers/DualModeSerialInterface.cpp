#include "DualModeSerialInterface.h"
#include <string.h>

void DualModeSerialInterface::enable() {
  _isEnabled = true;
  _state = MODE_IDLE;
  rx_len = 0;
  _last_byte_at = millis();
}
void DualModeSerialInterface::disable() {
  _isEnabled = false;
}

bool DualModeSerialInterface::isConnected() const {
  return true;   // no way of knowing, so assume yes
}

bool DualModeSerialInterface::isWriteBusy() const {
  return false;
}

bool DualModeSerialInterface::feedTextByte(int c, uint8_t dest[], size_t max_len, size_t& outLen) {
  if (c == '\n') return false;   // matches legacy simple_repeater loop: never buffered/echoed
  if (c == '\r') {
    // matches legacy loop exactly: '\r' itself is echoed (it's buffered like
    // any other char there), then a separate '\n' terminates the echoed line.
    _serial->write((uint8_t)'\r');
    _serial->print('\n');

    outLen = rx_len > max_len ? max_len : rx_len;
    memcpy(dest, rx_buf, outLen);
    rx_len = 0;
    _lastWasText = true;
    _state = MODE_IDLE;  // reset: next byte is freshly reclassified, not "still text"
    return true;
  }
  _serial->write((uint8_t)c);
  if (rx_len < sizeof(rx_buf) - 1) rx_buf[rx_len++] = (uint8_t)c;
  return false;
}

size_t DualModeSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  if (_lastWasText) {
    if (len == 0) return 0;   // legacy loop stayed silent on empty replies
    _serial->print("  -> ");
    _serial->write(src, len);
    _serial->print("\r\n");   // matches legacy loop's Serial.println(reply)
    return len;
  }

  if (len > MAX_SEND_FRAME_SIZE) {
    // frame is too big! (see getMaxSendFrameSize()'s override -- this check
    // must stay in lockstep with what that advertises, same as
    // SerialWifiInterface::writeFrame()'s own MAX_SEND_FRAME_SIZE check)
    return 0;
  }

  uint8_t hdr[3];
  hdr[0] = '>';
  hdr[1] = (len & 0xFF);  // LSB
  hdr[2] = (len >> 8);    // MSB

  _serial->write(hdr, 3);
  return _serial->write(src, len);
}

size_t DualModeSerialInterface::checkRecvFrame(uint8_t dest[], size_t max_len) {
  if (_state != MODE_IDLE && millis() - _last_byte_at > RESYNC_TIMEOUT_MS) {
    // no terminator arrived in time -- discard the stale partial command
    // and resync, rather than staying stuck ignoring real traffic.
    rx_len = 0;
    _state = MODE_IDLE;
  }

  // beebo: MODE_FRAMED_BODY bulk-reads the payload instead of consuming it
  // through the byte-at-a-time loop below. A ~4KB OTA_WRITE chunk read one
  // byte per _serial->read() call (each a locked ring-buffer pop on the
  // ESP32-S3's native USB CDC) measured ~4x slower than TCP's single bulk
  // client.readBytes() for the same size -- the frame length is already
  // known once we're in this state, so there's no reason to pay that
  // per-byte overhead the way the length-less text-mode path still has to.
  // Checked at the top of every outer-loop iteration (not just once before
  // it) so the LEN2->FRAMED_BODY transition below falls straight into bulk
  // reading within the same call instead of consuming one more body byte
  // through the single-byte path first.
  while (true) {
    if (_state == MODE_FRAMED_BODY) {
      // Only ever reads bytes _serial->available() already reports ready,
      // never more -- readBytes() blocks (up to its stream timeout) if
      // asked for more than is buffered, which would stall loop()'s other
      // work.
      while (rx_len < _frame_len) {
        int avail = _serial->available();
        if (avail <= 0) break;
        size_t remaining = _frame_len - rx_len;
        size_t room = rx_len < max_len ? max_len - rx_len : 0;
        size_t want = remaining < room ? remaining : room;
        if ((size_t)avail < want) want = (size_t)avail;
        if (want == 0) {
          // over max_len -- drain and discard the rest, one byte at a time
          // (rare: only when a caller's max_len is smaller than the frame).
          int c = _serial->read();
          if (c < 0) break;
          rx_len++;
          _last_byte_at = millis();
          continue;
        }
        int got = _serial->readBytes(&rx_buf[rx_len], want);
        if (got <= 0) break;
        rx_len += got;
        _last_byte_at = millis();
      }
      if (rx_len < _frame_len) return 0;   // still waiting on more bytes
      size_t out_len = _frame_len > max_len ? max_len : _frame_len;
      memcpy(dest, rx_buf, out_len);
      _state = MODE_IDLE;
      return out_len;
    }

    if (!_serial->available()) return 0;
    int c = _serial->read();
    if (c < 0) return 0;
    _last_byte_at = millis();

    switch (_state) {
      case MODE_IDLE:
        if (c == '<') {
          _state = MODE_FRAMED_LEN1;
        } else {
          rx_len = 0;
          _state = MODE_TEXT;
          size_t n;
          if (feedTextByte(c, dest, max_len, n)) return n;
        }
        break;
      case MODE_TEXT: {
        size_t n;
        if (feedTextByte(c, dest, max_len, n)) return n;
        break;
      }
      case MODE_FRAMED_LEN1:
        _frame_len = (uint8_t)c;   // LSB
        _state = MODE_FRAMED_LEN2;
        break;
      case MODE_FRAMED_LEN2:
        _frame_len |= ((uint16_t)c) << 8;   // MSB
        rx_len = 0;
        _lastWasText = false;
        _state = _frame_len > 0 ? MODE_FRAMED_BODY : MODE_IDLE;
        break;
      case MODE_FRAMED_BODY:
        break;   // unreachable -- handled at the top of this loop
    }
  }
}
