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

  if (len > MAX_FRAME_SIZE) {
    // frame is too big!
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

  while (_serial->available()) {
    int c = _serial->read();
    if (c < 0) break;
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
        if (rx_len < max_len) {
          rx_buf[rx_len] = (uint8_t)c;   // rest of frame will be discarded if > max_len
        }
        rx_len++;
        if (rx_len >= _frame_len) {  // received a complete frame?
          if (_frame_len > max_len) _frame_len = max_len;    // truncate
          memcpy(dest, rx_buf, _frame_len);
          _state = MODE_IDLE;  // reset state, for next command
          return _frame_len;
        }
        break;
    }
  }
  return 0;
}
