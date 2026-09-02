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

// Discard whatever accumulated in the hardware RX buffer while this sub sat
// unpolled and locked out by another active session -- MultiSerialInterface
// never calls checkRecvFrame() on a non-active sub, so nothing in that
// buffer was ever parsed, acknowledged, or replied to. It's either the
// trailing/incomplete remains of a connection attempt the client already
// gave up on (the exact case we intend to make fail), or one it's still
// mid-retry on -- either way the client got no response and will detect the
// failure on its own; nothing here is an established, in-flight exchange
// that this would silently cut off. Left undrained, the first stale byte
// gets misread as a fresh command once polling resumes: a '<' starts a
// phantom binary-frame wait, anything else drops into MODE_TEXT and gets
// echoed back out the port by feedTextByte().
void DualModeSerialInterface::discardStaleRx() {
  while (_serial->available()) _serial->read();
}


bool DualModeSerialInterface::isConnected() const {
  // beebo: an idle-liveness inference, not a real hardware signal --
  // there is no USB-level "the far side hung up" event for a client that
  // dies without closing anything cleanly (crash, SIGKILL, cable pulled
  // while the OS/hub side stays enumerated). ANDed into the USB
  // connected_fn alongside (bool)Serial (Beebo.cpp's addInterface() calls
  // -- HWCDC's own real SOF-based physical-disconnect detection, kept
  // as-is) so either signal going false is enough to release a wedged
  // session: (bool)Serial catches an actual unplug, this catches a
  // process that went silent with the cable still attached. A byte
  // arriving via any path updates _last_byte_at (enable(), the framed/
  // text parser, pollRawControl()'s session-less raw control frame --
  // see connect.py's periodic BEEBO_RAW_SUB_KEEPALIVE write during an
  // otherwise-idle `beebo -i` session), so a genuinely idle-but-alive
  // session never trips this on its own.
  return millis() - _last_byte_at < USB_IDLE_TIMEOUT_MS;
}

bool DualModeSerialInterface::pollRawControl(uint8_t& sub_id, uint8_t& data) {
  if (_state != MODE_IDLE) return false;
  if (_serial->peek() != RAW_MARKER) return false;
  if (_serial->available() < 3) return false;   // wait for the rest to arrive, don't consume yet

  _serial->read();   // the marker itself
  sub_id = (uint8_t)_serial->read();
  data = (uint8_t)_serial->read();
  _last_byte_at = millis();
  return true;
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

// Stream::write(buf, size) is allowed to short-write (return less than
// requested) rather than block -- e.g. the USB CDC peripheral's own TX
// buffer being momentarily full under rapid back-to-back replies, with no
// setTxBufferSize call on this core's USBCDC class (see main.cpp's own
// comment on why). A short write here silently truncates the frame with no
// way for the host to recover: its own byte-parser just waits forever for
// bytes that were never sent, stalling the command for its full timeout.
// Retry the remainder until the whole buffer is out.
//
// beebo: CONFIG_TINYUSB_CDC_TX_BUFSIZE is only 64 bytes on this core (no
// Arduino-level setTxBufferSize to raise it, per the comment above) -- any
// reply bigger than that (every beebo reply past a trivial OK/ERR: a
// 176-byte GET_STATS page, up to a 2048-byte BULK_XFER page) needs many
// retry iterations here, each one only able to push whatever room has
// opened up in that 64-byte ring since the last try.
//
// Two distinct failure modes were found here, both via a live wire capture
// (host-side TX/RX byte trace) of a real `no_event_received` repro:
//
// 1. A never-yielding busy-spin on a *partial* write (n > 0 but n < len)
//    can starve whatever drains the TX ring of the CPU time it needs (that
//    draining work happens off the loop()/Arduino task on this core),
//    turning what should be a few sub-millisecond USB packet transfers into
//    a stall lasting hundreds of ms -- confirmed via MonRing's
//    EVENT_LOOP_STALL diagnostic: captured LOOP_SEG_ROLE_LOOP (loopRepeater(),
//    which calls writeFrame() via checkSerialInterface()) blocking
//    304-603ms. Fixed by yield()ing every iteration that still made
//    progress, letting that draining work actually run between retries.
//
// 2. A *zero* return (peripheral momentarily not accepting ANY bytes) used
//    to `break` immediately, on the very first occurrence, silently
//    abandoning the rest of the frame -- exactly the truncation this
//    function's own top comment warns about, on the assumption that a zero
//    return only happens for "the peripheral stops accepting bytes
//    entirely (already a lost-connection condition)". That assumption is
//    wrong: a live capture caught a real frame (168 bytes) that stopped
//    dead at 104 bytes with no further bytes EVER arriving, while the next
//    request 15s later (the host's own retry) got a complete, correct
//    reply -- proof the peripheral recovered fine and this was a momentary,
//    not permanent, refusal. Fixed by retrying through zero returns too,
//    the same as partial ones, up to ZERO_WRITE_GIVEUP_MS of continuous
//    zero-returns before finally giving up (a bound still needed for an
//    actually-lost connection, e.g. USB physically unplugged mid-transfer).
// beebo: 200ms was found insufficient by live repro -- a real wire capture
// caught a frame stopping dead at 104 of a declared 168 bytes, with the
// remaining bytes never arriving, well past that window. No caller of
// writeFrame() can recover from a short write once the length header is
// already on the wire (that header commits to a length before this
// function even starts on the body -- see writeFrame() below), and every
// caller except two (both now fixed to compare against the full requested
// length, not just non-zero) silently ignores writeFrame()'s return value
// entirely. So the only real backstop against a corrupted, un-recoverable
// frame is making the give-up threshold generous enough that it's a rare,
// genuine-disconnect-only event -- a multi-second loop() delay here is a
// far better outcome than a truncated frame, which costs the client its
// full protocol timeout (15s) with no way to recover at all.
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

size_t DualModeSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  if (_lastWasText) {
    if (len == 0) return 0;   // legacy loop stayed silent on empty replies
    _serial->print("  -> ");
    writeAll(_serial, src, len);
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

  if (writeAll(_serial, hdr, 3) < 3) return 0;
  return writeAll(_serial, src, len);
}

// beebo: single-attempt counterpart to writeFrame() above -- see this
// method's own declaration comment in BaseSerialInterface.h for why
// DebugLog needs this instead of writeFrame()'s retry-until-sent
// writeAll(). The naive fix (just call write() once, no retry loop) turned
// out not to be enough on real hardware: on the native USB-Serial-JTAG
// peripheral (HWCDC.cpp), a single write() call itself blocks internally
// (up to tx_timeout_ms, default 100ms) whenever its TX ring buffer is
// full and isCDC_Connected() reads true -- and isCDC_Connected() gets
// reasserted by any USB bus activity (HWCDC.cpp's own ISR sets
// `connected = true` on every incoming packet), not by whether an
// application is actually reading, so it stays "connected" for as long as
// the cable is plugged in, host or no host reader. With DEBUG_LOG_ENABLE
// on and nothing draining USB, the ring buffer fills once and stays full,
// so every subsequent push -- one per TransportLog event, which includes
// CMD_RECV/CMD_DONE for every companion command on *every* transport --
// paid that ~100ms internally, back-to-back, on the same single-threaded
// main loop that also services BLE/TCP: confirmed on real hardware as the
// actual cause of TCP requests timing out while USB stayed responsive
// (whichever session's own read() was the one draining bytes). The real
// fix is availableForWrite() (Arduino's own non-blocking free-space
// query, unaffected by the same isCDC_Connected() staleness since it
// never calls the blocking retry path itself) -- skip the write
// entirely, no attempt at all, once there isn't room, rather than call
// write() and eat its internal stall.
size_t DualModeSerialInterface::writeFrameBestEffort(const uint8_t src[], size_t len) {
  if (_lastWasText || len > MAX_SEND_FRAME_SIZE) return 0;
  if ((size_t)_serial->availableForWrite() < 3 + len) return 0;

  uint8_t hdr[3];
  hdr[0] = '>';
  hdr[1] = (len & 0xFF);
  hdr[2] = (len >> 8);

  if (_serial->write(hdr, 3) < 3) return 0;
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
