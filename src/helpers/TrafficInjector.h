#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// beebo: companion traffic injector state and decisions (no Arduino
// dependency, native-testable). Arrivals are a Poisson process at the target
// rate; an arrival sends only while the node's own moving RX rate is below the
// target. See kbase/CHANNELS_AND_TRAFFIC.md.
class TrafficInjector {
public:
  static constexpr uint8_t NO_CHANNEL = 0xFF;
  static constexpr uint32_t MIN_GAP_MS = 2000;    // caps the send rate at 30 a minute
  static constexpr uint32_t MAX_GAP_MS = 120000;
  static constexpr uint8_t TEXT_SIZE = 32;

  bool active() const { return _channel != NO_CHANNEL && _rate != 0; }
  uint8_t channel() const { return _channel; }
  uint8_t channelHash() const { return _hash; }
  uint8_t rate() const { return _rate; }
  uint32_t sent() const { return _sent; }

  void setChannel(uint8_t idx, uint8_t hash) { _channel = idx; _hash = hash; _sent = 0; }
  void clearChannel() { _channel = NO_CHANNEL; }
  void setRate(uint8_t r) { _rate = r; }

  // rx_per_min: the node's moving 60 s RX rate; window_full: 60 s of history.
  bool shouldSend(uint32_t rx_per_min, bool window_full) const {
    return active() && window_full && rx_per_min < _rate;
  }

  uint32_t nextSeq() { return ++_sent; }

  // Exponential gap, mean 60 s / rate, from u16 in 1..65535 (0 = never sends).
  static uint32_t gapMs(uint8_t rate, uint16_t u) {
    if (rate == 0 || u == 0) return MAX_GAP_MS;
    float g = -logf(u / 65536.0f) * 60000.0f / rate;
    if (g < MIN_GAP_MS) return MIN_GAP_MS;
    if (g > MAX_GAP_MS) return MAX_GAP_MS;
    return (uint32_t)g;
  }

  // Counter padded with dots to size bytes (same as beebo traffic); returns size.
  static int text(char* buf, uint32_t seq, int size) {
    int n = snprintf(buf, size + 1, "%06u", (unsigned)(seq % 1000000));
    memset(buf + n, '.', size - n);
    buf[size] = 0;
    return size;
  }

private:
  uint8_t _channel = NO_CHANNEL;
  uint8_t _hash = 0;
  uint8_t _rate = 0;
  uint32_t _sent = 0;
};
