#pragma once

#include <Mesh.h>
#include <Arduino.h>
#include <Wire.h>

class AutoDiscoverRTCClock : public mesh::RTCClock {
  mesh::RTCClock* _fallback;

  bool i2c_probe(TwoWire& wire, uint8_t addr);
public:
  AutoDiscoverRTCClock(mesh::RTCClock& fallback) : _fallback(&fallback) { }

  void begin(TwoWire& wire);
  uint32_t getCurrentTime() override;
  void setCurrentTime(uint32_t time) override;
  // beebo: no detected I2C RTC chip has sub-second precision to offer, so
  // these only widen the no-chip-found case -- deferring straight to
  // _fallback's own getTime()/setTime() (ESP32RTCClock, when present) for
  // real ms precision, rather than the base class default (which would
  // round-trip through getCurrentTime()/setCurrentTime() here and lose it).
  void getTime(uint32_t &secs, uint16_t &ms) override;
  void setTime(uint32_t secs, uint16_t ms) override;

  void tick() override {
    _fallback->tick();   // is typically VolatileRTCClock, which now needs tick()
  }
};
