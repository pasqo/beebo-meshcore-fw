#pragma once

// Minimal native mock of the handful of Arduino globals DualModeSerialInterface.cpp
// actually calls (millis(), yield()). Time is a plain settable global so tests can
// move the clock deterministically instead of racing a real wall clock.

#include <stdint.h>
#include <stddef.h>
#include <cmath>
#include "Stream.h"   // real Arduino.h transitively brings in Stream too

inline uint32_t g_mock_millis = 0;

using std::isnan;

inline uint32_t millis() { return g_mock_millis; }
inline void yield() {}

inline void delay(uint32_t ms) {
  g_mock_millis += ms;
}
