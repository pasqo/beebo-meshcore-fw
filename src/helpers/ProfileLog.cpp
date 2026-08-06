#if ARDUINO
#include <Arduino.h>   // millis()/micros(), used inline in the header
#endif
#include "ProfileLog.h"

// Single definition for the shared profiling ring, same rationale as
// transport_log in TransportLog.cpp -- lives here so every firmware that
// links a PROFILE_SCOPE call site resolves the symbol, and builds that never
// reference it have the object stripped by linker GC.
ProfileLog profile_log;
