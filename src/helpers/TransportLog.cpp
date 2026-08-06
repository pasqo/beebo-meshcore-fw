#if ARDUINO
#include <Arduino.h>   // millis(), used by the inline log() in the header
#endif
#include "TransportLog.h"

// Single definition for the shared transport debug ring. Lives here (not in an
// application's main.cpp) so every firmware that links the transport helpers
// which log to it (esp32 SerialBLEInterface / SerialWifiInterface,
// MultiSerialInterface) resolves the symbol; builds that never reference it
// have the object stripped by linker GC.
TransportLog transport_log;
