// beebo: the only source of env:heltec_v4_3_hybrid_prep (see its comment in
// variants/heltec_v4/platformio.ini). Weak, because the dummy pass that
// compiles the IDF libs already carries its own setup()/loop().
#include <Arduino.h>

__attribute__((weak)) void setup() {}
__attribute__((weak)) void loop() {}
