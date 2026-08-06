#include <Arduino.h>
#include "target.h"

HeltecV4Board board;

#if defined(P_LORA_SCLK)
  static SPIClass spi;
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif

WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);

#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  EnvironmentSensorManager sensors;
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display(NULL);
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

bool radio_init() {
  fallback_clock.begin();
#ifndef HELTEC_LORA_V4_BARE
  // The bare/headless board has no external RTC chip; skip the I2C probe
  // entirely rather than risk a false-positive ACK on floating SDA/SCL
  // (no OLED soldered means nothing else holds those lines) silently
  // routing every getCurrentTime()/setCurrentTime() call to a phantom
  // chip instead of the working ESP32RTCClock fallback. Each such call
  // re-reads/re-writes over I2C, so a floating-bus false ACK causes the
  // clock to read back correctly on some calls and garbage on others --
  // exactly the "syncs, then reverts a few calls later, no reboot
  // involved" symptom seen on gatto. Leaving rtc_clock.begin() uncalled
  // keeps every AutoDiscoverRTCClock *_success flag false, so it already
  // falls through to _fallback on its own.
  rtc_clock.begin(Wire);
#endif

#if defined(P_LORA_SCLK)
  return radio.std_init(&spi);
#else
  return radio.std_init();
#endif
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

