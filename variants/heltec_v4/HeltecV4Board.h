#pragma once

#include <Arduino.h>
#include <helpers/RefCountedDigitalPin.h>
#include <helpers/ESP32Board.h>
#include <driver/rtc_io.h>
#include "LoRaFEMControl.h"

#ifndef ADC_MULTIPLIER
  #define ADC_MULTIPLIER 5.42
#endif

// beebo: battery ADC sample resolution, bits. 10-bit is the historical
// default (matches the divider calibration most beebo units shipped with);
// 12-bit is the ESP32-S3 ADC's native max, giving ~4x finer quantization
// (~4.2mV/step vs ~16.8mV/step at the default 5.42x divider) at the cost of
// exposing more of the raw analog noise floor as visible LSB wobble.
#ifndef ADC_RESOLUTION_BITS
  #define ADC_RESOLUTION_BITS 10
#endif

class HeltecV4Board : public ESP32Board {

protected:
  float adc_mult = ADC_MULTIPLIER;
  uint8_t adc_resolution = ADC_RESOLUTION_BITS;

public:
  RefCountedDigitalPin periph_power;
  LoRaFEMControl loRaFEMControl;
  HeltecV4Board() : periph_power(PIN_VEXT_EN,PIN_VEXT_EN_ACTIVE) { }

  void begin();
  void onBeforeTransmit(void) override;
  void onAfterTransmit(void) override;
  void enterDeepSleep(uint32_t secs, int pin_wake_btn = -1);
  void powerOff() override;
  bool setLoRaFemLnaEnabled(bool enable) override;
  bool canControlLoRaFemLna() const override;
  bool isLoRaFemLnaEnabled() const override;
  uint16_t getBattMilliVolts() override;
  bool setAdcMultiplier(float multiplier) override {
    if (multiplier == 0.0f) {
      adc_mult = ADC_MULTIPLIER;
    } else {
      adc_mult = multiplier;
    }
    return true;
  }
  float getAdcMultiplier() const override { return adc_mult; }
  bool setAdcResolution(uint8_t bits) override {
    if (bits != 10 && bits != 12) return false;
    adc_resolution = bits;
    return true;
  }
  uint8_t getAdcResolution() const override { return adc_resolution; }
  const char* getManufacturerName() const override;
};
