#include "HeltecV4Board.h"

void HeltecV4Board::begin() {
    ESP32Board::begin();


#ifndef HELTEC_V4_R8   // beebo: R8 has no ADC-control gate, VBAT is read directly
    pinMode(PIN_ADC_CTRL, OUTPUT);
    digitalWrite(PIN_ADC_CTRL, LOW); // Initially inactive
#endif

    loRaFEMControl.init();
    // beebo: report the auto-detected LoRa front-end (PA) in the boot log.
    Serial.printf("FEM: %s\n",
        loRaFEMControl.getFEMType() == KCT8103L_PA ? "KCT8103L (V4.3)" :
        loRaFEMControl.getFEMType() == GC1109_PA   ? "GC1109 (V4.2)"   : "unknown");

    periph_power.begin();
#ifdef HELTEC_V4_R8
    periph_power.claim();  // beebo: R8 VEXT also feeds the LoRa antenna boost rail, keep it on
#endif
    // beebo: PSRAM mode (quad vs octal) is fixed by the build's memory_type, so an R2
    // image on an R8 board (or the reverse) can't be told apart by pins alone -- flag it.
    Serial.printf("PSRAM: %u KB (built for %s)\n", (unsigned)(ESP.getPsramSize() / 1024),
#ifdef HELTEC_V4_R8
        "R8, expects >= 6144 KB");
    if (ESP.getPsramSize() < 6u * 1024 * 1024) Serial.println("WARNING: R8 build but PSRAM is not 8MB octal -- wrong image for this board");
#else
        "R2, expects <= 4096 KB");
    if (ESP.getPsramSize() > 4u * 1024 * 1024) Serial.println("WARNING: R2 build but PSRAM is larger than 2MB -- wrong image for this board");
#endif
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      long wakeup_source = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_source & (1 << P_LORA_DIO_1)) {  // received a LoRa packet (while in deep sleep)
        startup_reason = BD_STARTUP_RX_PACKET;
    }

      rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
      rtc_gpio_deinit((gpio_num_t)P_LORA_DIO_1);
    }
  }

  void HeltecV4Board::onBeforeTransmit(void) {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
    loRaFEMControl.setTxModeEnable();
  }

  void HeltecV4Board::onAfterTransmit(void) {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
    loRaFEMControl.setRxModeEnable();
  }

  void HeltecV4Board::powerOff() {
    // Turn off PA
    digitalWrite(P_LORA_PA_POWER, LOW);
    rtc_gpio_hold_en((gpio_num_t)P_LORA_PA_POWER);

    ESP32Board::powerOff();
  }

  uint16_t HeltecV4Board::getBattMilliVolts()  {
    analogReadResolution(adc_resolution);
#ifdef HELTEC_V4_R8
    // beebo: R8 has no ADC gate; ADC_MULTIPLIER is the divider ratio applied to calibrated millivolts
    uint32_t mv = 0;
    for (int i = 0; i < 8; i++) {
      mv += analogReadMilliVolts(PIN_VBAT_READ);
    }
    return adc_mult * (mv / 8);
#else
    digitalWrite(PIN_ADC_CTRL, HIGH);
    delay(10);
    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;

    digitalWrite(PIN_ADC_CTRL, LOW);

    uint32_t full_scale = 1UL << adc_resolution;
    return (adc_mult * (3.3 / full_scale) * raw) * 1000;
#endif
  }

  const char* HeltecV4Board::getManufacturerName() const {
#ifdef HELTEC_LORA_V4_TFT
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 TFT" : "Heltec V4 TFT";
#elif defined(HELTEC_V4_R8) && defined(HELTEC_LORA_V4_BARE)   // beebo: R8 bare board
    return "Heltec V4.3 R8 Headless";
#elif defined(HELTEC_LORA_V4_BARE)   // beebo: bare board, no OLED/TFT
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 Headless" : "Heltec V4 Headless";
#else
    return loRaFEMControl.getFEMType() == KCT8103L_PA ? "Heltec V4.3 OLED" : "Heltec V4 OLED";
#endif
  }

  bool HeltecV4Board::setLoRaFemLnaEnabled(bool enable) {
    if (!loRaFEMControl.isLnaCanControl()) {
      return false;
    }

    loRaFEMControl.setLNAEnable(enable);
    loRaFEMControl.setRxModeEnable();
    return true;
  }

  bool HeltecV4Board::canControlLoRaFemLna() const {
    return loRaFEMControl.isLnaCanControl();
  }

  bool HeltecV4Board::isLoRaFemLnaEnabled() const {
    return loRaFEMControl.isLNAEnabled();
  }
