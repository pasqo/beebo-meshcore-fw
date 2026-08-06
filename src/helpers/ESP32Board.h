#pragma once

#include <MeshCore.h>
#include <Arduino.h>

#ifndef USER_BTN_PRESSED
#define USER_BTN_PRESSED LOW
#endif

#if defined(ESP_PLATFORM)

#include <rom/rtc.h>
#include <sys/time.h>
#include <Wire.h>
#include "soc/rtc.h"
#include "esp_system.h"

// beebo: opt-in (-D BEEBO_RTC_PERSIST) persistence of the RTC across a true
// power cycle (ESP_RST_POWERON), where settimeofday()'s state is lost. Soft
// reboots (esp_restart()) already keep the RTC alive on ESP32-S3, so this is
// only read back on power-on. Off by default: upstream behavior (always
// falling back to the fixed recent-past date below) is unchanged.
#ifdef BEEBO_RTC_PERSIST
#include <Preferences.h>
static void beebo_persistRTCTimeForReboot(uint32_t ts) {
  Preferences prefs;
  if (prefs.begin("beebo", false)) {
    prefs.putULong("rtc_ts", ts);
    prefs.end();
  }
}
static void beebo_persistRTCTimeForReboot() {
  time_t now;
  time(&now);
  beebo_persistRTCTimeForReboot((uint32_t)now);
}
#endif

class ESP32Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;
  bool inhibit_sleep = false;
  static inline portMUX_TYPE sleepMux = portMUX_INITIALIZER_UNLOCKED;

public:
  void begin() {
    // for future use, sub-classes SHOULD call this from their begin()
    startup_reason = BD_STARTUP_NORMAL;    

  #ifdef ESP32_CPU_FREQ
    setCpuFrequencyMhz(ESP32_CPU_FREQ);
  #endif

  #ifdef PIN_VBAT_READ
    // battery read support
    pinMode(PIN_VBAT_READ, INPUT);
    adcAttachPin(PIN_VBAT_READ);
  #endif

  #ifdef P_LORA_TX_LED
    pinMode(P_LORA_TX_LED, OUTPUT);
    digitalWrite(P_LORA_TX_LED, LOW);
  #endif

  #if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
   #if PIN_BOARD_SDA >= 0 && PIN_BOARD_SCL >= 0
    Wire.begin(PIN_BOARD_SDA, PIN_BOARD_SCL);
   #endif
  #else
    Wire.begin();
  #endif    
  }

  // Temperature from ESP32 MCU
  float getMCUTemperature() override {
    uint32_t raw = 0;

    // To get and average the temperature so it is more accurate, especially in low temperature
    for (int i = 0; i < 4; i++) {
      raw += temperatureRead();
    }

    return raw / 4;
  }

  uint32_t getIRQGpio() override {
    return P_LORA_DIO_1; // default for SX1262
  }

  void sleep(uint32_t secs) override {
    // Skip if not allow to sleep
    if (inhibit_sleep) {
      delay(1); // Give MCU to OTA to run
      return;
    }

    // Set GPIO wakeup
    gpio_num_t wakeupPin = (gpio_num_t)getIRQGpio();    

    // Configure timer wakeup
    if (secs > 0) {
      esp_sleep_enable_timer_wakeup(secs * 1000000ULL); // Wake up periodically to do scheduled jobs
    }

    // Disable CPU interrupt servicing
    portENTER_CRITICAL(&sleepMux);

    // Skip sleep if there is a LoRa packet
    if (gpio_get_level(wakeupPin) == HIGH) {
      portEXIT_CRITICAL(&sleepMux);
      delay(1);
      return;
    }

    // Configure GPIO wakeup
    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable((gpio_num_t)wakeupPin, GPIO_INTR_HIGH_LEVEL); // Wake up when receiving a LoRa packet

    // MCU enters light sleep
    esp_light_sleep_start();

    // Avoid ISR flood during wakeup due to HIGH LEVEL interrupt
    gpio_wakeup_disable(wakeupPin);
    gpio_set_intr_type(wakeupPin, GPIO_INTR_POSEDGE);

    // Enable CPU interrupt servicing
    portEXIT_CRITICAL(&sleepMux);
  }

  uint8_t getStartupReason() const override { return startup_reason; }

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED off
  }
#elif defined(P_LORA_TX_NEOPIXEL_LED)
  #define NEOPIXEL_BRIGHTNESS    64  // white brightness (max 255)

  void onBeforeTransmit() override {
    neopixelWrite(P_LORA_TX_NEOPIXEL_LED, NEOPIXEL_BRIGHTNESS, NEOPIXEL_BRIGHTNESS, NEOPIXEL_BRIGHTNESS);   // turn TX neopixel on (White)
  }
  void onAfterTransmit() override {
    neopixelWrite(P_LORA_TX_NEOPIXEL_LED, 0, 0, 0);   // turn TX neopixel off
  }
#endif

  uint16_t getBattMilliVolts() override {
    #ifdef PIN_VBAT_READ
    analogReadResolution(12);

    uint32_t raw = 0;
    for (int i = 0; i < 4; i++) {
      raw += analogReadMilliVolts(PIN_VBAT_READ);
    }
    raw = raw / 4;

    return (2 * raw);
  #else
    return 0;  // not supported
  #endif
  }

  const char* getManufacturerName() const override {
    return "Generic ESP32";
  }

  void reboot() override {
#ifdef BEEBO_RTC_PERSIST
    beebo_persistRTCTimeForReboot();
#endif
    esp_restart();
  }

  // beebo: same as reboot(), but for a device whose own clock is wrong (so
  // an ordinary reboot() would just re-persist the same bad value forever,
  // and a soft reset doesn't reload NVS anyway -- ESP32RTCClock::begin()
  // only does that on a true ESP_RST_POWERON, see its own comment). Uses
  // the given timestamp (the caller's own clock, e.g. the connected host's)
  // for both: persists it to NVS for a genuine future power cycle, AND
  // corrects the live clock immediately -- safe specifically because
  // nothing runs between that and esp_restart() right below, so no
  // in-flight timer/deadline logic (advert/ACK/retry timers, MonRing
  // sequencing) ever gets a chance to observe the jump; the full restart
  // wipes all of that state regardless. Setting the live clock and NOT
  // rebooting immediately after would not be safe -- don't split these.
  void rebootWithTime(uint32_t ts) {
#ifdef BEEBO_RTC_PERSIST
    beebo_persistRTCTimeForReboot(ts);
#endif
    struct timeval tv;
    tv.tv_sec = ts;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    esp_restart();
  }

  bool startOTAUpdate(const char* id, char reply[]) override;

  void setInhibitSleep(bool inhibit) {
    inhibit_sleep = inhibit;
  }
};

class ESP32RTCClock : public mesh::RTCClock {
public:
  ESP32RTCClock() { }
  void begin() {
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_POWERON) {
      struct timeval tv;
#ifdef BEEBO_RTC_PERSIST
      // beebo: recover the timestamp saved by reboot() so a power cycle
      // only loses however long the boot itself took, not the RTC state --
      // falls back to a fixed recent-past date on the very first boot ever,
      // when nothing has been persisted yet.
      uint32_t saved_ts = 0;
      Preferences prefs;
      if (prefs.begin("beebo", true)) {
        saved_ts = prefs.getULong("rtc_ts", 0);
        prefs.end();
      }
      tv.tv_sec = saved_ts != 0 ? saved_ts : 1715770351;  // 15 May 2024, 8:50pm
#else
      // start with some date/time in the recent past
      tv.tv_sec = 1715770351;  // 15 May 2024, 8:50pm
#endif
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);
    }
  }
  uint32_t getCurrentTime() override {
    time_t _now;
    time(&_now);
    return _now;
  }
  void setCurrentTime(uint32_t time) override {
    struct timeval tv;
    tv.tv_sec = time;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
  }
};

#endif
