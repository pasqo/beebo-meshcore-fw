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
// beebo: non-static (unlike the rest of this file's small helpers) so
// ESP32Board.cpp's definition can log RLOG_ID_CLOCK_NVM_PUSH -- this header
// deliberately stays free of DebugLog.h (compiled for many non-beebo board
// environments), same reasoning as beebo_clockDrift()'s own comment
// below. Called from reboot()/rebootWithTime() (below) AND from every
// accepted clock-set call site (CommonCLI.cpp, Beebo.cpp) so rtc_ts stays
// fresh across an ordinary sync, not just an explicit reboot -- see
// kbase/CLOCK_DRIFT_COMPENSATION.md's "Behavior across reset kinds".
void beebo_persistRTCTimeForReboot(uint32_t ts);
void beebo_persistRTCTimeForReboot();

// beebo: beebo_clockDrift(offset) persists OFFSET as the new drift_off and
// returns it; beebo_clockDrift() (no args) reads back whatever's currently
// persisted, read-only -- 0 if never recorded or already cleared. Same
// set-vs-read overload split as beebo_persistRTCTimeForReboot() above.
// Used directly by Beebo.cpp (which already includes DebugLog.h/RLOGM) to
// log RLOG_ID_CLOCK_SYNC itself; CommonCLI.cpp uses
// beebo_logCurrentClockDrift() below instead, since it's shared code that
// doesn't pull in DebugLog.h.
uint32_t beebo_clockDrift(uint32_t offset);
uint32_t beebo_clockDrift();

// beebo: reads the current persisted drift_off and logs RLOG_ID_CLOCK_SYNC
// with it -- for callers (CommonCLI.cpp) that can't call RLOGM directly
// (this is shared code compiled for boards with no DebugLog.h at all).
// Called on a no-op clock query (nothing was corrected this call), so a
// client watching --debug sees this device's persisted-drift state on
// every "clock"/sync, whether or not that particular call changed it. For
// a call that DID just correct or reject a sync, log the immediate signed
// delta via beebo_logClockDrift() below instead -- it says what just
// happened, not what's still pending from a previous rejection.
void beebo_logCurrentClockDrift();

// beebo: same shared-code reason as beebo_logCurrentClockDrift() above --
// logs RLOG_ID_CLOCK_SYNC(delta) directly, for a caller that already knows
// the exact signed delta of what just happened: negative when a sync was
// just accepted and applied (curr - secs, curr < secs), positive when one
// was just rejected and persisted for a future boot to apply (curr - secs,
// curr > secs).
void beebo_logClockDrift(int32_t delta);
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
  // beebo: TS_MS (0-999, default 0) is the sub-second component of TS from
  // a millisecond-precision sync (plans/MS_PRECISION_CLOCK_ANCHOR.md) --
  // only refines the live-clock correction's tv_usec below; rtc_ts (the
  // NVS backup) stays seconds-only, unaffected.
  void rebootWithTime(uint32_t ts, uint16_t ts_ms = 0) {
#ifdef BEEBO_RTC_PERSIST
    beebo_persistRTCTimeForReboot(ts);
#endif
    struct timeval tv;
    tv.tv_sec = ts;
    tv.tv_usec = (long)ts_ms * 1000;
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
  // beebo: floor epoch used both as the last-resort fallback date below and
  // as the "does the live RTC already look real" plausibility check in
  // begin() -- a genuine RTC-domain power loss leaves time() reading near 0
  // (1970), always well before this; an RTC that survived (even across a
  // reset esp_reset_reason() reports as ESP_RST_POWERON -- see begin()'s own
  // comment) reads some real epoch, always well after this, since real
  // epochs only get newer over the life of this firmware.
  static constexpr uint32_t RTC_FALLBACK_EPOCH = 1715770351;  // 15 May 2024, 8:50pm

  // beebo: numeric values mirror DebugLog.h's RLOG_CLOCK_SRC_* exactly --
  // kept as plain local constants here (not an include of DebugLog.h)
  // since this header stays free of the debug-log system, compiled for
  // boards with no such thing at all. bootClockSource() below hands the
  // raw value back to a caller (Beebo.cpp) that already has DebugLog.h and
  // can translate it to the real RLOG_CLOCK_SRC_* name directly.
  static constexpr uint8_t CLOCK_SRC_RTC = 0;
  static constexpr uint8_t CLOCK_SRC_NVM = 1;
  static constexpr uint8_t CLOCK_SRC_FALLBACK = 2;

  void begin() {
    esp_reset_reason_t reason = esp_reset_reason();
    // beebo: esp_reset_reason() reports ESP_RST_POWERON for both a genuine
    // cold power-on (RTC domain power lost, time() reads ~0) and, on some
    // hardware, an EN/RST-button toggle that leaves the RTC domain powered
    // (time() still holds whatever was last set) -- there's no way to tell
    // these apart from the reset reason alone. Whether an RST-button press
    // on *this* board actually falls into the second case is still being
    // verified on real hardware as of 2026-09-12 (RLOG_ID_CLOCK_RTC below
    // exists specifically to get direct evidence, since a client watching
    // --debug has no independent way to know what time() held here without
    // it). Guards against the case it does apply -- only falls back to
    // NVS/the fixed date when the live clock doesn't already look
    // plausible (i.e. this really was a cold boot).
    time_t live_now;
    time(&live_now);
#ifdef BEEBO_RTC_PERSIST
    _pending_clock_rtc_log = true;
    _pending_clock_rtc_value = (uint32_t)live_now;
#endif
    bool live_clock_plausible = (uint32_t)live_now > RTC_FALLBACK_EPOCH;
    if (reason == ESP_RST_POWERON && !live_clock_plausible) {
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
      tv.tv_sec = saved_ts != 0 ? saved_ts : RTC_FALLBACK_EPOCH;
#else
      // start with some date/time in the recent past
      tv.tv_sec = RTC_FALLBACK_EPOCH;
#endif
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);
#ifdef BEEBO_RTC_PERSIST
      // beebo: deferred the same way RLOG_ID_CLOCK_SYNC is
      // (see takePendingDriftLog()'s comment) -- this runs before MonRing's
      // sink exists. detail = what was actually applied (tv.tv_sec), so a
      // client can tell a genuine cold boot (this event present, likely
      // stale) apart from an RST-button press where the live clock was
      // trusted instead (no CLOCK_NVM_PULL logged this boot at all).
      _pending_nvm_pull_log = true;
      _pending_nvm_pull_value = (uint32_t)tv.tv_sec;
      // beebo: deferred alongside CLOCK_NVM_PULL, logged next to CLOCK_SET
      // at the same point (see Beebo::startMonRing()) -- distinguishes a
      // genuine NVS restore from the fixed-date fallback used on a device
      // that's never persisted anything yet.
      _pending_clock_src = saved_ts != 0 ? CLOCK_SRC_NVM : CLOCK_SRC_FALLBACK;
      applyDriftOffset_();
#endif
    }
#ifdef BEEBO_RTC_PERSIST
    // beebo: ESP_RST_UNKNOWN covers an esptool flash; ESP_RST_SW covers an
    // ordinary reboot() (esp_restart()); a plausible-live-clock
    // ESP_RST_POWERON covers an RST-button toggle (see above) -- the RTC
    // counter survives all three intact (never powered down), so all three
    // need the same drift correction. Found missing 2026-09-09: `beebo
    // clock`'s own "Reboot the device to correct it" message pointed at a
    // plain reboot, which this branch never covered -- the RTC free-ran
    // straight through it, uncorrected.
    else if (reason == ESP_RST_UNKNOWN || reason == ESP_RST_SW ||
             (reason == ESP_RST_POWERON && live_clock_plausible)) {
      _pending_clock_src = CLOCK_SRC_RTC;
      applyDriftOffset_();
    }
#endif
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

#ifdef BEEBO_RTC_PERSIST
  // beebo: applyDriftOffset_() below runs from begin(), called via
  // clock_init() before MonRing exists at all -- an RLOGM() call there could
  // never reach MON_DEBUG (DebugLog's sink isn't wired yet). It stashes the
  // event here instead; the caller (Beebo::startMonRing(), right after
  // wiring that sink) takes it and logs it then. The two calls are
  // microseconds apart in practice (both run at the very top of setup()),
  // so the deferred RLOGM's own millis()-stamped timestamp is still
  // effectively the real one.
  // beebo: true once applyDriftOffset_() has run (has_offset was true, i.e.
  // a drift_off key existed and was cleared) -- out_offset is the amount
  // actually subtracted from the live clock (0 if the device_now_t >
  // drift_offset guard didn't pass). One RLOG_ID_CLOCK_SYNC(out_offset)
  // covers the whole boot-time correction; false (nothing to log) when
  // applyDriftOffset_() found no drift_off key at all.
  bool takePendingDriftLog(uint32_t *out_offset) {
    if (!_pending_drift_log) return false;
    _pending_drift_log = false;
    *out_offset = _pending_drift_offset;
    return true;
  }

  // beebo: same deferred-logging reason as the two above -- true once
  // begin() has restored rtc_ts (or the fixed fallback) from NVS on a
  // genuine cold-boot ESP_RST_POWERON, meaning the caller should log
  // RLOG_ID_CLOCK_NVM_PULL once MonRing's sink exists.
  bool takePendingNvmPull(uint32_t *out_value) {
    if (!_pending_nvm_pull_log) return false;
    _pending_nvm_pull_log = false;
    *out_value = _pending_nvm_pull_value;
    return true;
  }

  // beebo: unlike the other takePending*() calls above, this one is always
  // available after begin() runs (every boot sets it to exactly one of
  // RLOG_CLOCK_SRC_RTC/NVM/FALLBACK) -- read, not consumed/cleared, since
  // Beebo::startMonRing() logs it paired with every RLOG_ID_CLOCK_SET it
  // emits, including ones from runtime commands long after boot (which pass
  // RLOG_CLOCK_SRC_SYNC directly instead of calling this).
  uint8_t bootClockSource() const { return _pending_clock_src; }

  // beebo: deferred the same way takePendingNvmPull() above is -- true
  // every single boot (set unconditionally at the top of begin(), before
  // any branch runs), unlike the other takePending*() calls which are
  // conditional on which path begin() took.
  bool takePendingClockRtc(uint32_t *out_value) {
    if (!_pending_clock_rtc_log) return false;
    _pending_clock_rtc_log = false;
    *out_value = _pending_clock_rtc_value;
    return true;
  }
#endif

private:
#ifdef BEEBO_RTC_PERSIST
  void applyDriftOffset_();
  bool _pending_drift_log = false;
  uint32_t _pending_drift_offset = 0;
  uint8_t _pending_clock_src = CLOCK_SRC_RTC;
  bool _pending_nvm_pull_log = false;
  uint32_t _pending_nvm_pull_value = 0;
  bool _pending_clock_rtc_log = false;
  uint32_t _pending_clock_rtc_value = 0;
#endif
};

#endif
