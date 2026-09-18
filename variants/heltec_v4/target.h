#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <HeltecV4Board.h>
#include <helpers/radiolib/CustomSX1262Wrapper.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/EnvironmentSensorManager.h>
#ifdef DISPLAY_CLASS
#ifdef HELTEC_LORA_V4_OLED
    #include <helpers/ui/SSD1306Display.h>
#elif defined(HELTEC_LORA_V4_TFT)
    #include <helpers/ui/ST7789LCDDisplay.h>
#endif
  #include <helpers/ui/MomentaryButton.h>
#endif

extern HeltecV4Board board;
extern WRAPPER_CLASS radio_driver;
extern AutoDiscoverRTCClock rtc_clock;
extern ESP32RTCClock fallback_clock;  // beebo: needed directly (not just via rtc_clock) for takePendingNvmPull()/takePendingClockRtc()/bootClockSource()
extern EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  extern DISPLAY_CLASS display;
  extern MomentaryButton user_btn;
#endif

void clock_init();  // beebo: RTC-only subset of radio_init(), safe to call before board.begin() -- see target.cpp
bool radio_init();
mesh::LocalIdentity radio_new_identity();

