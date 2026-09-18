#ifdef ESP_PLATFORM

#include "ESP32Board.h"
#include <target.h>
#include "DebugLog.h"

#ifdef BEEBO_RTC_PERSIST
#include <Preferences.h>

void setRebootRTCTime(uint32_t ts) {
  Preferences prefs;
  if (prefs.begin("beebo", false)) {
    prefs.putULong("rtc_ts", ts);
    prefs.end();
  }
  RLOGH(RLOG_ID_CLOCK_NVM_PUSH, (int32_t)ts);
}

void setRebootRTCTime() {
  time_t now;
  time(&now);
  setRebootRTCTime((uint32_t)now);
}

uint32_t getRebootRTCTime() {
  Preferences prefs;
  uint32_t ts = 0;
  if (prefs.begin("beebo", true)) {
    ts = prefs.getULong("rtc_ts", 0);
    prefs.end();
  }
  return ts;
}

#else
// beebo: BEEBO_RTC_PERSIST off -- no NVM to save to/read from, so these
// fall back to a plain no-op/empty-read instead of requiring every caller
// to #ifdef around calling them.
void setRebootRTCTime(uint32_t ts) { }
void setRebootRTCTime() { }
uint32_t getRebootRTCTime() { return 0; }
#endif

#if defined(ADMIN_PASSWORD) && !defined(DISABLE_WIFI_OTA)   // Repeater or Room Server only
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <SPIFFS.h>

bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  inhibit_sleep = true;   // prevent sleep during OTA

  WiFi.softAP("MeshCore-OTA", NULL);
  sprintf(reply, "Started: http://%s/update", WiFi.softAPIP().toString().c_str());
  MESH_DEBUG_PRINTLN("startOTAUpdate: %s", reply);

  static char id_buf[60];
  sprintf(id_buf, "%s (%s)", id, getManufacturerName());
  static char home_buf[90];
  sprintf(home_buf, "<H2>Hi! I am a MeshCore Repeater. ID: %s</H2>", id);

  AsyncWebServer* server = new AsyncWebServer(80);

  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", home_buf);
  });
  server->on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/packet_log", "text/plain");
  });

  AsyncElegantOTA.setID(id_buf);
  AsyncElegantOTA.begin(server);    // Start ElegantOTA
  server->begin();

  return true;
}

#else
bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  return false; // not supported
}
#endif

void ESP32Board::powerOff() {
  enterDeepSleep(0); // Do not wakeup
}

void ESP32Board::enterDeepSleep(uint32_t secs) {
  // Power off the display if any
#ifdef DISPLAY_CLASS
  display.turnOff();
#endif

  // Power off LoRa
  radio_driver.powerOff();

  // Keep LoRa inactive during deepsleep
  digitalWrite(P_LORA_NSS, HIGH);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  gpio_hold_en((gpio_num_t)P_LORA_NSS);
#else
  rtc_gpio_hold_en((gpio_num_t)P_LORA_NSS);
#endif

  // Power off GPS if any
  if (sensors.getLocationProvider() != NULL) {
    sensors.getLocationProvider()->stop();
  }

  // Flush serial buffers
  Serial.flush();
  delay(100);

  // Clear stale wakeup sources to avoid ghost wakeup
  // This is required when Power Management and automatic lightsleep are enabled
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  if (secs > 0) {
    esp_sleep_enable_timer_wakeup(secs * 1000000ULL);
  }

  // Finally set ESP32 into deepsleep
  esp_deep_sleep_start(); // CPU halts here and never returns!
}
#endif
