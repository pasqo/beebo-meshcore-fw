#ifdef ESP_PLATFORM

#include "ESP32Board.h"
#include "DebugLog.h"

#ifdef BEEBO_RTC_PERSIST
void beebo_persistRTCTimeForReboot(uint32_t ts) {
  Preferences prefs;
  if (prefs.begin("beebo", false)) {
    prefs.putULong("rtc_ts", ts);
    prefs.end();
  }
  RLOGH(RLOG_ID_CLOCK_NVM_PUSH, (int32_t)ts);
}

void beebo_persistRTCTimeForReboot() {
  time_t now;
  time(&now);
  beebo_persistRTCTimeForReboot((uint32_t)now);
}

void beebo_recordClockDrift(uint32_t offset) {
  Preferences prefs;
  if (prefs.begin("beebo", false)) {
    prefs.putUInt("drift_off", offset);
    prefs.end();
  }
  RLOGH(RLOG_ID_CLOCK_DRIFT, (int32_t)offset);
}

uint32_t beebo_currentClockDrift() {
  Preferences prefs;
  uint32_t offset = 0;
  if (prefs.begin("beebo", true)) {
    offset = prefs.getUInt("drift_off", 0);
    prefs.end();
  }
  return offset;
}

void beebo_logCurrentClockDrift() {
  RLOGH(RLOG_ID_CLOCK_DRIFT, (int32_t)beebo_currentClockDrift());
}

// beebo: applies the last-known drift offset (see
// plans/CLOCK_DRIFT_COMPENSATION.md) to whatever getCurrentTime() currently
// reads. Called from both ESP_RST_UNKNOWN (where that reading is the RTC
// counter's own still-ticking, just-uncorrected value -- see BUGS.md's WiFi
// Protocol/clock investigation) and ESP_RST_POWERON (layered on top of the
// saved_ts restore). No-op if no offset has been recorded yet, e.g. a fresh
// device.
void ESP32RTCClock::applyDriftOffset_() {
  Preferences prefs;
  if (prefs.begin("beebo", true)) {
    bool has_offset = prefs.isKey("drift_off");
    uint32_t drift_offset = prefs.getUInt("drift_off", 0);
    prefs.end();
    if (has_offset) {
      time_t device_now_t;
      time(&device_now_t);
      if (device_now_t > drift_offset) {
        struct timeval tv;
        tv.tv_sec = device_now_t - drift_offset;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        _pending_drift_log = true;   // logged later by Beebo::startMonRing() -- see takePendingDriftLog()'s comment
        _pending_drift_offset = drift_offset;
      }
      // beebo: clear the recorded offset once consumed -- otherwise the same
      // stale drift gets re-applied (and re-subtracted) on every subsequent
      // boot that hits this function, compounding indefinitely instead of
      // being a one-time correction. Cleared even when the device_now_t
      // guard above didn't fire, since a stale/inapplicable offset is just
      // as wrong to keep around for the next boot. Logging the clear as
      // CLOCK_DRIFT(0) is deferred the same way CLOCK_DRIFT_APPLIED is
      // (see _pending_drift_log's own comment) -- this runs from begin(),
      // before MonRing/DebugLog's sink exists, so an RLOGH() here would
      // silently go nowhere.
      if (prefs.begin("beebo", false)) {
        prefs.remove("drift_off");
        prefs.end();
      }
      _pending_drift_cleared = true;
    }
  }
}
#endif

#if defined(ADMIN_PASSWORD) && !defined(DISABLE_WIFI_OTA)   // Repeater or Room Server only
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <SPIFFS.h>

bool ESP32Board::startOTAUpdate(const char* id, char reply[]) {
  inhibit_sleep = true;   // prevent sleep during OTA

#ifdef BEEBO_WIFI_OTA_STA
  // beebo: Web OTA rides the existing STA connection only (softAP is not
  // supported); decided at runtime, not by WIFI_SSID, so builds that bring
  // WiFi up from saved prefs work too and a downed link reports an error
  // instead of 0.0.0.0.
  if (WiFi.status() != WL_CONNECTED) {
    strcpy(reply, "Error: WiFi not connected");
    return false;
  }
  sprintf(reply, "Started: http://%s/update", WiFi.localIP().toString().c_str());
#else
  WiFi.softAP("MeshCore-OTA", NULL);
  sprintf(reply, "Started: http://%s/update", WiFi.softAPIP().toString().c_str());
#endif
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

#endif
