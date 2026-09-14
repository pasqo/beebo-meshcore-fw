#ifdef ESP_PLATFORM

#include "ESP32Board.h"
#include "DebugLog.h"

#ifdef BEEBO_RTC_PERSIST
#include <Preferences.h>

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

// beebo: OFFSET is milliseconds (not seconds) as of
// plans/MS_PRECISION_CLOCK_ANCHOR.md -- see applyDriftOffset_()'s own
// comment for why the boot-time correction needs that precision to avoid
// reintroducing the same error a millisecond-precision sync removed.
uint32_t beebo_clockDrift(uint32_t offset) {
  Preferences prefs;
  if (prefs.begin("beebo", false)) {
    prefs.putUInt("drift_off", offset);
    prefs.end();
  }
  DLOGH(DLOG_ID_CLOCK_DRIFT_SET, "drift_ms=%u", offset);
  return offset;
}

uint32_t beebo_clockDrift() {
  Preferences prefs;
  uint32_t curr = 0;
  if (prefs.begin("beebo", true)) {
    curr = prefs.getUInt("drift_off", 0);
    prefs.end();
  }
  return curr;
}
#else
// beebo: BEEBO_RTC_PERSIST off -- no NVM to save to/read from, so these
// fall back to plain no-ops/upstream-equivalent behavior (RTC just
// free-runs across a power cycle, drift never recorded) instead of
// requiring every caller to #ifdef around calling them at all.
void beebo_persistRTCTimeForReboot(uint32_t ts) { }
void beebo_persistRTCTimeForReboot() { }
uint32_t beebo_clockDrift(uint32_t offset) { return offset; }
uint32_t beebo_clockDrift() { return 0; }
#endif

// beebo: applies the last-known drift offset (see
// plans/CLOCK_DRIFT_COMPENSATION.md) to whatever getCurrentTime() currently
// reads. Called from both ESP_RST_UNKNOWN (where that reading is the RTC
// counter's own still-ticking, just-uncorrected value -- see BUGS.md's WiFi
// Protocol/clock investigation) and ESP_RST_POWERON (layered on top of the
// saved_ts restore). No-op if no offset has been recorded yet, e.g. a fresh
// device.
//
// beebo: drift_off is milliseconds (plans/MS_PRECISION_CLOCK_ANCHOR.md) --
// applied via settimeofday()'s tv_usec directly (bypassing RTCClock's own
// seconds-only setCurrentTime()), so a device that accumulated ahead-drift
// while running with a millisecond-precision anchor comes back up from
// this boot-time correction at the same precision, instead of a coarse
// whole-second correction reintroducing up to ~999ms of error right here,
// undone only once the next connect resyncs it.
//
// beebo: declared unconditionally (ESP32Board.h) -- BEEBO_RTC_PERSIST
// guards NVM read/write only, so with it off this is a plain no-op below
// (nothing was ever persisted to apply) rather than every caller needing
// its own #ifdef around calling this at all.
#ifdef BEEBO_RTC_PERSIST
void ESP32RTCClock::applyDriftOffset_() {
  Preferences prefs;
  if (prefs.begin("beebo", true)) {
    bool has_offset = prefs.isKey("drift_off");
    uint32_t drift_offset_ms = prefs.getUInt("drift_off", 0);
    prefs.end();
    if (has_offset) {
      time_t device_now_t;
      time(&device_now_t);
      uint64_t device_now_ms = (uint64_t)device_now_t * 1000;
      uint32_t applied = 0;
      if (device_now_ms > drift_offset_ms) {
        uint64_t corrected_ms = device_now_ms - drift_offset_ms;
        struct timeval tv;
        tv.tv_sec = corrected_ms / 1000;
        tv.tv_usec = (corrected_ms % 1000) * 1000;
        settimeofday(&tv, NULL);
        applied = drift_offset_ms;
      }
      // beebo: clear the recorded offset once consumed -- otherwise the same
      // stale drift gets re-applied (and re-subtracted) on every subsequent
      // boot that hits this function, compounding indefinitely instead of
      // being a one-time correction. Cleared even when the device_now_t
      // guard above didn't fire, since a stale/inapplicable offset is just
      // as wrong to keep around for the next boot.
      if (prefs.begin("beebo", false)) {
        prefs.remove("drift_off");
        prefs.end();
      }
      // beebo: one RLOG_ID_CLOCK_SYNC(applied) covers the whole boot-time
      // correction -- deferred (see takePendingDriftLog()'s comment) since
      // this runs from begin(), before MonRing/DebugLog's sink exists.
      _pending_drift_log = true;
      _pending_drift_offset = applied;
    }
  }
}
#else
void ESP32RTCClock::applyDriftOffset_() { }
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
