#include "Beebo.h"

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>
#include <esp_ota_ops.h>
DataStore store(SPIFFS, rtc_clock);

/* GLOBAL OBJECTS */
StdRNG fast_rng;
SimpleMeshTables tables;
Beebo beebo(radio_driver, fast_rng, rtc_clock, tables, store);

/* END GLOBAL OBJECTS */

void halt() {
  while (1)
    ;
}

void setup() {
  Serial.begin(115200);

  board.begin();

  if (!radio_init()) {
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  SPIFFS.begin(true);
  store.begin();
  beebo.begin();

  sensors.begin();

#if ENV_INCLUDE_GPS == 1
  beebo.applyGpsPrefs();
#endif

  board.onBootComplete();

  // beebo: allocate the monitor ring last in setup, after the transports are
  // up so it only claims spare PSRAM. Deliberately BEFORE the validity mark
  // below: if the ring (or anything in setup) wedges the node, we want the
  // bootloader to roll back to the previous working firmware, not confirm a
  // broken image as healthy.
  beebo.initMonRing();

  // Confirm this firmware is healthy so the bootloader doesn't roll back.
  // No-op if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is not set.
  esp_ota_mark_app_valid_cancel_rollback();
}

void loop() {
  beebo.loop();
  sensors.loop();
  rtc_clock.tick();
}
