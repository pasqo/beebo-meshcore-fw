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
  // beebo: raising the baud rate alone (115200 -> 921600, matching esptool's
  // own flashing speed) turned out not to be the real USB OTA bottleneck --
  // measured no change. This board's ARDUINO_USB_MODE=1 build uses the
  // ESP32-S3's USB-Serial-JTAG hardware peripheral (HWCDC), the same one
  // esptool's ROM bootloader uses to flash quickly over this exact port --
  // so the limit isn't the port/peripheral itself. HWCDC's default RX ring
  // buffer is small (a few hundred bytes); raw host-side writes measured
  // blocking for ~500ms once it fills and loop() hasn't drained it in time,
  // which a small buffer makes far more likely under real mesh/radio load
  // sharing the loop. Widen it to comfortably hold a full OTA frame so a
  // burst of chunk data has somewhere to land between loop() iterations.
  // Confirmed by measurement: widening RX alone took USB OTA from ~14kB/s to
  // ~45kB/s. 2x headroom in case loop() jitter still occasionally exceeds
  // one frame's worth of slack. No setTxBufferSize on this core's USBCDC
  // class (ARDUINO_USB_MODE=1's Serial is USBCDC, not HWCDC) -- OK/ERR
  // replies are tiny (a few bytes) anyway, unlikely to be TX-bound.
  Serial.setRxBufferSize(OTA_FRAME_SIZE * 2);
  Serial.begin(921600);

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
