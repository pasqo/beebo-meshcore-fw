#pragma once
#include <cstdint>

#define NODE_ROLE_COMPANION   0
#define NODE_ROLE_REPEATER    1

// beebo: companion-role-only settings with no real upstream companion_radio
// equivalent -- extracted out of NodePrefs.h to keep that struct limited to
// fields shared with real upstream examples/companion_radio/NodePrefs.h (see
// beebo/plans/COMMONCLI_TEXT_DISPATCH.md). Persisted to its own file,
// /beebo_companion, independent of /node_prefs.
struct BeeboCompanionPrefs {  // persisted to /beebo_companion
  uint8_t radio_fem_rxgain = 0; // LoRa FEM LNA RX gain (0=bypass, 1=LNA on)
  char wifi_ssid[64] = {0};   // runtime WiFi SSID (empty = WiFi disabled; set via CMD_SET_WIFI_CREDS)
  char wifi_pwd[64] = {0};    // runtime WiFi password
  uint8_t ble_enabled = 1;  // bring up BLE transport at startup (default 1)
  uint8_t tcp_enabled = 0;  // bring up WiFi/TCP transport at startup (default 0)
  uint8_t usb_enabled = 1;  // bring up USB companion transport at startup (default 1)
  uint8_t monring_config = 0; // MonRing: bit0=RX bit1=TX bit2=RADIO bit3=ENV bit4=BATT cap, bit7=enabled (default 0x97)
  float adc_multiplier = 0.0f; // battery ADC divider multiplier override; 0.0f = use board default
  uint8_t adc_resolution_bits = 12; // battery ADC sample resolution, bits (10 or 12); 0 = use board default
  uint16_t batt_sample_period_secs = 0; // battery resample period, seconds; 0 = default (BATT_SAMPLE_PERIOD_DEFAULT_SECS)
  uint16_t batt_sample_window_secs = 0; // seconds before period elapses to start trying for a radioIsIdle() sample; 0 = default (BATT_SAMPLE_WINDOW_DEFAULT_SECS)
  uint8_t batt_present = 0; // user-declared "has a real battery installed"; 0=unset/unknown, 1=no, 2=yes
  uint16_t batt_charged_mv = 0; // voltage at which a CHARGING trend reports CHARGED; 0 = default (BATT_FULL_MV_DEFAULT)
  uint16_t idle_margin_ms = 0; // margin radioIsIdle() waits past the last RX/TX before sampling Vbat; 0 = default (IDLE_MARGIN_DEFAULT_MS)
  uint8_t node_role = 0; // NODE_ROLE_* -- 0 (NODE_ROLE_COMPANION) is the implicit default on a device whose prefs predate this field
  uint32_t dedup_window_ms = 0; // window [ms] within which a dedup-table eviction still risks an undetected duplicate; 0 = disabled (a real, literal setting, not "use default" -- see SimpleMeshTables.h)
  char owner_password[16] = {0}; // beebo: SETTINGS_ISOLATION follow-up -- role-agnostic backup/recovery credential (node.owner.password), present on every build (static companion/repeater included) unlike ComPrefs.password, which only exists when BEEBO_ENABLE_REPEATER_ROLE=1. Empty = unset (there's no compiled-in default, unlike repeater.password's ADMIN_PASSWORD). Write-only -- see UNLOCK_ROLE_SECRETS in protocol.yaml.
  char board_name[32] = {0}; // beebo: DEVICE_NAME plan -- human-editable alias for this physical board's identity (board.id, the read-only eFuse MAC), distinct from the per-role persona names (companion.name/repeater.name). Present on every build, same reasoning as owner_password. Empty = unset; `beebo config`'s default filenames fall back to board.id's hex string until this is set.
  double node_lat = 0.0, node_lon = 0.0; // beebo: companion's own PREFS-mode advert_loc_policy coordinate (ADVERT_LOC_PREFS) -- what CMD_SET_ADVERT_LATLON/BEEBO_CMD_SET_COMPANION_LAT/LON/"set lat"/"lon" write. Distinct from the shared, role-agnostic sensors.node_lat/node_lon (the SHARE-mode live-GPS register, persisted separately as this struct's own folded-in NodePrefs-tail node_lat/node_lon in DataStore.cpp -- see that file's own comment). Mirrors ComPrefs.node_lat/node_lon (repeater's own copy), which real upstream CommonCLI.h/simple_repeater's NodePrefs has always had; real upstream companion_radio/NodePrefs.h never had an equivalent field at all (confirmed: no node_lat/node_lon member), which is why this lives here in BeeboCompanionPrefs instead of being bare-appended to the shared examples/companion_radio/NodePrefs.h struct.
};
