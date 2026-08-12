#include <Arduino.h>
#include "Beebo.h"
#include "DataStore.h"

DataStore::DataStore(FILESYSTEM& fs, mesh::RTCClock& clock)
    : _fs(&fs), _fsExtra(nullptr), _clock(&clock), identity_store(fs, "/identity") {
}

static File openWrite(FILESYSTEM* fs, const char* filename) {
  return fs->open(filename, "w", true);
}

void DataStore::begin() {
  // init 'blob store' support
  _fs->mkdir("/bl");
}

#include <SPIFFS.h>
#include <nvs_flash.h>

uint32_t DataStore::getStorageUsedKb() const {
  return SPIFFS.usedBytes() / 1024;
}

uint32_t DataStore::getStorageTotalKb() const {
  return SPIFFS.totalBytes() / 1024;
}

File DataStore::openRead(const char* filename) {
  return _fs->open(filename, "r", false);
}

File DataStore::openRead(FILESYSTEM* fs, const char* filename) {
  return fs->open(filename, "r", false);
}

bool DataStore::removeFile(const char* filename) {
  return _fs->remove(filename);
}

bool DataStore::removeFile(FILESYSTEM* fs, const char* filename) {
  return fs->remove(filename);
}

bool DataStore::formatFileSystem() {
  bool fs_success = ((fs::SPIFFSFS *)_fs)->format();
  esp_err_t nvs_err = nvs_flash_erase(); // no need to reinit, will be done by reboot
  return fs_success && (nvs_err == ESP_OK);
}

bool DataStore::loadMainIdentity(mesh::LocalIdentity &identity) {
  return identity_store.load("_main", identity);
}

bool DataStore::saveMainIdentity(const mesh::LocalIdentity &identity) {
  return identity_store.save("_main", identity);
}

static const char* roleIdentityName(uint8_t role) {
  return role == NODE_ROLE_REPEATER ? "_repeater" : "_companion";
}

bool DataStore::loadRoleIdentity(uint8_t role, mesh::LocalIdentity &identity) {
  return identity_store.load(roleIdentityName(role), identity);
}

bool DataStore::saveRoleIdentity(uint8_t role, const mesh::LocalIdentity &identity) {
  return identity_store.save(roleIdentityName(role), identity);
}

bool DataStore::migrateLegacyIdentity(uint8_t role, mesh::LocalIdentity &identity) {
  if (identity_store.load(roleIdentityName(role), identity)) return false;   // role already has its own identity
  if (!identity_store.load("_main", identity)) return false;   // no legacy identity to migrate
  identity_store.save(roleIdentityName(role), identity);
  return true;
}

// beebo: SETTINGS_ISOLATION -- read-only seed from stock companion_radio's
// files. Never writes /new_prefs or /node_prefs, never deletes either --
// unlike upstream's own /node_prefs -> /new_prefs migration (which does
// both), this is a one-time read into beebo's own /beebo_companion, and
// stock's files must survive a future reflash back to stock untouched.
void DataStore::loadLegacyNodePrefs(BeeboPrefs& prefs, double& node_lat, double& node_lon) {
  if (_fs->exists("/new_prefs")) {
    loadPrefsInt("/new_prefs", prefs, node_lat, node_lon);
  } else if (_fs->exists("/node_prefs")) {
    loadPrefsInt("/node_prefs", prefs, node_lat, node_lon);
  }
}

void DataStore::loadPrefsInt(const char *filename, BeeboPrefs& _prefs, double& node_lat, double& node_lon) {
  File file = openRead(_fs, filename);
  if (file) {
    uint8_t pad[8];

    file.read((uint8_t *)&_prefs.airtime_factor, sizeof(float));                           // 0
    file.read((uint8_t *)_prefs.node_name, sizeof(_prefs.node_name));                      // 4
    file.read(pad, 4);                                                                     // 36
    file.read((uint8_t *)&node_lat, sizeof(node_lat));                                     // 40
    file.read((uint8_t *)&node_lon, sizeof(node_lon));                                     // 48
    file.read((uint8_t *)&_prefs.freq, sizeof(_prefs.freq));                               // 56
    file.read((uint8_t *)&_prefs.sf, sizeof(_prefs.sf));                                   // 60
    file.read((uint8_t *)&_prefs.cr, sizeof(_prefs.cr));                                   // 61
    file.read((uint8_t *)&_prefs.client_repeat, sizeof(_prefs.client_repeat));             // 62
    file.read((uint8_t *)&_prefs.manual_add_contacts, sizeof(_prefs.manual_add_contacts)); // 63
    file.read((uint8_t *)&_prefs.bw, sizeof(_prefs.bw));                                   // 64
    file.read((uint8_t *)&_prefs.tx_power_dbm, sizeof(_prefs.tx_power_dbm));               // 68
    file.read((uint8_t *)&_prefs.telemetry_mode_base, sizeof(_prefs.telemetry_mode_base)); // 69
    file.read((uint8_t *)&_prefs.telemetry_mode_loc, sizeof(_prefs.telemetry_mode_loc));   // 70
    file.read((uint8_t *)&_prefs.telemetry_mode_env, sizeof(_prefs.telemetry_mode_env));   // 71
    file.read((uint8_t *)&_prefs.rx_delay_base, sizeof(_prefs.rx_delay_base));             // 72
    file.read((uint8_t *)&_prefs.advert_loc_policy, sizeof(_prefs.advert_loc_policy));     // 76
    file.read((uint8_t *)&_prefs.multi_acks, sizeof(_prefs.multi_acks));                   // 77
    file.read((uint8_t *)&_prefs.path_hash_mode, sizeof(_prefs.path_hash_mode));           // 78
    file.read(pad, 1);                                                                     // 79
    file.read((uint8_t *)&_prefs.ble_pin, sizeof(_prefs.ble_pin));                         // 80
    file.read((uint8_t *)&_prefs.buzzer_quiet, sizeof(_prefs.buzzer_quiet));               // 84
    file.read((uint8_t *)&_prefs.gps_enabled, sizeof(_prefs.gps_enabled));                 // 85
    file.read((uint8_t *)&_prefs.gps_interval, sizeof(_prefs.gps_interval));               // 86
    file.read((uint8_t *)&_prefs.autoadd_config, sizeof(_prefs.autoadd_config));           // 87
    file.read((uint8_t *)&_prefs.autoadd_max_hops, sizeof(_prefs.autoadd_max_hops));       // 88
    file.read((uint8_t *)&_prefs.rx_boosted_gain, sizeof(_prefs.rx_boosted_gain));         // 89
    file.read((uint8_t *)_prefs.default_scope_name, sizeof(_prefs.default_scope_name));    // 90
    file.read((uint8_t *)_prefs.default_scope_key, sizeof(_prefs.default_scope_key));     // 121
    // next: 137

    file.close();
  }
}

// beebo: SETTINGS_ISOLATION -- /beebo_companion now also carries NodePrefs
// (companion's radio/node settings) folded in after BeeboCompanionPrefs's
// own fields, so companion settings no longer touch /new_prefs at all
// after the one-time seed (see Beebo::begin()). Returns true if the file
// already held a fully migrated copy (BeeboCompanionPrefs's own fields
// AND the NodePrefs tail) -- false means the caller should seed NodePrefs
// via loadLegacyNodePrefs().
//
// Deliberately NOT gated on plain file existence: /beebo_companion
// predates this fold-in (BeeboCompanionPrefs was already a shipped,
// deployed file format before NodePrefs was appended to it), so a real
// device's existing file is old-format -- present, but ending right after
// dedup_window_ms, with no NodePrefs tail at all. Treating "exists" as
// "fully migrated" would silently read past EOF into the node.* fields
// (Arduino's File::read() is a no-op past EOF, leaving them at whatever
// the RAM defaults already were) -- the same class of bug that broke
// gatto's WiFi in a past migration attempt (COMMONCLI_TEXT_DISPATCH.md).
// file.available() after the known old-format fields is what actually
// distinguishes old-format from new-format, with no version byte needed.
bool DataStore::loadBeeboCompanionPrefs(BeeboPrefs& _prefs, BeeboBoardPrefs& _board, double& node_lat, double& node_lon) {
  bool has_node_prefs = false;
  File file = openRead(_fs, "/beebo_companion");
  if (file) {
    file.read((uint8_t *)&_prefs.radio_fem_rxgain, sizeof(_prefs.radio_fem_rxgain));               // 0
    file.read((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));                              // 1
    file.read((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));                                // 65
    file.read((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));                         // 129
    file.read((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));                         // 130
    file.read((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));                         // 131
    file.read((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));                   // 132
    // beebo: BOARD_BATTERY_PREFS.md -- these seven now read into `_board`
    // (BeeboBoardPrefs), not `_prefs` (BeeboBasePrefs no longer declares
    // them) -- same board-scoped legacy-tail-read role as _board.role/
    // board_password/board_name below/above: a genuine migration seed
    // when /beebo_board predates these fields, an inert echo of the
    // already-authoritative /beebo_board value otherwise (see
    // saveBeeboCompanionPrefs() below and Beebo::begin()'s migration
    // ordering comment). Byte offsets/order unchanged from before.
    file.read((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));                   // 133
    file.read((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs)); // 137
    file.read((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));                       // 139
    file.read((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));         // 140
    file.read((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs)); // 141
    file.read((uint8_t *)&_board.batt_charged_mv, sizeof(_board.batt_charged_mv));                 // 143
    file.read((uint8_t *)&_board.idle_margin_ms, sizeof(_board.idle_margin_ms));                   // 145
    file.read((uint8_t *)&_board.role, sizeof(_board.role));                                       // 147 -- was node_role
    file.read((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));                 // 148
    // next: 152 -- NodePrefs (now BeeboPrefs's 14 SharedPrefs + 11 distilled
    // fields) folded in from here, own layout independent of both stock's
    // /new_prefs byte layout and CommonCLI.h's repeater NodePrefs -- only
    // this function and saveBeeboCompanionPrefs() below need to agree on
    // it. Only present in a post-fold-in file -- an old-format
    // /beebo_companion ends right here.
    has_node_prefs = file.available() > 0;
    if (has_node_prefs) {
      file.read((uint8_t *)&_prefs.airtime_factor, sizeof(_prefs.airtime_factor));
      file.read((uint8_t *)_prefs.node_name, sizeof(_prefs.node_name));
      file.read((uint8_t *)&node_lat, sizeof(node_lat));
      file.read((uint8_t *)&node_lon, sizeof(node_lon));
      file.read((uint8_t *)&_prefs.freq, sizeof(_prefs.freq));
      file.read((uint8_t *)&_prefs.sf, sizeof(_prefs.sf));
      file.read((uint8_t *)&_prefs.cr, sizeof(_prefs.cr));
      file.read((uint8_t *)&_prefs.multi_acks, sizeof(_prefs.multi_acks));
      file.read((uint8_t *)&_prefs.manual_add_contacts, sizeof(_prefs.manual_add_contacts));
      file.read((uint8_t *)&_prefs.bw, sizeof(_prefs.bw));
      file.read((uint8_t *)&_prefs.tx_power_dbm, sizeof(_prefs.tx_power_dbm));
      file.read((uint8_t *)&_prefs.telemetry_mode_base, sizeof(_prefs.telemetry_mode_base));
      file.read((uint8_t *)&_prefs.telemetry_mode_loc, sizeof(_prefs.telemetry_mode_loc));
      file.read((uint8_t *)&_prefs.telemetry_mode_env, sizeof(_prefs.telemetry_mode_env));
      file.read((uint8_t *)&_prefs.rx_delay_base, sizeof(_prefs.rx_delay_base));
      file.read((uint8_t *)&_prefs.ble_pin, sizeof(_prefs.ble_pin));
      file.read((uint8_t *)&_prefs.advert_loc_policy, sizeof(_prefs.advert_loc_policy));
      file.read((uint8_t *)&_prefs.buzzer_quiet, sizeof(_prefs.buzzer_quiet));
      file.read((uint8_t *)&_prefs.gps_enabled, sizeof(_prefs.gps_enabled));
      file.read((uint8_t *)&_prefs.gps_interval, sizeof(_prefs.gps_interval));
      file.read((uint8_t *)&_prefs.autoadd_config, sizeof(_prefs.autoadd_config));
      file.read((uint8_t *)&_prefs.rx_boosted_gain, sizeof(_prefs.rx_boosted_gain));
      file.read((uint8_t *)&_prefs.client_repeat, sizeof(_prefs.client_repeat));
      file.read((uint8_t *)&_prefs.path_hash_mode, sizeof(_prefs.path_hash_mode));
      file.read((uint8_t *)&_prefs.autoadd_max_hops, sizeof(_prefs.autoadd_max_hops));
      file.read((uint8_t *)_prefs.default_scope_name, sizeof(_prefs.default_scope_name));
      file.read((uint8_t *)_prefs.default_scope_key, sizeof(_prefs.default_scope_key));
      // next: board_password folded in from here (was owner_password) --
      // only present in a post-owner-password-field file (SETTINGS_ISOLATION
      // follow-up), same has_node_prefs-style hazard as the NodePrefs
      // fold-in above: an already-migrated file from before this field
      // existed ends right after default_scope_key, with no
      // board_password blob at all.
      if (file.available() >= (int)sizeof(_board.board_password)) {
        file.read((uint8_t *)_board.board_password, sizeof(_board.board_password));
        // next: board_name folded in from here -- same tail-hazard as
        // board_password above: a file saved before this field existed
        // ends right after board_password, with no board_name blob at all.
        if (file.available() >= (int)sizeof(_board.board_name)) {
          file.read((uint8_t *)_board.board_name, sizeof(_board.board_name));
          // next: node_lat/node_lon (BeeboBasePrefs's PREFS-mode
          // advert_loc_policy coordinate) folded in from here -- same
          // tail-hazard as board_password/board_name above: a file saved
          // before these fields existed ends right after board_name, with
          // no coords blob at all. Distinct from the node_lat/node_lon
          // out-params read earlier in this same function (those are the
          // SHARE-mode sensors.node_lat/lon register, an unrelated field).
          if (file.available() >= (int)(sizeof(_prefs.node_lat) + sizeof(_prefs.node_lon))) {
            file.read((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
            file.read((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
            // next: monring_event_mask folded in from here -- same
            // tail-hazard as node_lat/node_lon above: a file saved before
            // this field existed ends right after node_lon, with no mask
            // present.
            if (file.available() >= (int)sizeof(_prefs.monring_event_mask)) {
              file.read((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
            }
          }
        }
      }
    }

    file.close();
  }
  return has_node_prefs;
}

void DataStore::saveBeeboCompanionPrefs(const BeeboPrefs& _prefs, const BeeboBoardPrefs& _board, double node_lat, double node_lon) {
  File file = openWrite(_fs, "/beebo_companion");
  if (file) {
    file.write((uint8_t *)&_prefs.radio_fem_rxgain, sizeof(_prefs.radio_fem_rxgain));               // 0
    file.write((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));                              // 1
    file.write((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));                                // 65
    file.write((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));                         // 129
    file.write((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));                         // 130
    file.write((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));                         // 131
    file.write((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));                   // 132
    // beebo: BOARD_BATTERY_PREFS.md -- written from `_board` now, see
    // loadBeeboCompanionPrefs() above. Permanent inert echo into
    // /beebo_companion's legacy tail slot, same as _board.role/
    // board_password/board_name -- keeps this file's on-disk layout byte-
    // for-byte unchanged forever, so every field after this point stays
    // correctly aligned regardless of firmware vintage.
    file.write((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));                   // 133
    file.write((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs)); // 137
    file.write((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));                       // 139
    file.write((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));         // 140
    file.write((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs)); // 141
    file.write((uint8_t *)&_board.batt_charged_mv, sizeof(_board.batt_charged_mv));                 // 143
    file.write((uint8_t *)&_board.idle_margin_ms, sizeof(_board.idle_margin_ms));                   // 145
    file.write((uint8_t *)&_board.role, sizeof(_board.role));                                       // 147 -- was node_role
    file.write((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));                 // 148
    // next: 152 -- NodePrefs folded in from here, see loadBeeboCompanionPrefs()
    file.write((uint8_t *)&_prefs.airtime_factor, sizeof(_prefs.airtime_factor));
    file.write((uint8_t *)_prefs.node_name, sizeof(_prefs.node_name));
    file.write((uint8_t *)&node_lat, sizeof(node_lat));
    file.write((uint8_t *)&node_lon, sizeof(node_lon));
    file.write((uint8_t *)&_prefs.freq, sizeof(_prefs.freq));
    file.write((uint8_t *)&_prefs.sf, sizeof(_prefs.sf));
    file.write((uint8_t *)&_prefs.cr, sizeof(_prefs.cr));
    file.write((uint8_t *)&_prefs.multi_acks, sizeof(_prefs.multi_acks));
    file.write((uint8_t *)&_prefs.manual_add_contacts, sizeof(_prefs.manual_add_contacts));
    file.write((uint8_t *)&_prefs.bw, sizeof(_prefs.bw));
    file.write((uint8_t *)&_prefs.tx_power_dbm, sizeof(_prefs.tx_power_dbm));
    file.write((uint8_t *)&_prefs.telemetry_mode_base, sizeof(_prefs.telemetry_mode_base));
    file.write((uint8_t *)&_prefs.telemetry_mode_loc, sizeof(_prefs.telemetry_mode_loc));
    file.write((uint8_t *)&_prefs.telemetry_mode_env, sizeof(_prefs.telemetry_mode_env));
    file.write((uint8_t *)&_prefs.rx_delay_base, sizeof(_prefs.rx_delay_base));
    file.write((uint8_t *)&_prefs.ble_pin, sizeof(_prefs.ble_pin));
    file.write((uint8_t *)&_prefs.advert_loc_policy, sizeof(_prefs.advert_loc_policy));
    file.write((uint8_t *)&_prefs.buzzer_quiet, sizeof(_prefs.buzzer_quiet));
    file.write((uint8_t *)&_prefs.gps_enabled, sizeof(_prefs.gps_enabled));
    file.write((uint8_t *)&_prefs.gps_interval, sizeof(_prefs.gps_interval));
    file.write((uint8_t *)&_prefs.autoadd_config, sizeof(_prefs.autoadd_config));
    file.write((uint8_t *)&_prefs.rx_boosted_gain, sizeof(_prefs.rx_boosted_gain));
    file.write((uint8_t *)&_prefs.client_repeat, sizeof(_prefs.client_repeat));
    file.write((uint8_t *)&_prefs.path_hash_mode, sizeof(_prefs.path_hash_mode));
    file.write((uint8_t *)&_prefs.autoadd_max_hops, sizeof(_prefs.autoadd_max_hops));
    file.write((uint8_t *)_prefs.default_scope_name, sizeof(_prefs.default_scope_name));
    file.write((uint8_t *)_prefs.default_scope_key, sizeof(_prefs.default_scope_key));
    file.write((uint8_t *)_board.board_password, sizeof(_board.board_password));
    file.write((uint8_t *)_board.board_name, sizeof(_board.board_name));
    file.write((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
    file.write((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
    file.write((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));

    file.close();
  }
}

// beebo: BeeboPrefs unification Part 1 -- BeeboBoardPrefs's own file
// (role, board_password, board_name), genuinely untouched by the
// role-switch park/load handoff. Loaded first at boot, before either
// role's own file -- see Beebo::begin(). Plain existence used to be enough
// here (this file was new as of that refactor, no older/shorter format to
// distinguish from a fully-migrated one) -- as of BOARD_BATTERY_PREFS.md
// that's no longer true: a /beebo_board written before adc_multiplier/
// adc_resolution_bits/batt_present/batt_sample_period_secs/
// batt_sample_window_secs/batt_charged_mv/idle_margin_ms existed ends
// right after board_name, same tail-hazard shape as loadBeeboCompanionPrefs/
// loadBeeboRepeaterPrefs. Returns false (not just "file missing") in that
// case too, so Beebo::begin() knows to persist the just-migrated values
// (seeded from whichever of /beebo_companion's or /beebo_repeater's own
// legacy tail loaded them into `_prefs` during loadRoleState()) --
// otherwise they'd only ever exist in RAM until some unrelated board write
// happened to flush them.
bool DataStore::loadBeeboBoardPrefs(BeeboBoardPrefs& _prefs) {
  File file = openRead(_fs, "/beebo_board");
  if (!file) return false;
  file.read((uint8_t *)&_prefs.role, sizeof(_prefs.role));                     // 0
  file.read((uint8_t *)_prefs.board_password, sizeof(_prefs.board_password));  // 1
  file.read((uint8_t *)_prefs.board_name, sizeof(_prefs.board_name));          // 17
  // next: 49 -- adc_multiplier/adc_resolution_bits/batt_present/
  // batt_sample_period_secs/batt_sample_window_secs/batt_charged_mv/
  // idle_margin_ms folded in from here. Only present in a post-fold-in
  // file -- an older /beebo_board ends right here.
  size_t battery_fields_len = sizeof(_prefs.adc_multiplier) + sizeof(_prefs.adc_resolution_bits)
    + sizeof(_prefs.batt_present) + sizeof(_prefs.batt_sample_period_secs)
    + sizeof(_prefs.batt_sample_window_secs) + sizeof(_prefs.batt_charged_mv)
    + sizeof(_prefs.idle_margin_ms);
  bool has_battery_fields = (size_t)file.available() >= battery_fields_len;
  if (has_battery_fields) {
    file.read((uint8_t *)&_prefs.adc_multiplier, sizeof(_prefs.adc_multiplier));           // 49
    file.read((uint8_t *)&_prefs.adc_resolution_bits, sizeof(_prefs.adc_resolution_bits)); // 53
    file.read((uint8_t *)&_prefs.batt_present, sizeof(_prefs.batt_present));               // 54
    file.read((uint8_t *)&_prefs.batt_sample_period_secs, sizeof(_prefs.batt_sample_period_secs)); // 55
    file.read((uint8_t *)&_prefs.batt_sample_window_secs, sizeof(_prefs.batt_sample_window_secs)); // 57
    file.read((uint8_t *)&_prefs.batt_charged_mv, sizeof(_prefs.batt_charged_mv));         // 59
    file.read((uint8_t *)&_prefs.idle_margin_ms, sizeof(_prefs.idle_margin_ms));           // 61
    // next: 63
  }
  file.close();
  return has_battery_fields;
}

void DataStore::saveBeeboBoardPrefs(const BeeboBoardPrefs& _prefs) {
  File file = openWrite(_fs, "/beebo_board");
  if (file) {
    file.write((uint8_t *)&_prefs.role, sizeof(_prefs.role));                     // 0
    file.write((uint8_t *)_prefs.board_password, sizeof(_prefs.board_password));  // 1
    file.write((uint8_t *)_prefs.board_name, sizeof(_prefs.board_name));          // 17
    // next: 49 -- BOARD_BATTERY_PREFS.md, see loadBeeboBoardPrefs()
    file.write((uint8_t *)&_prefs.adc_multiplier, sizeof(_prefs.adc_multiplier));           // 49
    file.write((uint8_t *)&_prefs.adc_resolution_bits, sizeof(_prefs.adc_resolution_bits)); // 53
    file.write((uint8_t *)&_prefs.batt_present, sizeof(_prefs.batt_present));               // 54
    file.write((uint8_t *)&_prefs.batt_sample_period_secs, sizeof(_prefs.batt_sample_period_secs)); // 55
    file.write((uint8_t *)&_prefs.batt_sample_window_secs, sizeof(_prefs.batt_sample_window_secs)); // 57
    file.write((uint8_t *)&_prefs.batt_charged_mv, sizeof(_prefs.batt_charged_mv));         // 59
    file.write((uint8_t *)&_prefs.idle_margin_ms, sizeof(_prefs.idle_margin_ms));           // 61
    // next: 63
    file.close();
  }
}

// beebo: SETTINGS_ISOLATION -- same old-format-vs-new-format hazard as
// loadBeeboCompanionPrefs() above: /beebo_repeater predates the ComPrefs
// fold-in (BeeboRepeaterPrefs was already a shipped 4-byte dedup_window_ms
// file), so a real device's existing file ends right after that field,
// with no ComPrefs blob at all. Detect via file.available(), not plain
// existence, or the com_prefs blob silently never gets populated from the
// file (com_prefs keeps whatever the ctor/previous-boot RAM state was).
bool DataStore::loadBeeboRepeaterPrefs(BeeboPrefs& _prefs, BeeboBoardPrefs& _board, void* com_prefs, size_t com_prefs_len) {
  bool has_com_prefs = false;
  File file = openRead(_fs, "/beebo_repeater");
  if (file) {
    file.read((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));   // 0
    // next: 4 -- ComPrefs blob folded in from here. Only present in a
    // post-fold-in file -- an old-format /beebo_repeater ends right here.
    has_com_prefs = (size_t)file.available() >= com_prefs_len;
    if (has_com_prefs) {
      file.read((uint8_t *)com_prefs, com_prefs_len);                                // 4, raw ComPrefs blob
      // next: BeeboBasePrefs's remaining fields folded in from here --
      // repeater's own independent copy of every BeeboBasePrefs field
      // (SETTINGS_REFACTOR.md Part 1; before this refactor only
      // dedup_window_ms had a repeater-side value at all). Tail-guarded,
      // same has_com_prefs-style hazard as above: a file saved before
      // this fold-in ends right after the ComPrefs blob, with none of
      // these fields present.
      if (file.available() > 0) {
        file.read((uint8_t *)&_prefs.radio_fem_rxgain, sizeof(_prefs.radio_fem_rxgain));
        // beebo: BOARD_BATTERY_PREFS.md -- these seven now read into
        // `_board`, not `_prefs` -- same legacy-tail-seed/inert-echo role
        // as loadBeeboCompanionPrefs()'s own `_board` reads. Byte
        // offsets/order unchanged from before.
        file.read((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));
        file.read((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));
        file.read((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));
        file.read((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs));
        file.read((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs));
        file.read((uint8_t *)&_board.batt_charged_mv, sizeof(_board.batt_charged_mv));
        file.read((uint8_t *)&_board.idle_margin_ms, sizeof(_board.idle_margin_ms));
        file.read((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
        file.read((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
        file.read((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));
        file.read((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));
        file.read((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));
        file.read((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));
        file.read((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));
        file.read((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));
        // next: monring_event_mask folded in from here -- same tail-hazard
        // as the fields above: a file saved before this field existed ends
        // right after monring_config, with no mask present.
        if (file.available() >= (int)sizeof(_prefs.monring_event_mask)) {
          file.read((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
        }
      }
    }

    file.close();
  }
  return has_com_prefs;
}

void DataStore::saveBeeboRepeaterPrefs(const BeeboPrefs& _prefs, const BeeboBoardPrefs& _board, const void* com_prefs, size_t com_prefs_len) {
  File file = openWrite(_fs, "/beebo_repeater");
  if (file) {
    file.write((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));   // 0
    file.write((const uint8_t *)com_prefs, com_prefs_len);                            // 4, raw ComPrefs blob
    // next: BeeboBasePrefs's remaining fields, see loadBeeboRepeaterPrefs()
    file.write((uint8_t *)&_prefs.radio_fem_rxgain, sizeof(_prefs.radio_fem_rxgain));
    // beebo: BOARD_BATTERY_PREFS.md -- written from `_board` now, see
    // loadBeeboRepeaterPrefs() above. Permanent inert echo, same rationale
    // as saveBeeboCompanionPrefs()'s own `_board` writes.
    file.write((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));
    file.write((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));
    file.write((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));
    file.write((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs));
    file.write((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs));
    file.write((uint8_t *)&_board.batt_charged_mv, sizeof(_board.batt_charged_mv));
    file.write((uint8_t *)&_board.idle_margin_ms, sizeof(_board.idle_margin_ms));
    file.write((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
    file.write((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
    file.write((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));
    file.write((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));
    file.write((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));
    file.write((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));
    file.write((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));
    file.write((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));
    file.write((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
    // next: 4 + com_prefs_len + BeeboBasePrefs's remaining-field bytes

    file.close();
  }
}

void DataStore::loadContacts(DataStoreHost* host) {
File file = openRead(_getContactsChannelsFS(), "/contacts3");
    if (file) {
      bool full = false;
      while (!full) {
        ContactInfo c;
        uint8_t pub_key[32];
        uint8_t unused;

        bool success = (file.read(pub_key, 32) == 32);
        success = success && (file.read((uint8_t *)&c.name, 32) == 32);
        success = success && (file.read(&c.type, 1) == 1);
        success = success && (file.read(&c.flags, 1) == 1);
        success = success && (file.read(&unused, 1) == 1);
        success = success && (file.read((uint8_t *)&c.sync_since, 4) == 4); // was 'reserved'
        success = success && (file.read((uint8_t *)&c.out_path_len, 1) == 1);
        success = success && (file.read((uint8_t *)&c.last_advert_timestamp, 4) == 4);
        success = success && (file.read(c.out_path, 64) == 64);
        success = success && (file.read((uint8_t *)&c.lastmod, 4) == 4);
        success = success && (file.read((uint8_t *)&c.gps_lat, 4) == 4);
        success = success && (file.read((uint8_t *)&c.gps_lon, 4) == 4);

        if (!success) break; // EOF

        c.id = mesh::Identity(pub_key);
        if (!host->onContactLoaded(c)) full = true;
      }
      file.close();
    }
}

void DataStore::saveContacts(DataStoreHost* host, bool (*filter)(const ContactInfo& c)) {
  File file = openWrite(_getContactsChannelsFS(), "/contacts3");
  if (file) {
    uint32_t idx = 0;
    ContactInfo c;
    uint8_t unused = 0;

    while (host->getContactForSave(idx, c)) {
      if (filter && !filter(c)) {
        idx++;  // advance to next contact
        continue;
      }
      bool success = (file.write(c.id.pub_key, 32) == 32);
      success = success && (file.write((uint8_t *)&c.name, 32) == 32);
      success = success && (file.write(&c.type, 1) == 1);
      success = success && (file.write(&c.flags, 1) == 1);
      success = success && (file.write(&unused, 1) == 1);
      success = success && (file.write((uint8_t *)&c.sync_since, 4) == 4);
      success = success && (file.write((uint8_t *)&c.out_path_len, 1) == 1);
      success = success && (file.write((uint8_t *)&c.last_advert_timestamp, 4) == 4);
      success = success && (file.write(c.out_path, 64) == 64);
      success = success && (file.write((uint8_t *)&c.lastmod, 4) == 4);
      success = success && (file.write((uint8_t *)&c.gps_lat, 4) == 4);
      success = success && (file.write((uint8_t *)&c.gps_lon, 4) == 4);

      if (!success) break; // write failed

      idx++;  // advance to next contact
    }
    file.close();
  }
}

void DataStore::loadChannels(DataStoreHost* host) {
    File file = openRead(_getContactsChannelsFS(), "/channels2");
    if (file) {
      bool full = false;
      uint8_t channel_idx = 0;
      while (!full) {
        ChannelDetails ch;
        uint8_t unused[4];

        bool success = (file.read(unused, 4) == 4);
        success = success && (file.read((uint8_t *)ch.name, 32) == 32);
        success = success && (file.read((uint8_t *)ch.channel.secret, 32) == 32);

        if (!success) break; // EOF

        if (host->onChannelLoaded(channel_idx, ch)) {
          channel_idx++;
        } else {
          full = true;
        }
      }
      file.close();
    }
}

void DataStore::saveChannels(DataStoreHost* host) {
  File file = openWrite(_getContactsChannelsFS(), "/channels2");
  if (file) {
    uint8_t channel_idx = 0;
    ChannelDetails ch;
    uint8_t unused[4];
    memset(unused, 0, 4);

    while (host->getChannelForSave(channel_idx, ch)) {
      bool success = (file.write(unused, 4) == 4);
      success = success && (file.write((uint8_t *)ch.name, 32) == 32);
      success = success && (file.write((uint8_t *)ch.channel.secret, 32) == 32);

      if (!success) break; // write failed
      channel_idx++;
    }
    file.close();
  }
}

inline void makeBlobPath(const uint8_t key[], int key_len, char* path, size_t path_size) {
  char fname[18];
  if (key_len > 8) key_len = 8; // just use first 8 bytes (prefix)
  mesh::Utils::toHex(fname, key, key_len);
  sprintf(path, "/bl/%s", fname);
}

uint8_t DataStore::getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) {
  char path[64];
  makeBlobPath(key, key_len, path, sizeof(path));

  if (_fs->exists(path)) {
    File f = openRead(_fs, path);
    if (f) {
      int len = f.read(dest_buf, 255); // currently MAX 255 byte blob len supported!!
      f.close();
      return len;
    }
  }
  return 0; // not found
}

bool DataStore::putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len) {
  char path[64];
  makeBlobPath(key, key_len, path, sizeof(path));

  File f = openWrite(_fs, path);
  if (f) {
    int n = f.write(src_buf, len);
    f.close();
    if (n == len) return true; // success!

    _fs->remove(path); // blob was only partially written!
  }
  return false; // error
}

bool DataStore::deleteBlobByKey(const uint8_t key[], int key_len) {
  char path[64];
  makeBlobPath(key, key_len, path, sizeof(path));

  _fs->remove(path);
  
  return true; // return true even if file did not exist
}
