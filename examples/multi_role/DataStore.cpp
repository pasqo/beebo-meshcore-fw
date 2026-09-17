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

  // Load /beebo_abi first, before any other persisted file -- an absent
  // file leaves every _abi field at its all-zero default, which every
  // load*Prefs() below treats as "predates this mechanism".
  File file = openRead(_fs, "/beebo_abi");
  if (file) {
    file.read((uint8_t *)&_abi.board_prefs_version, sizeof(_abi.board_prefs_version));
    file.read((uint8_t *)&_abi.companion_prefs_version, sizeof(_abi.companion_prefs_version));
    file.read((uint8_t *)&_abi.repeater_prefs_version, sizeof(_abi.repeater_prefs_version));
    file.read((uint8_t *)&_abi.prefs_tlv_abi_version, sizeof(_abi.prefs_tlv_abi_version));
    file.read((uint8_t *)_abi.tool_version, sizeof(_abi.tool_version));
    file.read((uint8_t *)&_abi.saved_at, sizeof(_abi.saved_at));
    file.close();
  }
}

void DataStore::saveAbi() {
  strncpy(_abi.tool_version, FIRMWARE_VERSION, sizeof(_abi.tool_version) - 1);
  _abi.tool_version[sizeof(_abi.tool_version) - 1] = '\0';
  _abi.saved_at = (uint32_t)_clock->getCurrentTime();
  File file = openWrite(_fs, "/beebo_abi");
  if (file) {
    file.write((uint8_t *)&_abi.board_prefs_version, sizeof(_abi.board_prefs_version));
    file.write((uint8_t *)&_abi.companion_prefs_version, sizeof(_abi.companion_prefs_version));
    file.write((uint8_t *)&_abi.repeater_prefs_version, sizeof(_abi.repeater_prefs_version));
    file.write((uint8_t *)&_abi.prefs_tlv_abi_version, sizeof(_abi.prefs_tlv_abi_version));
    file.write((uint8_t *)_abi.tool_version, sizeof(_abi.tool_version));
    file.write((uint8_t *)&_abi.saved_at, sizeof(_abi.saved_at));
    file.close();
  }
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

// Read-only seed from stock companion_radio's files. Never writes
// /new_prefs or /node_prefs, never deletes either -- a one-time read into
// beebo's own /beebo_companion, so stock's files survive a future reflash
// back to stock untouched.
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

// /beebo_companion carries NodePrefs (companion's radio/node settings)
// folded in after BeeboCompanionPrefs's own fields, so companion settings
// never touch /new_prefs after the one-time seed (see Beebo::begin()).
// Returns true if the file already held a fully migrated copy
// (BeeboCompanionPrefs's own fields AND the NodePrefs tail) -- false means
// the caller should seed NodePrefs via loadLegacyNodePrefs().
//
// Deliberately NOT gated on plain file existence: /beebo_companion
// predates this fold-in (BeeboCompanionPrefs was already a shipped,
// deployed file format before NodePrefs was appended to it), so a real
// device's existing file is old-format -- present, but ending right after
// dedup_window_ms, with no NodePrefs tail at all. Treating "exists" as
// "fully migrated" would silently read past EOF into the node.* fields
// (Arduino's File::read() is a no-op past EOF, leaving them at whatever
// the RAM defaults already were, silently wrong rather than a load error).
// file.available() after the known old-format fields is what actually
// distinguishes old-format from new-format, with no version byte needed.
bool DataStore::loadBeeboCompanionPrefs(BeeboPrefs& _prefs, BeeboBoardPrefs& _board, double& node_lat, double& node_lon) {
  bool has_node_prefs = false;
  File file = openRead(_fs, "/beebo_companion");
  if (file) {
    file.read((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_rxgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_rxgain));               // 0
    file.read((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));                              // 1
    file.read((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));                                // 65
    file.read((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));                         // 129
    file.read((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));                         // 130
    file.read((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));                         // 131
    file.read((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));                   // 132
    if (_abi.companion_prefs_version < COMPANION_PREFS_VERSION) {
      // Legacy layout only -- role/adc_multiplier/adc_resolution_bits/
      // batt_present/batt_sample_period_secs/batt_sample_window_secs used
      // to be echoed here from `_board`, purely as a one-time migration
      // seed for a pre-/beebo_board device. /beebo_board is the sole
      // authoritative store for these once companion_prefs_version reaches
      // COMPANION_PREFS_VERSION -- see BeeboAbi.h.
      file.read((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));
      file.read((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs));
      file.read((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));
      file.read((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));
      file.read((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs));
      uint16_t discard;  // was batt_charged_mv, idle_margin_ms -- BeeboAbi.h
      file.read((uint8_t *)&discard, sizeof(discard));
      file.read((uint8_t *)&discard, sizeof(discard));
      file.read((uint8_t *)&_board.role, sizeof(_board.role));  // was node_role
    }
    file.read((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));
    // NodePrefs (now BeeboPrefs's 14 SharedPrefs + 11 distilled
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
      if (_abi.companion_prefs_version < COMPANION_PREFS_VERSION) {
        // Legacy layout only -- board_password/board_name were echoed
        // here too; same one-time-seed rationale as the block above.  An
        // already-migrated file from before these fields existed ends
        // right after default_scope_key/board_password respectively, with
        // no blob at all -- hence the nested availability checks.
        if (file.available() >= (int)sizeof(_board.board_password)) {
          file.read((uint8_t *)_board.board_password, sizeof(_board.board_password));
          if (file.available() >= (int)sizeof(_board.board_name)) {
            file.read((uint8_t *)_board.board_name, sizeof(_board.board_name));
          }
        }
      }
      // node_lat/node_lon (BeeboBasePrefs's PREFS-mode advert_loc_policy
      // coordinate) -- distinct from the node_lat/node_lon out-params read
      // earlier in this function (those are the SHARE-mode
      // sensors.node_lat/lon register, an unrelated field). A file below
      // COMPANION_PREFS_VERSION may still predate these too.
      if (file.available() >= (int)(sizeof(_prefs.node_lat) + sizeof(_prefs.node_lon))) {
        file.read((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
        file.read((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
        if (file.available() >= (int)sizeof(_prefs.monring_event_mask)) {
          file.read((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
          // radio_fem_txgain -- tail-guarded the same way monring_event_mask
          // above is, defaulting to 0 (off) for a file saved before this
          // field existed.
          if (file.available() >= (int)sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain)) {
            file.read((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_txgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain));
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
    file.write((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_rxgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_rxgain));               // 0
    file.write((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));                              // 1
    file.write((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));                                // 65
    file.write((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));                         // 129
    file.write((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));                         // 130
    file.write((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));                         // 131
    file.write((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));                   // 132
    // role/board_password/board_name/battery-ADC fields are no longer
    // echoed here -- /beebo_board is their sole store as of
    // COMPANION_PREFS_VERSION (BeeboAbi.h).
    file.write((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));
    // NodePrefs folded in from here, see loadBeeboCompanionPrefs()
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
    file.write((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
    file.write((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
    file.write((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
    file.write((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_txgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain));

    file.close();
  }
  _abi.companion_prefs_version = COMPANION_PREFS_VERSION;
  saveAbi();
}

// A /beebo_board written before the battery/ADC fields existed ends right
// after board_name -- has_battery_fields below distinguishes that from a
// fully-migrated file. Returns false (not just "file missing") in that
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
  // batt_sample_period_secs/batt_sample_window_secs folded in from here.
  // Only present in a post-fold-in file -- an older /beebo_board ends
  // right here. A file below BOARD_PREFS_VERSION additionally carries two
  // now-dropped fields right after batt_sample_window_secs (BeeboAbi.h).
  size_t battery_fields_len = sizeof(_prefs.adc_multiplier) + sizeof(_prefs.adc_resolution_bits)
    + sizeof(_prefs.batt_present) + sizeof(_prefs.batt_sample_period_secs)
    + sizeof(_prefs.batt_sample_window_secs);
  if (_abi.board_prefs_version < BOARD_PREFS_VERSION) battery_fields_len += 2 * sizeof(uint16_t);
  bool has_battery_fields = (size_t)file.available() >= battery_fields_len;
  if (has_battery_fields) {
    file.read((uint8_t *)&_prefs.adc_multiplier, sizeof(_prefs.adc_multiplier));           // 49
    file.read((uint8_t *)&_prefs.adc_resolution_bits, sizeof(_prefs.adc_resolution_bits)); // 53
    file.read((uint8_t *)&_prefs.batt_present, sizeof(_prefs.batt_present));               // 54
    file.read((uint8_t *)&_prefs.batt_sample_period_secs, sizeof(_prefs.batt_sample_period_secs)); // 55
    file.read((uint8_t *)&_prefs.batt_sample_window_secs, sizeof(_prefs.batt_sample_window_secs)); // 57
    if (_abi.board_prefs_version < BOARD_PREFS_VERSION) {
      uint16_t discard;
      file.read((uint8_t *)&discard, sizeof(discard));
      file.read((uint8_t *)&discard, sizeof(discard));
    }
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
    // next: 49 -- see loadBeeboBoardPrefs()
    file.write((uint8_t *)&_prefs.adc_multiplier, sizeof(_prefs.adc_multiplier));           // 49
    file.write((uint8_t *)&_prefs.adc_resolution_bits, sizeof(_prefs.adc_resolution_bits)); // 53
    file.write((uint8_t *)&_prefs.batt_present, sizeof(_prefs.batt_present));               // 54
    file.write((uint8_t *)&_prefs.batt_sample_period_secs, sizeof(_prefs.batt_sample_period_secs)); // 55
    file.write((uint8_t *)&_prefs.batt_sample_window_secs, sizeof(_prefs.batt_sample_window_secs)); // 57
    file.close();
  }
  _abi.board_prefs_version = BOARD_PREFS_VERSION;
  saveAbi();
}

// Same old-format-vs-new-format hazard as loadBeeboCompanionPrefs() above:
// a real device's existing /beebo_repeater can end right after
// dedup_window_ms, with no ComPrefs blob at all. Detect via
// file.available(), not plain existence, or the com_prefs blob silently
// never gets populated from the file (com_prefs keeps whatever the
// ctor/previous-boot RAM state was).
// beebo: ComPrefs (CommonCLI.h's upstream NodePrefs) stopped being a plain
// POD struct as of companion-v1.17.1's ConfigSerializer refactor -- it now
// carries a vtable and several ConfigSerializer-derived sub-object members
// (radio/bridge/gps/power/repeat/room), each with its own vtable and a
// pointer back to the parent object. A raw sizeof(ComPrefs) byte blob (the
// pre-1.17.1 approach this used to use) blits that live, pointer-laden
// object's memory straight from/to disk -- corrupting the vtable and every
// field at the first mismatch between on-disk and in-memory layout. Fields
// below are read/written individually instead, in ComPrefs's own declared
// order, skipping its ConfigSerializer sub-objects entirely (they're
// text-CLI serialization glue, not backing data -- see CommonCLI.h).
static size_t comPrefsFieldsLen(const ComPrefs& c) {
  return sizeof(c.airtime_factor) + sizeof(c.node_name) + sizeof(c.node_lat) + sizeof(c.node_lon)
    + sizeof(c.password) + sizeof(c.freq) + sizeof(c.tx_power_dbm) + sizeof(c.disable_fwd)
    + sizeof(c.advert_interval) + sizeof(c.flood_advert_interval) + sizeof(c.rx_delay_base)
    + sizeof(c.tx_delay_factor) + sizeof(c.guest_password) + sizeof(c.direct_tx_delay_factor)
    + sizeof(c.guard) + sizeof(c.sf) + sizeof(c.cr) + sizeof(c.allow_read_only) + sizeof(c.multi_acks)
    + sizeof(c.bw) + sizeof(c.flood_max) + sizeof(c.flood_max_unscoped) + sizeof(c.flood_max_advert)
    + sizeof(c.interference_threshold) + sizeof(c.agc_reset_interval) + sizeof(c.bridge_enabled)
    + sizeof(c.bridge_delay) + sizeof(c.bridge_pkt_src) + sizeof(c.bridge_baud) + sizeof(c.bridge_channel)
    + sizeof(c.bridge_secret) + sizeof(c.powersaving_enabled) + sizeof(c.gps_enabled) + sizeof(c.gps_interval)
    + sizeof(c.advert_loc_policy) + sizeof(c.discovery_mod_timestamp) + sizeof(c.adc_multiplier)
    + sizeof(c.owner_info) + sizeof(c.rx_boosted_gain) + sizeof(c.radio_fem_rxgain) + sizeof(c.radio_fem_txgain)
    + sizeof(c.path_hash_mode) + sizeof(c.loop_detect) + sizeof(c.cad_enabled) + sizeof(c.extra_sf);
}

bool DataStore::loadBeeboRepeaterPrefs(BeeboPrefs& _prefs, BeeboBoardPrefs& _board) {
  bool has_com_prefs = false;
  ComPrefs& c = _prefs;  // BeeboRepeaterPrefs's ComPrefs base -- see BeeboRepeaterPrefs.h
  File file = openRead(_fs, "/beebo_repeater");
  if (file) {
    file.read((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));   // 0
    // next: 4 -- ComPrefs fields folded in from here, one at a time (see
    // comment above). Only present in a post-fold-in file -- an
    // old-format /beebo_repeater ends right here.
    // beebo: also require repeater_prefs_version >= 3 -- a device that
    // last saved under the broken raw-blob build (BeeboAbi.h) has a file
    // whose ComPrefs region is a different, larger byte count, which would
    // otherwise still satisfy the available() check below and get
    // misread against the new field boundaries. Treated as "no com_prefs
    // yet" instead, so Beebo::loadRoleState() falls through to a clean
    // reseed from /com_prefs or /prefs.json.
    has_com_prefs = _abi.repeater_prefs_version >= REPEATER_PREFS_VERSION
      && (size_t)file.available() >= comPrefsFieldsLen(c);
    if (has_com_prefs) {
      file.read((uint8_t *)&c.airtime_factor, sizeof(c.airtime_factor));             // 4, ComPrefs fields
      file.read((uint8_t *)c.node_name, sizeof(c.node_name));
      file.read((uint8_t *)&c.node_lat, sizeof(c.node_lat));
      file.read((uint8_t *)&c.node_lon, sizeof(c.node_lon));
      file.read((uint8_t *)c.password, sizeof(c.password));
      file.read((uint8_t *)&c.freq, sizeof(c.freq));
      file.read((uint8_t *)&c.tx_power_dbm, sizeof(c.tx_power_dbm));
      file.read((uint8_t *)&c.disable_fwd, sizeof(c.disable_fwd));
      file.read((uint8_t *)&c.advert_interval, sizeof(c.advert_interval));
      file.read((uint8_t *)&c.flood_advert_interval, sizeof(c.flood_advert_interval));
      file.read((uint8_t *)&c.rx_delay_base, sizeof(c.rx_delay_base));
      file.read((uint8_t *)&c.tx_delay_factor, sizeof(c.tx_delay_factor));
      file.read((uint8_t *)c.guest_password, sizeof(c.guest_password));
      file.read((uint8_t *)&c.direct_tx_delay_factor, sizeof(c.direct_tx_delay_factor));
      file.read((uint8_t *)&c.guard, sizeof(c.guard));
      file.read((uint8_t *)&c.sf, sizeof(c.sf));
      file.read((uint8_t *)&c.cr, sizeof(c.cr));
      file.read((uint8_t *)&c.allow_read_only, sizeof(c.allow_read_only));
      file.read((uint8_t *)&c.multi_acks, sizeof(c.multi_acks));
      file.read((uint8_t *)&c.bw, sizeof(c.bw));
      file.read((uint8_t *)&c.flood_max, sizeof(c.flood_max));
      file.read((uint8_t *)&c.flood_max_unscoped, sizeof(c.flood_max_unscoped));
      file.read((uint8_t *)&c.flood_max_advert, sizeof(c.flood_max_advert));
      file.read((uint8_t *)&c.interference_threshold, sizeof(c.interference_threshold));
      file.read((uint8_t *)&c.agc_reset_interval, sizeof(c.agc_reset_interval));
      file.read((uint8_t *)&c.bridge_enabled, sizeof(c.bridge_enabled));
      file.read((uint8_t *)&c.bridge_delay, sizeof(c.bridge_delay));
      file.read((uint8_t *)&c.bridge_pkt_src, sizeof(c.bridge_pkt_src));
      file.read((uint8_t *)&c.bridge_baud, sizeof(c.bridge_baud));
      file.read((uint8_t *)&c.bridge_channel, sizeof(c.bridge_channel));
      file.read((uint8_t *)c.bridge_secret, sizeof(c.bridge_secret));
      file.read((uint8_t *)&c.powersaving_enabled, sizeof(c.powersaving_enabled));
      file.read((uint8_t *)&c.gps_enabled, sizeof(c.gps_enabled));
      file.read((uint8_t *)&c.gps_interval, sizeof(c.gps_interval));
      file.read((uint8_t *)&c.advert_loc_policy, sizeof(c.advert_loc_policy));
      file.read((uint8_t *)&c.discovery_mod_timestamp, sizeof(c.discovery_mod_timestamp));
      file.read((uint8_t *)&c.adc_multiplier, sizeof(c.adc_multiplier));
      file.read((uint8_t *)c.owner_info, sizeof(c.owner_info));
      file.read((uint8_t *)&c.rx_boosted_gain, sizeof(c.rx_boosted_gain));
      file.read((uint8_t *)&c.radio_fem_rxgain, sizeof(c.radio_fem_rxgain));
      file.read((uint8_t *)&c.radio_fem_txgain, sizeof(c.radio_fem_txgain));
      file.read((uint8_t *)&c.path_hash_mode, sizeof(c.path_hash_mode));
      file.read((uint8_t *)&c.loop_detect, sizeof(c.loop_detect));
      file.read((uint8_t *)&c.cad_enabled, sizeof(c.cad_enabled));
      file.read((uint8_t *)c.extra_sf, sizeof(c.extra_sf));
      // next: BeeboBasePrefs's remaining fields folded in from here --
      // repeater's own independent copy of every BeeboBasePrefs field.
      // Tail-guarded, same has_com_prefs-style hazard as above: a file
      // saved before this fold-in ends right after the ComPrefs blob,
      // with none of these fields present.
      if (file.available() > 0) {
        // Explicitly BeeboBasePrefs's own field, not ComPrefs's identically-
        // named one (upstream's own legacy-migration-only field, needed
        // there for loadPrefsInt()'s byte-offset compat) -- BeeboPrefs
        // multiply-inherits both, so an unqualified access is ambiguous.
        file.read((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_rxgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_rxgain));
        if (_abi.repeater_prefs_version < REPEATER_PREFS_VERSION) {
          // Legacy layout only -- adc_multiplier/adc_resolution_bits/
          // batt_present/batt_sample_period_secs/batt_sample_window_secs
          // are read here as a one-time migration seed for a
          // pre-/beebo_board device. /beebo_board is
          // the sole authoritative store for these once
          // repeater_prefs_version reaches REPEATER_PREFS_VERSION.
          file.read((uint8_t *)&_board.adc_multiplier, sizeof(_board.adc_multiplier));
          file.read((uint8_t *)&_board.adc_resolution_bits, sizeof(_board.adc_resolution_bits));
          file.read((uint8_t *)&_board.batt_present, sizeof(_board.batt_present));
          file.read((uint8_t *)&_board.batt_sample_period_secs, sizeof(_board.batt_sample_period_secs));
          file.read((uint8_t *)&_board.batt_sample_window_secs, sizeof(_board.batt_sample_window_secs));
          uint16_t discard;  // was batt_charged_mv, idle_margin_ms -- BeeboAbi.h
          file.read((uint8_t *)&discard, sizeof(discard));
          file.read((uint8_t *)&discard, sizeof(discard));
        }
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
          // ble_pin -- unlike companion's own NodePrefs fold-in
          // (loadBeeboCompanionPrefs()), without this field
          // repeater.node.ble.pin writes fine to RAM/flushes via the
          // dirty-bit path but silently reverts to 0 ("unset") on every
          // reboot. Appended here rather than inserted alongside companion's
          // matching field so an existing file saved before this fix isn't
          // misread -- tail-guarded the same way monring_event_mask above
          // is, defaulting to 0 (unset) rather than misreading a foreign
          // byte range past a shorter old-format file's actual end.
          if (file.available() >= (int)sizeof(_prefs.ble_pin)) {
            file.read((uint8_t *)&_prefs.ble_pin, sizeof(_prefs.ble_pin));
            // radio_fem_txgain -- tail-guarded the same way ble_pin above
            // is, defaulting to 0 (off) for a file saved before this field
            // existed.
            if (file.available() >= (int)sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain)) {
              file.read((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_txgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain));
            }
          }
        }
      }
    }

    file.close();
  }
  return has_com_prefs;
}

void DataStore::saveBeeboRepeaterPrefs(const BeeboPrefs& _prefs, const BeeboBoardPrefs& _board) {
  const ComPrefs& c = _prefs;  // BeeboRepeaterPrefs's ComPrefs base -- see BeeboRepeaterPrefs.h
  File file = openWrite(_fs, "/beebo_repeater");
  if (file) {
    file.write((uint8_t *)&_prefs.dedup_window_ms, sizeof(_prefs.dedup_window_ms));   // 0
    // ComPrefs fields, one at a time -- see loadBeeboRepeaterPrefs()'s
    // comment on why this can no longer be a raw struct blob.
    file.write((uint8_t *)&c.airtime_factor, sizeof(c.airtime_factor));               // 4, ComPrefs fields
    file.write((uint8_t *)c.node_name, sizeof(c.node_name));
    file.write((uint8_t *)&c.node_lat, sizeof(c.node_lat));
    file.write((uint8_t *)&c.node_lon, sizeof(c.node_lon));
    file.write((uint8_t *)c.password, sizeof(c.password));
    file.write((uint8_t *)&c.freq, sizeof(c.freq));
    file.write((uint8_t *)&c.tx_power_dbm, sizeof(c.tx_power_dbm));
    file.write((uint8_t *)&c.disable_fwd, sizeof(c.disable_fwd));
    file.write((uint8_t *)&c.advert_interval, sizeof(c.advert_interval));
    file.write((uint8_t *)&c.flood_advert_interval, sizeof(c.flood_advert_interval));
    file.write((uint8_t *)&c.rx_delay_base, sizeof(c.rx_delay_base));
    file.write((uint8_t *)&c.tx_delay_factor, sizeof(c.tx_delay_factor));
    file.write((uint8_t *)c.guest_password, sizeof(c.guest_password));
    file.write((uint8_t *)&c.direct_tx_delay_factor, sizeof(c.direct_tx_delay_factor));
    file.write((uint8_t *)&c.guard, sizeof(c.guard));
    file.write((uint8_t *)&c.sf, sizeof(c.sf));
    file.write((uint8_t *)&c.cr, sizeof(c.cr));
    file.write((uint8_t *)&c.allow_read_only, sizeof(c.allow_read_only));
    file.write((uint8_t *)&c.multi_acks, sizeof(c.multi_acks));
    file.write((uint8_t *)&c.bw, sizeof(c.bw));
    file.write((uint8_t *)&c.flood_max, sizeof(c.flood_max));
    file.write((uint8_t *)&c.flood_max_unscoped, sizeof(c.flood_max_unscoped));
    file.write((uint8_t *)&c.flood_max_advert, sizeof(c.flood_max_advert));
    file.write((uint8_t *)&c.interference_threshold, sizeof(c.interference_threshold));
    file.write((uint8_t *)&c.agc_reset_interval, sizeof(c.agc_reset_interval));
    file.write((uint8_t *)&c.bridge_enabled, sizeof(c.bridge_enabled));
    file.write((uint8_t *)&c.bridge_delay, sizeof(c.bridge_delay));
    file.write((uint8_t *)&c.bridge_pkt_src, sizeof(c.bridge_pkt_src));
    file.write((uint8_t *)&c.bridge_baud, sizeof(c.bridge_baud));
    file.write((uint8_t *)&c.bridge_channel, sizeof(c.bridge_channel));
    file.write((uint8_t *)c.bridge_secret, sizeof(c.bridge_secret));
    file.write((uint8_t *)&c.powersaving_enabled, sizeof(c.powersaving_enabled));
    file.write((uint8_t *)&c.gps_enabled, sizeof(c.gps_enabled));
    file.write((uint8_t *)&c.gps_interval, sizeof(c.gps_interval));
    file.write((uint8_t *)&c.advert_loc_policy, sizeof(c.advert_loc_policy));
    file.write((uint8_t *)&c.discovery_mod_timestamp, sizeof(c.discovery_mod_timestamp));
    file.write((uint8_t *)&c.adc_multiplier, sizeof(c.adc_multiplier));
    file.write((uint8_t *)c.owner_info, sizeof(c.owner_info));
    file.write((uint8_t *)&c.rx_boosted_gain, sizeof(c.rx_boosted_gain));
    file.write((uint8_t *)&c.radio_fem_rxgain, sizeof(c.radio_fem_rxgain));
    file.write((uint8_t *)&c.radio_fem_txgain, sizeof(c.radio_fem_txgain));
    file.write((uint8_t *)&c.path_hash_mode, sizeof(c.path_hash_mode));
    file.write((uint8_t *)&c.loop_detect, sizeof(c.loop_detect));
    file.write((uint8_t *)&c.cad_enabled, sizeof(c.cad_enabled));
    file.write((uint8_t *)c.extra_sf, sizeof(c.extra_sf));
    // next: BeeboBasePrefs's remaining fields, see loadBeeboRepeaterPrefs()
    // Explicitly BeeboBasePrefs's own field -- see loadBeeboRepeaterPrefs()'s
    // matching comment on why this must be qualified.
    file.write((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_rxgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_rxgain));
    // battery-ADC fields are no longer echoed here -- /beebo_board is
    // their sole store as of REPEATER_PREFS_VERSION (BeeboAbi.h).
    file.write((uint8_t *)&_prefs.node_lat, sizeof(_prefs.node_lat));
    file.write((uint8_t *)&_prefs.node_lon, sizeof(_prefs.node_lon));
    file.write((uint8_t *)_prefs.wifi_ssid, sizeof(_prefs.wifi_ssid));
    file.write((uint8_t *)_prefs.wifi_pwd, sizeof(_prefs.wifi_pwd));
    file.write((uint8_t *)&_prefs.ble_enabled, sizeof(_prefs.ble_enabled));
    file.write((uint8_t *)&_prefs.tcp_enabled, sizeof(_prefs.tcp_enabled));
    file.write((uint8_t *)&_prefs.usb_enabled, sizeof(_prefs.usb_enabled));
    file.write((uint8_t *)&_prefs.monring_config, sizeof(_prefs.monring_config));
    file.write((uint8_t *)&_prefs.monring_event_mask, sizeof(_prefs.monring_event_mask));
    // beebo: ble_pin -- see loadBeeboRepeaterPrefs()'s matching comment.
    file.write((uint8_t *)&_prefs.ble_pin, sizeof(_prefs.ble_pin));
    file.write((uint8_t *)&_prefs.BeeboBasePrefs::radio_fem_txgain, sizeof(_prefs.BeeboBasePrefs::radio_fem_txgain));

    file.close();
  }
  _abi.repeater_prefs_version = REPEATER_PREFS_VERSION;
  saveAbi();
}

#if BEEBO_ENABLE_COMPANION_ROLE
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
#endif // BEEBO_ENABLE_COMPANION_ROLE

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
