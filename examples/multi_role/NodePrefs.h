#pragma once
#include <cstdint>

#define TELEM_MODE_DENY            0
#define TELEM_MODE_ALLOW_FLAGS     1     // use contact.flags
#define TELEM_MODE_ALLOW_ALL       2

#define ADVERT_LOC_NONE       0
#define ADVERT_LOC_SHARE      1

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1). Supersedes
// the stock ../companion_radio/NodePrefs.h include -- this file is the sole
// source of the local `NodePrefs` type from here on.
//
// SharedPrefs documents the 14 fields real (stock companion_radio)
// NodePrefs and ComPrefs (CommonCLI.h's own NodePrefs, aliased to ComPrefs
// in Beebo.h) share by name+type. It's never inherited or instantiated --
// it exists purely so that 14-field list has one canonical, greppable
// declaration site.
struct SharedPrefs {
  float airtime_factor;
  char node_name[32];
  float freq;
  uint8_t sf;
  uint8_t cr;
  uint8_t multi_acks;
  float bw;
  int8_t tx_power_dbm;
  float rx_delay_base;
  uint8_t advert_loc_policy;
  uint8_t gps_enabled;
  uint32_t gps_interval;
  uint8_t rx_boosted_gain;
  uint8_t path_hash_mode;
};

#if BEEBO_ENABLE_REPEATER_ROLE
// Repeater role is compiled in, so ComPrefs (CommonCLI.h's NodePrefs,
// inherited by BeeboRepeaterPrefs) exists and singly owns the 14
// SharedPrefs-named fields. Distill local NodePrefs down to real stock
// NodePrefs's 11 fields NOT in that overlap, so BeeboPrefs can inherit both
// this and BeeboRepeaterPrefs without an ambiguous-member conflict.
struct NodePrefs {
  uint8_t manual_add_contacts;
  uint8_t telemetry_mode_base;
  uint8_t telemetry_mode_loc;
  uint8_t telemetry_mode_env;
  uint32_t ble_pin;
  uint8_t buzzer_quiet;
  uint8_t autoadd_config;
  uint8_t client_repeat;
  uint8_t autoadd_max_hops;
  char default_scope_name[31];
  uint8_t default_scope_key[16];
};
#else
// Repeater role compiled out entirely (heltec_v4_3_companion static build)
// -- there is no ComPrefs/BeeboRepeaterPrefs base in BeeboPrefs to own the
// 14 SharedPrefs fields, so NodePrefs here must be the full, real stock
// companion_radio/NodePrefs.h shape (byte-for-byte identical field set) so
// generic code can still read/write freq/sf/cr/etc. by plain name. Keeps
// BeeboPrefs field-name access uniform across build configurations --
// callers never need to know which of these two shapes is active.
struct NodePrefs {
  float airtime_factor;
  char node_name[32];
  float freq;
  uint8_t sf;
  uint8_t cr;
  uint8_t multi_acks;
  uint8_t manual_add_contacts;
  float bw;
  int8_t tx_power_dbm;
  uint8_t telemetry_mode_base;
  uint8_t telemetry_mode_loc;
  uint8_t telemetry_mode_env;
  float rx_delay_base;
  uint32_t ble_pin;
  uint8_t  advert_loc_policy;
  uint8_t  buzzer_quiet;
  uint8_t  gps_enabled;
  uint32_t gps_interval;
  uint8_t autoadd_config;
  uint8_t rx_boosted_gain;
  uint8_t client_repeat;
  uint8_t path_hash_mode;
  uint8_t autoadd_max_hops;
  char default_scope_name[31];
  uint8_t default_scope_key[16];
};
#endif
