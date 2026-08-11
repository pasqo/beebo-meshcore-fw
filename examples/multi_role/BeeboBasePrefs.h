#pragma once
#include <cstdint>
#include <helpers/SimpleMeshTables.h>

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1). Beebo-only
// additions with no counterpart in either stock NodePrefs shape (see
// NodePrefs.h) -- singly-declared here so BeeboPrefs never has two
// same-named bases to disambiguate, but each field still gets an
// **independent per-role persisted value** (see DataStore.cpp's
// saveBeeboCompanionPrefs/saveBeeboRepeaterPrefs) -- "agnostic" describes
// where the field is *declared*, not what value it holds. Same
// per-role-value pattern as the 14 SharedPrefs fields in NodePrefs.h.
//
// adc_multiplier and node_lat/node_lon are genuinely native ComPrefs
// fields upstream, but companion needs them too and, in a
// heltec_v4_3_companion static build (BEEBO_ENABLE_REPEATER_ROLE=0),
// BeeboPrefs has no ComPrefs base to source them from at all -- see
// NodePrefs.h's build-conditional split. Declaring them here instead of
// relying on ComPrefs inheritance keeps BeeboPrefs field access uniform
// (`_beebo_prefs.node_lat` always resolves the same way) regardless of
// which roles are compiled in.
struct BeeboBasePrefs {
  uint8_t radio_fem_rxgain = 0;      // relocated from CommonCLI.h, see Part 2
#if !BEEBO_ENABLE_REPEATER_ROLE
  // beebo: adc_multiplier and node_lat/node_lon are genuinely native
  // ComPrefs fields upstream (see NodePrefs.h's build-conditional split) --
  // only declared here when ComPrefs isn't inherited at all (companion-only
  // static build). When BEEBO_ENABLE_REPEATER_ROLE is set, ComPrefs already
  // singly owns these via BeeboRepeaterPrefs; redeclaring them here too
  // would recreate exactly the ambiguous-member problem this whole
  // distillation exercise exists to avoid.
  float adc_multiplier = 0.0f;       // battery ADC divider multiplier override; 0.0f = use board default
#endif
  uint8_t adc_resolution_bits = 12;  // battery ADC sample resolution, bits (10 or 12)
  uint8_t batt_present = 0;
  uint16_t batt_sample_period_secs = 0;
  uint16_t batt_sample_window_secs = 0;
  uint16_t batt_charged_mv = 0;
  uint16_t idle_margin_ms = 0;
  uint32_t dedup_window_ms = DEDUP_LIVE_WINDOW_MS_DEFAULT;
#if !BEEBO_ENABLE_REPEATER_ROLE
  double node_lat = 0.0, node_lon = 0.0;
#endif
  char wifi_ssid[64] = {0};      // each role can run a different transport config --
  char wifi_pwd[64] = {0};       // e.g. repeater WiFi-only/no-BLE while companion keeps
  uint8_t ble_enabled = 1;       // BLE enabled for phone pairing. Independent per-role
  uint8_t tcp_enabled = 0;       // value, single declaration -- same pattern as
  uint8_t usb_enabled = 1;       // radio_fem_rxgain/dedup_window_ms above.
  uint32_t monring_config = 0;   // MonRing itself is one shared ring, but its config
                                 // (capture policy, etc.) can still differ per role.
  uint32_t monring_event_mask = 0xFFFFFFFFu;  // per-event-type MON_EVENT capture bitmask
                                 // (MonRing::_event_type_mask), same per-role-differs
                                 // rationale as monring_config above.
};
