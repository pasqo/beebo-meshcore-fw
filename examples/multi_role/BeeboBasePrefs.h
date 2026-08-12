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
// node_lat/node_lon are genuinely native ComPrefs fields upstream, but
// companion needs them too and, in a heltec_v4_3_companion static build
// (BEEBO_ENABLE_REPEATER_ROLE=0), BeeboPrefs has no ComPrefs base to
// source them from at all -- see NodePrefs.h's build-conditional split.
// Declaring them here instead of relying on ComPrefs inheritance keeps
// BeeboPrefs field access uniform (`_beebo_prefs.node_lat` always
// resolves the same way) regardless of which roles are compiled in.
// adc_multiplier used to follow this same pattern but was pulled out
// into BeeboBoardPrefs (board.adc.multiplier) -- see BOARD_BATTERY_PREFS.md
// -- since it isn't per-role data in the first place; ComPrefs's own
// adc_multiplier field (still reachable via BeeboRepeaterPrefs on a
// repeater-enabled build) is intentionally left unread/unwritten now.
struct BeeboBasePrefs {
  uint8_t radio_fem_rxgain = 0;      // relocated from CommonCLI.h, see Part 2
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
