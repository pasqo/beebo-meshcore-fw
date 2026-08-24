#pragma once
#include <stdint.h>

// Self-describing on-disk/wire format generation numbers, loaded first
// (DataStore::begin(), before any other persisted file), so every
// subsequent loader knows explicitly which layout to expect instead of
// inferring it from remaining byte count. An absent /beebo_abi means every
// version below is implicitly 0 -- a file saved before this mechanism
// existed, handled by each loader's own pre-ABI fallback path.
//
// Bump a version only when that file's on-disk layout actually changes;
// unrelated firmware/tool releases must not move it (see MonRing.h's
// MONRING_ABI_VERSION for the same convention applied to ring records).
struct BeeboAbi {
  uint8_t  board_prefs_version = 0;      // /beebo_board layout generation
  uint8_t  companion_prefs_version = 0;  // /beebo_companion layout generation
  uint8_t  repeater_prefs_version = 0;   // /beebo_repeater layout generation
  uint8_t  prefs_tlv_abi_version = 0;    // PREFS_TLV_FIELDS key-table generation
  char     tool_version[32] = {0};       // BEEBO_FW_VERSION at last save, diagnostics only
  uint32_t saved_at = 0;                 // unix time of last save, diagnostics only
};

// 2: batt_charged_mv/idle_margin_ms dropped from BeeboBoardPrefs (both
// unused since classifyBattTrend()/radioIsIdle() always use the compiled
// BATT_FULL_MV/IDLE_MARGIN_MS macros -- see BattTrend.h).
#define BOARD_PREFS_VERSION     2
#define COMPANION_PREFS_VERSION 2
#define REPEATER_PREFS_VERSION  2
// 1: first generation reported over GET_PREFS_TLV. Not yet wired onto the
// wire -- reserved for when a client needs to gate key usage on it.
#define PREFS_TLV_ABI_VERSION   1
