#pragma once
#include <cstdint>

#define NODE_ROLE_COMPANION   0
#define NODE_ROLE_REPEATER    1
#define NODE_ROLE_COUNT       2
// Sentinel role byte for any wire opcode taking an explicit role: means
// "whichever role is currently live" -- lets a caller that just wants its
// own current role's data skip a separate GET_NODE_ROLE round trip. Never a
// valid role_state_store[] index itself; resolved to _board.role before use.
#define NODE_ROLE_LIVE         0xFF

// beebo: compile-time default boot role, companion-first -- a build with
// both roles compiled in (heltec_v4_3_multi_role) defaults to companion; a
// static-role build's only compiled-in role wins regardless (the other
// branch's macro is always 0 there). One #elif per role, linear (not
// combinatorial) as more roles are added.
#if BEEBO_ENABLE_COMPANION_ROLE
  #define NODE_ROLE_DEFAULT NODE_ROLE_COMPANION
#elif BEEBO_ENABLE_REPEATER_ROLE
  #define NODE_ROLE_DEFAULT NODE_ROLE_REPEATER
#endif

// beebo: SETTINGS_REFACTOR.md Part 3 -- role_state_store[] (Beebo.h) is
// always sized NODE_ROLE_COUNT (2), regardless of which role(s) a
// given build compiles in, and is always indexed directly by the real
// NODE_ROLE_COMPANION/NODE_ROLE_REPEATER value -- no remapping macro. A
// static-role build wastes the other role's slot (never touched, since
// isNodeRoleBuiltIn() gates any access to the uncompiled role) rather than
// aliasing it onto the live role's slot.

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1). Genuinely
// single-valued, role-independent identity fields -- persisted to their
// own /beebo_board file, untouched by the role-switch park/load handoff
// entirely (unlike everything in BeeboBasePrefs, which is structurally
// single-declared but holds an independent value per role).
//
// `role` lives here, not as BeeboPrefs's own bare field -- it's board-level
// state ("which role is this board currently running"), not
// companion-role data, and /beebo_board is loaded once, first, at boot
// before either role's own file, so the role to activate is known before
// anything role-specific is touched (see DataStore::loadBeeboBoardPrefs()
// and Beebo::begin()).
struct BeeboBoardPrefs {
  uint8_t role = NODE_ROLE_DEFAULT; // node.role - companion/repeater role selection. Default = companion, unless only repeater is compiled in.
  char board_password[16] = {0};    // board.password - backup/recovery credential. Default = unset.
  char board_name[32] = {0};        // board.name - human-editable alias for this physical board's identity (board.id, the read-only eFuse MAC). Default = unset.

  // One physical VBAT ADC/battery per board, not one per role, so these
  // live here rather than in BeeboBasePrefs (which gives each role its own
  // independently-persisted copy). adc_multiplier is a deliberate
  // exception to the general "reuse upstream ComPrefs when the repeater
  // role is compiled in" convention BeeboBasePrefs.h otherwise follows --
  // that convention would leave companion with no storage for it at all
  // on a multi_role build.
  float adc_multiplier = 0.0f;       // board.adc.multiplier - battery ADC divider multiplier override; 0.0f = use board default
  uint8_t adc_resolution_bits = 12;  // board.adc.resolution - battery ADC sample resolution, bits (10 or 12)
  uint8_t batt_present = 0;          // board.battery.present
  uint16_t batt_sample_period_secs = 0;  // board.battery.sample_period
  uint16_t batt_sample_window_secs = 0;  // board.battery.sample_window
};
