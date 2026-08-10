#pragma once

#include "BeeboCompanionPrefs.h"
#include "BeeboRepeaterPrefs.h"
#include "BeeboBasePrefs.h"

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1). Must be
// included from Beebo.h only, after the ComPrefs alias trick (#define
// NodePrefs ComPrefs / #include <helpers/CommonCLI.h> / #undef NodePrefs)
// -- BeeboRepeaterPrefs.h's `ComPrefs` base only resolves in that context.
//
// The 14 SharedPrefs-named fields (see NodePrefs.h) stay singly-owned:
// when BEEBO_ENABLE_REPEATER_ROLE is set, they come from ComPrefs via
// BeeboRepeaterPrefs and local NodePrefs is the distilled 11-field shape;
// when it's not set, BeeboRepeaterPrefs/ComPrefs isn't inherited at all and
// local NodePrefs is the full 25-field stock shape instead (see
// NodePrefs.h) -- so `_beebo_prefs.freq` etc. always resolves to exactly
// one base, in either build configuration, with no ambiguity and no
// caller-visible difference.
struct BeeboPrefs : public BeeboCompanionPrefs,
#if BEEBO_ENABLE_REPEATER_ROLE
                     public BeeboRepeaterPrefs,
#endif
                     public BeeboBasePrefs
{
  // beebo: SETTINGS_REFACTOR.md Part 3 -- role/board identity
  // (board_password/board_name) moved OUT of BeeboPrefs entirely, into
  // their own single BeeboBoardPrefs object (Beebo.h's `_board`) --
  // board-level state, one value for the whole device, not per-role data,
  // so it doesn't belong on the struct that becomes one array slot per
  // role (BeeboRoleState.h). See BeeboBoardPrefs.h.

  // beebo: SETTINGS_REFACTOR.md root-cause fix -- self-contained dirty
  // tracking for exactly what this struct now owns (BeeboCompanionPrefs/
  // BeeboRepeaterPrefs/BeeboBasePrefs), so a single generic save routes
  // correctly per role-state slot instead of every call site needing to
  // branch and pick a differently-named flag by hand. RAM-only
  // bookkeeping, never persisted -- every DataStore save/load function
  // writes/reads named fields individually (see DataStore.cpp), never a
  // raw struct dump, so this bool adds no wire-format risk.
  bool dirty = false;
};
