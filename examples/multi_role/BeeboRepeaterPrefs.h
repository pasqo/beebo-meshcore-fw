#pragma once
#include <cstdint>
#include <helpers/SimpleMeshTables.h>

// beebo: repeater-role-only settings with no home in CommonCLI.h's real
// ComPrefs struct -- extracted to its own file, /beebo_repeater, rather than
// shadow-appended past ComPrefs's own serialized length in /com_prefs (as
// _fwd_dedup_window_ms used to be). That shadow trick stops working once
// CommonCLI::savePrefs() (the real upstream implementation) becomes the
// actual write path for /com_prefs: it truncates the file to exactly
// ComPrefs's own struct size on every save, which would silently wipe out
// any appended byte past that point. See
// beebo/plans/COMMONCLI_TEXT_DISPATCH.md's "Persistence" section.
struct BeeboRepeaterPrefs {  // persisted to /beebo_repeater
  // beebo: repeater's own independent dedup live-eviction window, ms (see
  // SimpleMeshTables.h's DEDUP_LIVE_WINDOW_MS_DEFAULT comment) -- companion
  // has its own independent copy too (BeeboCompanionPrefs.dedup_window_ms),
  // same SETTINGS_TREE_CLEANUP.md Decision A split as rx_delay_base/
  // airtime_factor -- each role persists its own value even though only one
  // role is ever live at a time, so switching roles doesn't lose tuning. 0 is a
  // real, literal "disabled" value once a file holds it, not "unset" --
  // matches _fwd_dedup_window_ms's existing in-class default exactly (this
  // struct's default is what a brand-new /beebo_repeater-less device gets,
  // same as _fwd_dedup_window_ms's own in-class initializer today).
  uint32_t dedup_window_ms = DEDUP_LIVE_WINDOW_MS_DEFAULT;
};
