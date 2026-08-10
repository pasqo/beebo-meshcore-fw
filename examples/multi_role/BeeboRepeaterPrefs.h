#pragma once
#include <cstdint>

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1) -- currently
// empty of its own beebo-only fields (dedup_window_ms moved to
// BeeboBasePrefs, see that file). Kept as a distinct type anyway:
// DataStore's saveBeeboRepeaterPrefs/loadBeeboRepeaterPrefs signatures
// reference it, and it's the natural place for any future repeater-only
// beebo addition.
//
// `ComPrefs` (CommonCLI.h's own NodePrefs, aliased in Beebo.h) is only
// ever inherited by BeeboPrefs when BEEBO_ENABLE_REPEATER_ROLE is set --
// see BeeboPrefs.h -- but the type itself is declared unconditionally by
// Beebo.h's CommonCLI.h include, so this file (and the ComPrefs base
// below) can stay unguarded; it's simply unused/uninherited in a
// companion-only static build.
struct BeeboRepeaterPrefs : public ComPrefs {
};
