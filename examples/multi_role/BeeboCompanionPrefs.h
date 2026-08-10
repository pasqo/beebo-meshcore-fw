#pragma once
#include <cstdint>
#include "NodePrefs.h"
#include "BeeboBoardPrefs.h" // beebo: NODE_ROLE_COMPANION/NODE_ROLE_REPEATER now live here (role is board-level state, see that file)

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1) -- currently
// empty of its own beebo-only fields (all moved to BeeboBasePrefs, see that
// file). Kept as a distinct type anyway: DataStore's
// saveBeeboCompanionPrefs/loadBeeboCompanionPrefs signatures reference it,
// and it's the natural place for any future companion-only beebo addition.
// NodePrefs here is the local NodePrefs.h shape (distilled or full,
// depending on BEEBO_ENABLE_REPEATER_ROLE -- see that file).
struct BeeboCompanionPrefs : public NodePrefs {
};
