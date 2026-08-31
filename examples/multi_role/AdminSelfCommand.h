#pragma once

// beebo: 'self' sentinel (plans/ADMIN_SELF_COMMAND.md) -- pulled out of Beebo.cpp into its
// own Arduino-free header so matchAdminSelfCommand() is unit-testable from fw/test/ (native
// platform, no Arduino/ESP32 dependency), alongside where handleAdminSelfCommand() (Beebo.cpp,
// Arduino-dependent) uses it.

#include <cstring>

// Case-sensitive match on the first whitespace-delimited token of an admin cmd's text.
// Returns a pointer to the remainder (the actual command) on match, NULL otherwise. text is
// not const since callers pass a pointer into a mutable buffer, and the returned pointer
// aliases it.
inline char* matchAdminSelfCommand(char* text) {
  if (strncmp(text, "self", 4) != 0) return NULL;
  if (text[4] == 0) return text + 4;   // bare "self", empty command
  if (text[4] != ' ') return NULL;     // e.g. "selfish ..." must not match
  return text + 5;
}
