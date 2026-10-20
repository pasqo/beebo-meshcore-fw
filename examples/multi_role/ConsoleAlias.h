#pragma once

// `beebo settings` tree names accepted by the text console: `<path>` reads,
// `<path> <value>` writes. Arduino-free so fw/test/ can build it. The table
// is generated from protocol.yaml's `console:` section.

#include <cstddef>
#include <cstdio>
#include <cstring>
#include "ConsoleAliasTable.h"

static const size_t CONSOLE_ALIAS_CMD_MAX = 168;   // room for "set <key> <value>" from a 160 char app command

// Rewrites `command` to the text-CLI form ("get <key>" / "set <key> <value>")
// into `out`. False, with `out` untouched, when the first token is not an
// aliased path or the result does not fit. An optional "repeater." or
// "companion." before the path is accepted.
inline bool consoleAliasRewrite(const char* command, char* out, size_t cap, bool* repeater_only = nullptr) {
  size_t n = 0;
  while (command[n] && command[n] != ' ') n++;
  const char* path = command;
  static const char* const roles[] = {"repeater.", "companion."};
  for (const char* role : roles) {
    size_t rl = strlen(role);
    if (n > rl && strncmp(path, role, rl) == 0) { path += rl; n -= rl; break; }
  }
  for (unsigned i = 0; i < CONSOLE_ALIAS_COUNT; i++) {
    const ConsoleAliasEntry& e = CONSOLE_ALIASES[i];
    if (strlen(e.path) != n || strncmp(e.path, path, n) != 0) continue;
    const char* value = path + n;
    while (*value == ' ') value++;
    size_t vlen = strlen(value);
    while (vlen && value[vlen - 1] == ' ') vlen--;
    int len = vlen ? snprintf(out, 0, "set %s %.*s", e.key, (int)vlen, value) : snprintf(out, 0, "get %s", e.key);
    if (len < 0 || (size_t)len >= cap) return false;
    if (vlen) snprintf(out, cap, "set %s %.*s", e.key, (int)vlen, value);
    else snprintf(out, cap, "get %s", e.key);
    if (repeater_only) *repeater_only = e.repeater_only;
    return true;
  }
  return false;
}
