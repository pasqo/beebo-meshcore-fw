#pragma once

// `settings <path> [value]` on the text console: the `beebo settings` command of the CLI.
// `<path>` reads, `<path> <value>` writes, a branch or no path lists names. Arduino-free so
// fw/test/ can build it. The table is generated from protocol.yaml's `console:` section.

#include <cstddef>
#include <cstdio>
#include <cstring>
#include "ConsoleAliasTable.h"

static const size_t CONSOLE_ALIAS_CMD_MAX = 168;   // room for "set <key> <value>" from a 160 char app command

enum ConsoleSettingsResult {
  CONSOLE_NOT_SETTINGS,        // not a `settings` command; `out` untouched
  CONSOLE_SETTINGS_REWRITTEN,  // `out` is the "get <key>" / "set <key> <value>" text command
  CONSOLE_SETTINGS_LISTED,     // `out` is a list of names
  CONSOLE_SETTINGS_UNKNOWN,    // no such path (or a value given to a branch); `out` untouched
  CONSOLE_SETTINGS_TOO_LONG,   // the text command does not fit `out`; `out` untouched
};

// The part of `path` (n chars) after an optional "repeater." or "companion.".
inline const char* consolePathStrip(const char* path, size_t* n) {
  static const char* const roles[] = {"repeater.", "companion."};
  for (const char* role : roles) {
    size_t rl = strlen(role);
    if (*n > rl && strncmp(path, role, rl) == 0) { *n -= rl; return path + rl; }
  }
  return path;
}

// The names one level below `prefix` (n chars, "" for the top), each once, a branch with a
// trailing '.', comma separated into `out`; cut with "..." when it does not fit.
inline bool consoleListChildren(const char* prefix, size_t n, char* out, size_t cap) {
  size_t used = 0;
  bool any = false, cut = false;
  for (unsigned i = 0; i < CONSOLE_ALIAS_COUNT && !cut; i++) {
    const char* path = CONSOLE_ALIASES[i].path;
    if (n) {
      if (strncmp(path, prefix, n) != 0 || path[n] != '.') continue;
      path += n + 1;
    }
    const char* dot = strchr(path, '.');
    size_t len = dot ? (size_t)(dot - path) : strlen(path);
    bool dup = false;   // an earlier entry already gave this name
    for (unsigned j = 0; j < i && !dup; j++) {
      const char* q = CONSOLE_ALIASES[j].path;
      if (n && !(strncmp(q, prefix, n) == 0 && q[n] == '.')) continue;
      if (n) q += n + 1;
      dup = strncmp(q, path, len) == 0 && (q[len] == '.' || q[len] == 0) && (dot != nullptr) == (strchr(q, '.') != nullptr);
    }
    if (dup) continue;
    size_t need = (any ? 2 : 0) + len + (dot ? 1 : 0);
    if (used + need + 4 > cap) { cut = true; break; }   // room for "..."
    if (any) { out[used++] = ','; out[used++] = ' '; }
    else out[0] = 0;
    memcpy(out + used, path, len);
    used += len;
    if (dot) out[used++] = '.';
    out[used] = 0;
    any = true;
  }
  if (cut) { if (used + 4 <= cap) strcpy(out + used, "..."); }
  return any;
}

// Handles one command line. `repeater_only` is set for a path the companion role refuses,
// `companion_only` for one the repeater role refuses.
inline ConsoleSettingsResult consoleSettings(const char* command, char* out, size_t cap, bool* repeater_only = nullptr,
                                             bool* companion_only = nullptr) {
  while (*command == ' ') command++;
  if (strncmp(command, "settings", 8) != 0 || (command[8] != 0 && command[8] != ' ')) return CONSOLE_NOT_SETTINGS;
  const char* p = command + 8;
  while (*p == ' ') p++;
  size_t n = 0;
  while (p[n] && p[n] != ' ') n++;
  const char* path = consolePathStrip(p, &n);
  const char* value = path + n;
  while (*value == ' ') value++;
  size_t vlen = strlen(value);
  while (vlen && value[vlen - 1] == ' ') vlen--;

  if (n == 0) return consoleListChildren("", 0, out, cap) ? CONSOLE_SETTINGS_LISTED : CONSOLE_SETTINGS_UNKNOWN;
  for (unsigned i = 0; i < CONSOLE_ALIAS_COUNT; i++) {
    const ConsoleAliasEntry& e = CONSOLE_ALIASES[i];
    if (strlen(e.path) != n || strncmp(e.path, path, n) != 0) continue;
    int len = vlen ? snprintf(nullptr, 0, "set %s %.*s", e.key, (int)vlen, value) : snprintf(nullptr, 0, "get %s", e.key);
    if (len < 0 || (size_t)len >= cap) return CONSOLE_SETTINGS_TOO_LONG;
    if (vlen) snprintf(out, cap, "set %s %.*s", e.key, (int)vlen, value);
    else snprintf(out, cap, "get %s", e.key);
    if (repeater_only) *repeater_only = e.repeater_only;
    if (companion_only) *companion_only = e.companion_only;
    return CONSOLE_SETTINGS_REWRITTEN;
  }
  if (vlen == 0 && consoleListChildren(path, n, out, cap)) return CONSOLE_SETTINGS_LISTED;
  return CONSOLE_SETTINGS_UNKNOWN;
}
