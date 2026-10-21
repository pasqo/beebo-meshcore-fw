#pragma once

// The CLI's own spelling of node commands that the firmware knows by an older name
// (`neighbors forget` is `neighbor.remove`). Arduino-free so fw/test/ can build it; the table
// is generated from protocol.yaml's `console_names:` section.

#include <cstddef>
#include <cstdio>
#include <cstring>
#include "ConsoleNamesTable.h"

// Rewrites `command` into `out` when it starts with a CLI form (whole words); false, with `out`
// untouched, when it does not or the result does not fit.
inline bool consoleNameRewrite(const char* input, char* out, size_t cap) {
  char norm[168];   // single spaces between the words, none at the ends
  size_t k = 0;
  for (const char* p = input; *p && k + 1 < sizeof(norm); p++) {
    if (*p == ' ' && (k == 0 || norm[k - 1] == ' ')) continue;
    norm[k++] = *p;
  }
  while (k && norm[k - 1] == ' ') k--;
  norm[k] = 0;
  const char* command = norm;
  for (unsigned i = 0; i < CONSOLE_NAME_COUNT; i++) {
    const char* cli = CONSOLE_NAMES[i].cli;
    size_t n = strlen(cli);
    if (strncmp(command, cli, n) != 0 || (command[n] != 0 && command[n] != ' ')) continue;
    const char* rest = command + n;
    while (*rest == ' ') rest++;
    size_t rl = strlen(rest);
    while (rl && rest[rl - 1] == ' ') rl--;
    int len = rl ? snprintf(nullptr, 0, "%s %.*s", CONSOLE_NAMES[i].text, (int)rl, rest) : snprintf(nullptr, 0, "%s", CONSOLE_NAMES[i].text);
    if (len < 0 || (size_t)len >= cap) return false;
    if (rl) snprintf(out, cap, "%s %.*s", CONSOLE_NAMES[i].text, (int)rl, rest);
    else snprintf(out, cap, "%s", CONSOLE_NAMES[i].text);
    return true;
  }
  return false;
}
