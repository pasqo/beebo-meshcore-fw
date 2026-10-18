#pragma once

// Arduino-free split of a CMD_RUN_CLI_COMMAND text, unit-testable from fw/test/.
// The companion app may prefix a command with a 2 character tag and '|' ("ab|get name");
// the reply carries the same prefix so the app can match it to the request.

#include <cstring>

// Length of a handler reply held in a `cap` byte buffer, clamped to `max_out` (the room a wire
// frame has left). Never reads past `cap`.
inline size_t cliReplyLen(const char* reply, size_t cap, size_t max_out) {
  size_t n = 0;
  while (n < cap && reply[n]) n++;
  return n < max_out ? n : max_out;
}

// Skips leading spaces. On a prefixed command (longer than 4 characters with '|' at index 2)
// copies the 3 prefix characters into `prefix` (room for 4) and returns the text after them;
// otherwise `prefix` is empty and the command is returned as is.
inline char* splitCliPrefix(char* command, char* prefix) {
  while (*command == ' ') command++;
  if (strlen(command) > 4 && command[2] == '|') {
    memcpy(prefix, command, 3);
    prefix[3] = 0;
    return command + 3;
  }
  prefix[0] = 0;
  return command;
}
