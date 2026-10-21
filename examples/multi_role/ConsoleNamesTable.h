#pragma once

// GENERATED FILE -- do not hand-edit. Source: protocol.yaml. Regenerate with `python tools/gen_protocol.py`.

struct ConsoleNameEntry {
  const char* cli;   // the CLI's command words, with the flag where one selects it
  const char* text;  // the node's text command
};

static const ConsoleNameEntry CONSOLE_NAMES[] = {
  {"neighbors peek", "neighbors"},
  {"neighbors forget", "neighbor.remove"},
  {"neighbors discover", "discover.neighbors"},
  {"region tree", "region"},
  {"region allow", "region allowf"},
  {"region deny", "region denyf"},
  {"advert flood", "advert"},
  {"advert direct", "advert.zerohop"},
};
static const unsigned CONSOLE_NAME_COUNT = sizeof(CONSOLE_NAMES) / sizeof(CONSOLE_NAMES[0]);
