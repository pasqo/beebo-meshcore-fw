#pragma once
#include <cstdint>
#include <cstring>
#include "BotCore.h"
#include "NodePrefs.h"
#include "BeeboBoardPrefs.h"

// Currently
// empty of its own beebo-only fields (all moved to BeeboBasePrefs, see that
// file). Kept as a distinct type anyway: DataStore's
// saveBeeboCompanionPrefs/loadBeeboCompanionPrefs signatures reference it,
// and it's the natural place for any future companion-only beebo addition.
// NodePrefs here is the local NodePrefs.h shape (distilled or full,
// depending on BEEBO_ENABLE_REPEATER_ROLE -- see that file).
struct BeeboCompanionPrefs : public NodePrefs {
  // #test channel bot (BotCore.h, plans/BOT.md). Companion role only; the
  // fields are stored together at the tail of /beebo_companion.
  uint8_t bot = 0;               // style bitmask: 1 test, 2 neighbor, 0 off
  uint8_t bot_reply_gap = 10;    // s between any two replies, 2..255
  uint16_t bot_sender_gap = 60;  // s between replies to one sender, 0 = no limit
  uint8_t bot_reply_jitter = 4;  // s, maximum random delay before a reply
  char bot_name[32] = {0};       // sender name on replies, empty = node name; default #beebo-bot

  // Beebo's constructor memsets the live slot to zero before applying
  // defaults, and the loader needs them before reading an old file's tail.
  void setBotDefaults() {
    bot = 0;
    bot_reply_gap = 10;
    bot_sender_gap = 60;
    bot_reply_jitter = 4;
    memset(bot_name, 0, sizeof(bot_name));
    strcpy(bot_name, bot::DEFAULT_NAME);
  }
};
