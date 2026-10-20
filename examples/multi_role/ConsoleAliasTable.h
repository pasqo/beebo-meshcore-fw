#pragma once

// GENERATED FILE -- do not hand-edit. Source: protocol.yaml. Regenerate with `python tools/gen_protocol.py`.

struct ConsoleAliasEntry {
  const char* path;   // `beebo settings` tree path, without the role prefix
  const char* key;    // the text-CLI key it stands for
  bool repeater_only; // companion role answers with an error
};

static const ConsoleAliasEntry CONSOLE_ALIASES[] = {
  {"board.node.role", "node.role", false},
  {"radio.txpower", "tx", false},
  {"routing.tuning.trial.enabled", "tuning.trial.enabled", true},
  {"routing.tuning.trial.profile", "tuning.trial.profile", true},
  {"routing.tuning.trial.start", "tuning.trial.start", true},
  {"routing.tuning.trial.combined", "tuning.trial.combined", true},
  {"routing.tuning.trial.confidence_pct", "tuning.trial.confidence_pct", true},
  {"routing.tuning.trial.min_gain_pct", "tuning.trial.min_gain_pct", true},
  {"routing.tuning.trial.min_pkt_rate", "tuning.trial.min_pkt_rate", true},
  {"routing.tuning.trial.block_size_s", "tuning.trial.block_size_s", true},
  {"routing.tuning.trial.block_count", "tuning.trial.block_count", true},
};
static const unsigned CONSOLE_ALIAS_COUNT = sizeof(CONSOLE_ALIASES) / sizeof(CONSOLE_ALIASES[0]);
