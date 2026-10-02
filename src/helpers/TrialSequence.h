#pragma once
#include <stdint.h>

// beebo: run order and per-switch value lists of the on-device trial sequence
// (Beebo::loopTuning). Header-only, no Arduino dependency.
//
// A switch is one trialable setting. Its list is A first, then up to
// MAX_VALUES - 1 challengers. The sequence starts by storing every selected
// switch's v[0], so each first trial compares v[0] with v[1]; each later trial
// compares the winner (the value stored by then) with the next challenger. So a
// list of n values is n - 1 trials.
// Switch indexes are wire-stable (bit i of the switches mask); the run order is
// orderAt(), independent of them.
class TrialSequence {
public:
  // Switch indexes are wire-stable (bit i of the switches mask). Values are
  // held as whole numbers in the trial records: the float settings in steps of
  // 0.1 (rx delay base), 0.01 (tx and direct tx delay factors) and 0.05
  // (airtime factor), the AGC reset interval in its own 4 s units; the two
  // booleans CAD and multi acks are 0/1.
  enum Switch {
    LNA = 0, RX_BOOST = 1, CR = 2, AGC = 3, INTERFERENCE = 4,
    RX_DELAY = 5, TX_DELAY = 6, DIRECT_TX_DELAY = 7, AIRTIME = 8,
    CAD = 9, MULTI_ACKS = 10, NUM_SWITCHES = 11
  };
  static const int MAX_VALUES = 4;

  // Settings-tree / text-key name of each switch.
  static const char* name(int sw) {
    static const char* const names[NUM_SWITCHES] = {
      "lna", "rxboost", "cr", "agc", "interference", "rxdelay", "txdelay",
      "directtxdelay", "airtime", "cad", "multiacks" };
    return names[sw];
  }

  // The switch index run i-th: the receive path (LNA, RX boost, AGC), then
  // channel-busy behavior (interference threshold, CAD), the link settings that
  // cost airtime (coding rate, multi acks), then the timing knobs.
  static int orderAt(int i) {
    static const uint8_t order[NUM_SWITCHES] = {
      LNA, RX_BOOST, AGC, INTERFERENCE, CAD, CR, MULTI_ACKS, RX_DELAY, TX_DELAY,
      DIRECT_TX_DELAY, AIRTIME };
    return order[i];
  }

  // The first switch of ORDER whose bit is set in `pending`, -1 if none.
  static int nextSwitch(uint16_t pending) {
    for (int i = 0; i < NUM_SWITCHES; i++) {
      if (pending & (1u << orderAt(i))) return orderAt(i);
    }
    return -1;
  }

  // The switch whose name is the start of `text` followed by '.'; -1 if none.
  // *rest points past the dot.
  static int byName(const char* text, const char** rest) {
    for (int i = 0; i < NUM_SWITCHES; i++) {
      const char* n = name(i);
      int k = 0;
      while (n[k] && text[k] == n[k]) k++;
      if (!n[k] && text[k] == '.') {
        *rest = &text[k + 1];
        return i;
      }
    }
    return -1;
  }

  // Largest value each switch takes, in its wire units.
  static int maxValue(int sw) {
    switch (sw) {
      case LNA: case RX_BOOST: case CAD: case MULTI_ACKS: return 1;
      case CR: return 8;
      case AGC: return 255;
      case INTERFERENCE: return 9;
      case RX_DELAY: return 200;         // 20.0
      case TX_DELAY: case DIRECT_TX_DELAY: return 200;   // 2.00
      case AIRTIME: return 180;          // 9.00
      default: return -1;
    }
  }
  static bool validValue(int sw, uint8_t v) {
    if (sw < 0 || sw >= NUM_SWITCHES) return false;
    return v <= maxValue(sw) && (sw != CR || v >= 5);
  }

  // Parse "8,16,32" (spaces around numbers allowed) into `out` (room for
  // MAX_VALUES); the count, or -1 on anything else, an empty list, too many
  // values or a value above 255.
  static int parse(const char* text, uint8_t* out) {
    int n = 0;
    const char* p = text;
    while (true) {
      while (*p == ' ') p++;
      if (*p < '0' || *p > '9') return -1;
      unsigned v = 0;
      while (*p >= '0' && *p <= '9') {
        v = v * 10 + (unsigned)(*p - '0');
        if (v > 255) return -1;
        p++;
      }
      if (n >= MAX_VALUES) return -1;
      out[n++] = (uint8_t)v;
      while (*p == ' ') p++;
      if (*p == '\0') return n;
      if (*p != ',') return -1;
      p++;
    }
  }

  struct List {
    uint8_t v[MAX_VALUES];
    uint8_t n;     // at least 2
    uint8_t pos;   // index of the current challenger, from 1

    // Default list: A is the upstream default, then values across the range.
    void reset(int sw) {
      static const uint8_t table[NUM_SWITCHES][MAX_VALUES] = {
        {0, 1, 0, 0},            // LNA: off, on
        {1, 0, 0, 0},            // RX boost: on (the Heltec V4 default), off
        {5, 6, 7, 8},            // coding rate
        {0, 2, 4, 8},            // AGC reset interval: 0, 8, 16, 32 s
        {0, 3, 6, 9},            // interference threshold
        {0, 70, 130, 200},       // rx delay base: 0, 7, 13, 20
        {50, 0, 100, 200},       // tx delay factor: 0.5, 0, 1, 2
        {30, 0, 100, 200},       // direct tx delay factor: 0.3, 0, 1, 2
        {20, 10, 60, 180},       // airtime factor: 1.0, 0.5, 3, 9
        {0, 1, 0, 0},            // CAD: off, on
        {0, 1, 0, 0},            // multi acks: off, on
      };
      static const uint8_t counts[NUM_SWITCHES] = {2, 2, 4, 4, 4, 4, 4, 4, 4, 2, 2};
      for (int i = 0; i < MAX_VALUES; i++) v[i] = table[sw][i];
      n = counts[sw];
      pos = 1;
    }
    uint8_t first() const { return v[0]; }        // A: the value the sequence starts from
    uint8_t current() const { return v[pos]; }    // the current challenger (B)
    // Replace the list (A then challengers, 2 to MAX_VALUES values after
    // consecutive repeats collapse); a rejected call leaves it unchanged.
    bool set(int sw, const uint8_t* values, int count) {
      if (count < 1 || count > MAX_VALUES) return false;
      for (int i = 0; i < count; i++) if (!validValue(sw, values[i])) return false;
      // consecutive repeats collapse, which lets packed() pad with the last value
      uint8_t w[MAX_VALUES];
      int m = 0;
      for (int i = 0; i < count; i++) {
        if (m == 0 || values[i] != w[m - 1]) w[m++] = values[i];
      }
      if (m < 2) return false;
      for (int i = 0; i < m; i++) v[i] = w[i];
      n = (uint8_t)m;
      pos = 1;
      return true;
    }
    // Move to the next challenger; false when the last one was just decided.
    bool advance() {
      if (pos + 1 >= n) return false;
      pos++;
      return true;
    }
    void restart() { pos = 1; }
    // A is whatever is stored (tuning.trial.start stored): every value is a
    // challenger, the first one too.
    void fromStored() { pos = 0; }
    // "8,16,32" into buf (size at least 4 * MAX_VALUES); the length.
    int format(char* buf) const {
      int len = 0;
      for (int i = 0; i < n; i++) {
        if (i) buf[len++] = ',';
        unsigned x = v[i];
        if (x >= 100) buf[len++] = (char)('0' + x / 100);
        if (x >= 10) buf[len++] = (char)('0' + x / 10 % 10);
        buf[len++] = (char)('0' + x % 10);
      }
      buf[len] = '\0';
      return len;
    }
    // The four value bytes of one word (first value lowest), the unused ones
    // repeating the last value: the wire form of the list (BEEBO_CMD_GET_TUNING_
    // TRIAL_VALUES, whose OK frame carries exactly one u32) and the setting
    // event's value. A reader collapses consecutive repeats to get the list back.
    uint32_t packed() const {
      uint32_t w = 0;
      for (int i = 0; i < MAX_VALUES; i++) w |= (uint32_t)v[i < n ? i : n - 1] << (8 * i);
      return w;
    }
  };

  // One default-initialized list per switch.
  struct Lists {
    List v[NUM_SWITCHES];
    Lists() { for (int i = 0; i < NUM_SWITCHES; i++) v[i].reset(i); }
  };
};
