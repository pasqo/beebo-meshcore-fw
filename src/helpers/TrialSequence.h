#pragma once
#include <stdint.h>

// beebo: run order and per-switch challenger lists of the on-device trial
// sequence (Beebo::loopTuning). Header-only, no Arduino dependency.
//
// A switch is one trialable setting. Its list holds up to MAX_VALUES
// challenger values; each is tried in listed order against whatever the
// stored value is at that point, so an adopted winner is the next challenger's
// arm A. Switch indexes are wire-stable (bit i of the switches mask); the run
// order is ORDER, independent of them.
class TrialSequence {
public:
  enum Switch { LNA = 0, RX_BOOST = 1, CR = 2, NUM_SWITCHES = 3 };
  static const int MAX_VALUES = 4;

  // Switch indexes in the order the sequence runs them.
  static constexpr uint8_t ORDER[NUM_SWITCHES] = { LNA, RX_BOOST, CR };

  // The first switch of ORDER whose bit is set in `pending`, -1 if none.
  static int nextSwitch(uint16_t pending) {
    for (int i = 0; i < NUM_SWITCHES; i++) {
      if (pending & (1u << ORDER[i])) return ORDER[i];
    }
    return -1;
  }

  static bool validValue(int sw, uint8_t v) {
    switch (sw) {
      case LNA: case RX_BOOST: return v <= 1;
      case CR: return v >= 5 && v <= 8;
      default: return false;
    }
  }

  static uint8_t defaultValue(int sw) { return sw == CR ? 8 : 0; }

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
    uint8_t n;
    uint8_t pos;

    void reset(int sw) { v[0] = defaultValue(sw); n = 1; pos = 0; }
    uint8_t current() const { return v[pos]; }
    // Replace the list; a rejected call leaves it unchanged.
    bool set(int sw, const uint8_t* values, int count) {
      if (count < 1 || count > MAX_VALUES) return false;
      for (int i = 0; i < count; i++) if (!validValue(sw, values[i])) return false;
      // consecutive repeats collapse, which lets packed() pad with the last value
      n = 0;
      for (int i = 0; i < count; i++) {
        if (n == 0 || values[i] != v[n - 1]) v[n++] = values[i];
      }
      pos = 0;
      return true;
    }
    // Move to the next challenger; false when the last one was just decided.
    bool advance() {
      if (pos + 1 >= n) return false;
      pos++;
      return true;
    }
    void restart() { pos = 0; }
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
};
