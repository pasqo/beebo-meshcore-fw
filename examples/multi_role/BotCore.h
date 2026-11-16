#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// beebo: #test channel bot logic (no Arduino dependency, native-testable).
// Parses a group text, decides whether to answer, formats the reply and holds
// the single pending reply. The firmware glue (Beebo::onChannelMessageRecv()/
// botTick()) only feeds it packets and sends what due() returns.
// See plans/BOT.md.
namespace bot {

constexpr uint8_t STYLE_TEST = 1;       // test, ping: every sender, hops and path
constexpr uint8_t STYLE_NEIGHBOR = 2;   // nbr: direct senders only, link figures
constexpr uint8_t STYLE_MASK = 3;

constexpr uint8_t NAME_LEN_MAX = 31;
constexpr uint8_t SENDER_MAX = 32;
constexpr int MAX_TEXT = 160;           // BaseChatMesh MAX_TEXT_LEN
constexpr uint8_t MAX_HOP_CAP = 21;     // applyHopCap() protocol maximum
constexpr uint8_t SENDER_SLOTS = 8;
constexpr uint8_t MIN_REPLY_GAP = 2;
constexpr uint16_t MAX_SENDER_GAP = 3600;
constexpr uint8_t MAX_JITTER = 30;

// sha256("#test")[:16], the hashtag channel secret (docs/companion_protocol.md)
constexpr uint8_t TEST_SECRET[16] = {
  0x9c, 0xd8, 0xfc, 0xf2, 0x2a, 0x47, 0x33, 0x3b, 0x59, 0x1d, 0x96, 0xa2, 0xb8, 0x48, 0xb7, 0x3f};

// A channel secret is 32 bytes in the table, the 16-byte key followed by zeros.
inline bool isTestSecret(const uint8_t* secret32) {
  if (memcmp(secret32, TEST_SECRET, 16) != 0) return false;
  for (int i = 16; i < 32; i++) if (secret32[i]) return false;
  return true;
}

enum Kind : uint8_t { KIND_NONE, KIND_TEST, KIND_NEIGHBOR, KIND_HELP };

enum Verdict : uint8_t {
  V_IGNORE,           // not a trigger, or its style is off: not counted
  V_REPLY,            // reply queued
  V_DROP_SELF,
  V_DROP_NOT_DIRECT,
  V_DROP_PENDING,
  V_DROP_GAP,
  V_DROP_SENDER,
  V_COUNT
};

struct Config {
  uint8_t styles = 0;
  uint8_t reply_gap = 10;       // s, minimum between any two replies
  uint16_t sender_gap = 60;     // s, minimum between replies to one sender (0 = none)
  uint8_t reply_jitter = 4;     // s, maximum random delay before a reply
  char name[NAME_LEN_MAX + 1] = {0};   // sender name on replies, empty = node name

  void clamp() {
    styles &= STYLE_MASK;
    if (reply_gap < MIN_REPLY_GAP) reply_gap = MIN_REPLY_GAP;
    if (sender_gap > MAX_SENDER_GAP) sender_gap = MAX_SENDER_GAP;
    if (reply_jitter > MAX_JITTER) reply_jitter = MAX_JITTER;
    name[NAME_LEN_MAX] = 0;
  }
};

inline bool validStyles(uint8_t v) { return (v & ~STYLE_MASK) == 0; }

// No ": " (it would break every receiver's sender/body split) and no control characters.
inline bool validName(const char* n) {
  size_t len = strlen(n);
  if (len > NAME_LEN_MAX) return false;
  for (size_t i = 0; i < len; i++) {
    if ((uint8_t)n[i] < 0x20 || n[i] == 0x7f) return false;
    if (n[i] == ':' && n[i + 1] == ' ') return false;
  }
  return true;
}

// The reply greeting, picked by the random value.
constexpr const char* DEFAULT_NAME = "#beebo-bot";   // reply name until one is set
constexpr int NOISE_COUNT = 6;
inline const char* noise(uint16_t rnd) {
  static const char* const NOISES[NOISE_COUNT] = {"meow", "purr", "mrrp", "mew", "prrr", "nya"};
  return NOISES[rnd % NOISE_COUNT];
}

// BaseChatMesh::sendGroupMessage() truncates at MAX_TEXT minus "<name>: ".
inline int maxBodyLen(const char* name) { return MAX_TEXT - (int)strlen(name) - 2; }

struct Split {
  const char* sender;
  uint8_t sender_len;
  const char* body;
};

// "<sender>: <body>", split at the first ": ".
inline bool splitText(const char* text, Split& out) {
  const char* p = strstr(text, ": ");
  if (!p || p == text) return false;
  out.sender = text;
  out.sender_len = (uint8_t)(p - text > 255 ? 255 : p - text);
  out.body = p + 2;
  return true;
}

inline bool eqNoCase(const char* a, int alen, const char* lit) {
  if ((int)strlen(lit) != alen) return false;
  for (int i = 0; i < alen; i++) {
    char c = a[i];
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (c != lit[i]) return false;
  }
  return true;
}

inline Kind classify(const char* body, uint8_t styles) {
  while (*body == ' ') body++;
  int len = (int)strlen(body);
  while (len > 0 && body[len - 1] == ' ') len--;
  if (len == 0) return KIND_NONE;
  if (styles == 0) return KIND_NONE;
  if (eqNoCase(body, len, "h") || eqNoCase(body, len, "help")) return KIND_HELP;
  if ((styles & STYLE_TEST) && (eqNoCase(body, len, "test") || eqNoCase(body, len, "t") ||
                                eqNoCase(body, len, "ping") || eqNoCase(body, len, "p"))) return KIND_TEST;
  if ((styles & STYLE_NEIGHBOR) && (eqNoCase(body, len, "nbr") || eqNoCase(body, len, "n"))) return KIND_NEIGHBOR;
  return KIND_NONE;
}

// Repeater hops of a received flood packet, from its raw path.
struct Path {
  uint8_t hops = 0;
  uint8_t size = 1;               // bytes per hash
  const uint8_t* first = nullptr; // first repeater entry
};

// A hop-capped sender pre-fills the path with zero entries and then its own
// hash (applyHopCap), so the entries up to and including the first nonzero one
// are the sender, not hops. A cap leaving a single fill entry reads as one
// plain hop: the path cannot tell them apart.
inline Path decodePath(const uint8_t* path, uint8_t path_len) {
  Path r;
  r.size = (uint8_t)((path_len >> 6) + 1);
  uint8_t count = path_len & 63;
  r.first = path;
  uint8_t skipped = 0;
  while (skipped < count) {
    bool zero = true;
    for (uint8_t b = 0; b < r.size; b++) if (path[skipped * r.size + b]) { zero = false; break; }
    if (!zero) break;
    skipped++;
  }
  if (skipped == 0) {
    r.hops = count;
  } else if (skipped >= count) {
    r.hops = 0;
  } else {
    r.hops = (uint8_t)(count - skipped - 1);
    r.first = path + (skipped + 1) * r.size;
  }
  return r;
}

struct Link {
  Path path;
  int8_t snr_x4 = 0;       // SNR in quarter dB, as Packet::_snr
  int16_t rssi = 0;
  int16_t noise_dbm = 0;
};

class State {
public:
  void setConfig(const Config& c) { _cfg = c; _cfg.clamp(); }
  const Config& config() const { return _cfg; }

  // Decide on one received group text. On V_REPLY the reply text, its delay
  // and its hop cap are held until due() releases it.
  Verdict onMessage(uint32_t now_ms, const char* text, const char* node_name, const Link& link, uint16_t rnd) {
    Split sp;
    if (!splitText(text, sp)) return V_IGNORE;
    Kind kind = classify(sp.body, _cfg.styles);
    if (kind == KIND_NONE) return V_IGNORE;

    char sender[SENDER_MAX + 1];
    uint8_t slen = sp.sender_len > SENDER_MAX ? SENDER_MAX : sp.sender_len;
    memcpy(sender, sp.sender, slen);
    sender[slen] = 0;

    if (strcmp(sender, node_name) == 0 || (_cfg.name[0] && strcmp(sender, _cfg.name) == 0)) return drop(V_DROP_SELF);
    if (kind == KIND_NEIGHBOR && link.path.hops != 0) return drop(V_DROP_NOT_DIRECT);
    if (_pending) return drop(V_DROP_PENDING);
    if (_has_last && (uint32_t)(now_ms - _last_ms) < (uint32_t)_cfg.reply_gap * 1000u) return drop(V_DROP_GAP);
    Entry* e = findSender(sender, kind);
    if (e && _cfg.sender_gap && (uint32_t)(now_ms - e->t) < (uint32_t)_cfg.sender_gap * 1000u) return drop(V_DROP_SENDER);

    if (!e) e = newSender();
    strcpy(e->sender, sender);
    e->kind = kind;
    e->t = now_ms;
    e->used = true;

    format(kind, sender, _cfg.name[0] ? _cfg.name : node_name, link, rnd);
    _deadline = now_ms + (uint32_t)(((uint64_t)rnd * (uint32_t)_cfg.reply_jitter * 1000u) >> 16);
    uint8_t cap = (uint8_t)(link.path.hops >= MAX_HOP_CAP ? MAX_HOP_CAP : link.path.hops + 1);
    _hop_cap = cap;
    _pending = true;
    return V_REPLY;
  }

  // The reply text once its delay has passed, else null. Releasing it starts the global gap.
  const char* due(uint32_t now_ms) {
    if (!_pending || (int32_t)(now_ms - _deadline) < 0) return nullptr;
    _pending = false;
    _has_last = true;
    _last_ms = now_ms;
    _replies++;
    return _text;
  }

  void cancel() { _pending = false; }   // role switch, transmit off, channel gone

  bool pending() const { return _pending; }
  const char* pendingText() const { return _text; }
  uint8_t pendingHopCap() const { return _hop_cap; }
  uint32_t replies() const { return _replies; }
  uint32_t dropped() const { return _dropped; }
  uint32_t droppedBy(Verdict v) const { return v < V_COUNT ? _dropped_by[v] : 0; }

private:
  struct Entry {
    char sender[SENDER_MAX + 1];
    uint8_t kind;
    bool used = false;
    uint32_t t;
  };

  Verdict drop(Verdict v) { _dropped++; _dropped_by[v]++; return v; }

  Entry* findSender(const char* sender, Kind kind) {
    for (auto& e : _table) if (e.used && e.kind == kind && strcmp(e.sender, sender) == 0) return &e;
    return nullptr;
  }

  // A free slot, else the oldest entry.
  Entry* newSender() {
    Entry* oldest = &_table[0];
    for (auto& e : _table) {
      if (!e.used) return &e;
      if ((int32_t)(oldest->t - e.t) > 0) oldest = &e;
    }
    return oldest;
  }

  static int hexPath(char* out, int cap, const Path& p, bool shorten) {
    int n = 0;
    auto entry = [&](uint8_t i) {
      for (uint8_t b = 0; b < p.size && n + 2 < cap; b++) n += snprintf(out + n, cap - n, "%02x", p.first[i * p.size + b]);
    };
    if (!shorten || p.hops <= 2) {
      for (uint8_t i = 0; i < p.hops; i++) {
        if (i && n + 1 < cap) out[n++] = '>';
        entry(i);
      }
    } else {
      entry(0);
      if (n + 4 < cap) { memcpy(out + n, ">..>", 4); n += 4; }
      entry(p.hops - 1);
    }
    out[n] = 0;
    return n;
  }

  void format(Kind kind, const char* sender, const char* name, const Link& l, uint16_t rnd) {
    int limit = maxBodyLen(name);
    if (limit > MAX_TEXT) limit = MAX_TEXT;
    char detail[MAX_TEXT + 1];
    int n = 0;
    if (kind == KIND_HELP) {
      char limits[80];
      int m = snprintf(limits, sizeof(limits), "Limits: 1 reply/%us", (unsigned)_cfg.reply_gap);
      if (_cfg.sender_gap) m += snprintf(limits + m, sizeof(limits) - m, ", 1/sender/%us", (unsigned)_cfg.sender_gap);
      else m += snprintf(limits + m, sizeof(limits) - m, ", no per-sender limit");
      if (_cfg.reply_jitter) m += snprintf(limits + m, sizeof(limits) - m, ", up to %us delay", (unsigned)_cfg.reply_jitter);
      // Full text first; the nbr detail, then the disclaimer, go when it does not fit.
      for (int level = 0; level < 3; level++) {
        n = 0;
        detail[0] = 0;
        if (_cfg.styles & STYLE_TEST) n += snprintf(detail + n, sizeof(detail) - n, "t/test/p/ping: hops and path. ");
        if (_cfg.styles & STYLE_NEIGHBOR)
          n += snprintf(detail + n, sizeof(detail) - n, level == 0 ? "n/nbr: SNR/RSSI/margin, direct only. " : "n/nbr: direct only. ");
        n += snprintf(detail + n, sizeof(detail) - n, "%s.%s", limits, level < 2 ? " Best effort, no guarantee." : "");
        if (4 + (int)strlen(sender) + (int)strlen(detail) <= limit) break;
      }
      if (4 + (int)strlen(sender) + (int)strlen(detail) > limit) {
        char trig[16];
        trig[0] = 0;
        if (_cfg.styles & STYLE_TEST) strcat(trig, "test/ping");
        if (_cfg.styles & STYLE_NEIGHBOR) strcat(trig, trig[0] ? "/nbr" : "nbr");
        n = snprintf(detail, sizeof(detail), "%s: 1 reply/%us", trig, (unsigned)_cfg.reply_gap);
        if (_cfg.sender_gap) n += snprintf(detail + n, sizeof(detail) - n, ", 1/sender/%us", (unsigned)_cfg.sender_gap);
        else n += snprintf(detail + n, sizeof(detail) - n, ", no per-sender limit");
        if (_cfg.reply_jitter) n += snprintf(detail + n, sizeof(detail) - n, ", up to %us delay", (unsigned)_cfg.reply_jitter);
        snprintf(detail + n, sizeof(detail) - n, ", no guarantee");
      }
      snprintf(_text, sizeof(_text), "@[%s] %s", sender, detail);
    } else {
      if (l.path.hops == 0) {
        if (kind == KIND_NEIGHBOR) {
          int a = l.snr_x4 < 0 ? -(int)l.snr_x4 : (int)l.snr_x4;
          int tenths = (a * 10 + 2) / 4;
          n = snprintf(detail, sizeof(detail), "direct, SNR %s%d.%d, RSSI %d, margin %d dB",
                       (l.snr_x4 < 0 && tenths > 0) ? "-" : "", tenths / 10, tenths % 10,
                       (int)l.rssi, (int)l.rssi - (int)l.noise_dbm);
        } else {
          n = snprintf(detail, sizeof(detail), "0 hops");
        }
      } else {
        char path[MAX_TEXT + 1];
        const char* tail = ". Use h for policy";
        const char* hi = noise(rnd);
        for (int pass = 0; pass < 2; pass++) {
          hexPath(path, sizeof(path), l.path, pass == 1);
          snprintf(detail, sizeof(detail), "%u hop%s via %s", (unsigned)l.path.hops, l.path.hops == 1 ? "" : "s", path);
          int total = (int)(strlen(sender) + 4 + strlen(hi) + 2 + strlen(detail) + strlen(tail));
          if (total <= limit) break;
        }
      }
      snprintf(_text, sizeof(_text), "@[%s] %s: %s. Use h for policy", sender, noise(rnd), detail);
    }
    if ((int)strlen(_text) > limit) _text[limit] = 0;
  }

  Config _cfg;
  Entry _table[SENDER_SLOTS];
  char _text[MAX_TEXT + 1] = {0};
  bool _pending = false;
  bool _has_last = false;
  uint8_t _hop_cap = 0;
  uint32_t _deadline = 0;
  uint32_t _last_ms = 0;
  uint32_t _replies = 0;
  uint32_t _dropped = 0;
  uint32_t _dropped_by[V_COUNT] = {0};
};

}  // namespace bot
