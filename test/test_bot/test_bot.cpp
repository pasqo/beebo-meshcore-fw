#include <gtest/gtest.h>
#include <initializer_list>
#include <string.h>
#include <BotCore.h>

using namespace bot;

namespace {

// Raw packet path as the receiver sees it: path_len byte = ((size - 1) << 6) | count.
struct RawPath {
  uint8_t bytes[64];
  uint8_t path_len = 0;
  RawPath() { memset(bytes, 0, sizeof(bytes)); }
  RawPath& add(std::initializer_list<uint8_t> entry, uint8_t size) {
    for (uint8_t b : entry) bytes[path_len_bytes++] = b;
    count++;
    path_len = (uint8_t)(((size - 1) << 6) | count);
    return *this;
  }
  uint8_t path_len_bytes = 0;
  uint8_t count = 0;
};

Link makeLink(const RawPath& p, int8_t snr_x4 = 22, int16_t rssi = -90, int16_t noise = -112) {
  Link l;
  l.path = decodePath(p.bytes, p.path_len);
  l.snr_x4 = snr_x4;
  l.rssi = rssi;
  l.noise_dbm = noise;
  return l;
}

const uint32_t T0 = 1000000;

Config cfg(uint8_t styles) {
  Config c;
  c.styles = styles;
  return c;
}

// ---- channel secret ----

TEST(BotChannel, SecretIsTheDocumentedHashtagKey) {
  // sha256("#test")[:16], docs/companion_protocol.md (the native SHA256 is a mock, so no live hash here;
  // the hardware test checks the real firmware against a channel the CLI derived from the name).
  const uint8_t expected[16] = {0x9c, 0xd8, 0xfc, 0xf2, 0x2a, 0x47, 0x33, 0x3b,
                                0x59, 0x1d, 0x96, 0xa2, 0xb8, 0x48, 0xb7, 0x3f};
  EXPECT_EQ(0, memcmp(expected, TEST_SECRET, 16));
}

TEST(BotChannel, MatchesSixteenByteSecretWithZeroTail) {
  uint8_t s[32] = {0};
  memcpy(s, TEST_SECRET, 16);
  EXPECT_TRUE(isTestSecret(s));
  s[20] = 1;
  EXPECT_FALSE(isTestSecret(s));   // a 32-byte secret that only starts like it
  s[20] = 0;
  s[0] ^= 1;
  EXPECT_FALSE(isTestSecret(s));
}

// ---- text parsing ----

TEST(BotParse, SplitsAtFirstColonSpace) {
  Split s;
  ASSERT_TRUE(splitText("alice: test", s));
  EXPECT_EQ(5, s.sender_len);
  EXPECT_EQ(0, strncmp(s.sender, "alice", 5));
  EXPECT_STREQ("test", s.body);
  ASSERT_TRUE(splitText("a: b: test", s));
  EXPECT_EQ(1, s.sender_len);
  EXPECT_STREQ("b: test", s.body);
}

TEST(BotParse, RejectsTextWithoutSenderPrefix) {
  Split s;
  EXPECT_FALSE(splitText("test", s));
  EXPECT_FALSE(splitText("alice:test", s));
  EXPECT_FALSE(splitText(": test", s));
  EXPECT_FALSE(splitText("", s));
}

TEST(BotParse, ClassifiesTriggersCaseAndWhitespaceInsensitive) {
  EXPECT_EQ(KIND_TEST, classify("test", STYLE_MASK));
  EXPECT_EQ(KIND_TEST, classify("  PiNg ", STYLE_MASK));
  EXPECT_EQ(KIND_NEIGHBOR, classify("nbr", STYLE_MASK));
  EXPECT_EQ(KIND_HELP, classify("h", STYLE_MASK));
  EXPECT_EQ(KIND_HELP, classify("Help", STYLE_TEST));
}

TEST(BotParse, NearMissesAreNotTriggers) {
  const char* misses[] = {"hello", "testing", "tes", "test!", "nbrs", "hh", "help me", "", " ", "@[x] test",
                          "test test"};
  for (const char* m : misses) EXPECT_EQ(KIND_NONE, classify(m, STYLE_MASK)) << m;
}

TEST(BotParse, TriggerNeedsItsStyle) {
  EXPECT_EQ(KIND_NONE, classify("test", STYLE_NEIGHBOR));
  EXPECT_EQ(KIND_NONE, classify("ping", STYLE_NEIGHBOR));
  EXPECT_EQ(KIND_NONE, classify("nbr", STYLE_TEST));
  EXPECT_EQ(KIND_NONE, classify("h", 0));
  EXPECT_EQ(KIND_NONE, classify("test", 0));
}

// ---- settings validation ----

TEST(BotConfig, StyleBitsAndNames) {
  EXPECT_TRUE(validStyles(0));
  EXPECT_TRUE(validStyles(3));
  EXPECT_FALSE(validStyles(4));
  EXPECT_FALSE(validStyles(0x80));
}

TEST(BotConfig, NameRules) {
  EXPECT_TRUE(validName(""));
  EXPECT_TRUE(validName("Beebo-Bot"));
  EXPECT_TRUE(validName("a:b"));
  EXPECT_FALSE(validName("a: b"));
  EXPECT_FALSE(validName("tab\there"));
  EXPECT_FALSE(validName("del\x7f"));
  EXPECT_TRUE(validName("0123456789012345678901234567890"));     // 31
  EXPECT_FALSE(validName("01234567890123456789012345678901"));   // 32
}

TEST(BotConfig, DefaultsAndClamp) {
  Config c;
  EXPECT_EQ(0, c.styles);
  EXPECT_EQ(10, c.reply_gap);
  EXPECT_EQ(60, c.sender_gap);
  EXPECT_EQ(4, c.reply_jitter);
  EXPECT_STREQ("", c.name);
  c.reply_gap = 0;
  c.sender_gap = 9999;
  c.reply_jitter = 200;
  c.styles = 0xFF;
  c.clamp();
  EXPECT_EQ(2, c.reply_gap);
  EXPECT_EQ(3600, c.sender_gap);
  EXPECT_EQ(30, c.reply_jitter);
  EXPECT_EQ(3, c.styles);
  c.reply_gap = 1;
  c.clamp();
  EXPECT_EQ(2, c.reply_gap);
}

// ---- path decoding ----

TEST(BotPath, NoPathIsDirect) {
  RawPath p;
  Path d = decodePath(p.bytes, p.path_len);
  EXPECT_EQ(0, d.hops);
}

TEST(BotPath, PlainPathCountsEveryEntry) {
  RawPath one;
  one.add({0x3f}, 1).add({0xa2}, 1);
  Path d = decodePath(one.bytes, one.path_len);
  EXPECT_EQ(2, d.hops);
  EXPECT_EQ(1, d.size);
  EXPECT_EQ(0x3f, d.first[0]);

  RawPath two;
  two.add({0x3f, 0x01}, 2).add({0xa2, 0x02}, 2).add({0x07, 0x03}, 2);
  d = decodePath(two.bytes, two.path_len);
  EXPECT_EQ(3, d.hops);
  EXPECT_EQ(2, d.size);

  RawPath three;
  three.add({0x3f, 0x01, 0x02}, 3);
  d = decodePath(three.bytes, three.path_len);
  EXPECT_EQ(1, d.hops);
  EXPECT_EQ(3, d.size);
}

TEST(BotPath, CappedSenderPrefillIsNotCountedAsHops) {
  // applyHopCap: zero entries, then the sender's own hash, then each repeater appends.
  RawPath p;
  p.add({0, 0, 0}, 3).add({0, 0, 0}, 3).add({0, 0, 0}, 3).add({0xaa, 0xbb, 0xcc}, 3);
  Path d = decodePath(p.bytes, p.path_len);
  EXPECT_EQ(0, d.hops);   // the sender's own hash is the last entry: a direct neighbor
  p.add({0x11, 0x22, 0x33}, 3).add({0x44, 0x55, 0x66}, 3);
  d = decodePath(p.bytes, p.path_len);
  EXPECT_EQ(2, d.hops);
  EXPECT_EQ(3, d.size);
  EXPECT_EQ(0x11, d.first[0]);
  EXPECT_EQ(0x44, d.first[3]);
}

TEST(BotPath, AllZeroPathIsTreatedAsDirect) {
  RawPath p;
  p.add({0, 0, 0}, 3).add({0, 0, 0}, 3);
  EXPECT_EQ(0, decodePath(p.bytes, p.path_len).hops);
}

TEST(BotPath, CapWithSingleFillEntryReadsOneExtraHop) {
  // max_hops 20 leaves one entry (the sender's hash), which is the same bytes as one
  // plain repeater hop: the path alone cannot tell them apart. Documented limit.
  RawPath p;
  p.add({0xaa, 0xbb, 0xcc}, 3);
  EXPECT_EQ(1, decodePath(p.bytes, p.path_len).hops);
}

// ---- reply text ----

struct Fix {
  Config c;
  State s;
  Fix(uint8_t styles = STYLE_MASK) { c = cfg(styles); s.setConfig(c); }
  Verdict msg(uint32_t now, const char* text, const Link& l, const char* node = "Beebo", uint16_t rnd = 0) {
    return s.onMessage(now, text, node, l, rnd);
  }
};

TEST(BotReply, TestDirect) {
  Fix f;
  RawPath p;
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: test", makeLink(p)));
  EXPECT_STREQ("@[alice] got it: direct. Use h for policy", f.s.pendingText());
}

TEST(BotReply, TestMultiHopShowsHopsAndPath) {
  Fix f;
  RawPath p;
  p.add({0x3f}, 1).add({0xa2}, 1);
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: ping", makeLink(p)));
  EXPECT_STREQ("@[alice] got it: 2 hops via 3f>a2. Use h for policy", f.s.pendingText());
}

TEST(BotReply, SingleHopIsSingular) {
  Fix f;
  RawPath p;
  p.add({0x07}, 1);
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: test", makeLink(p)));
  EXPECT_STREQ("@[alice] got it: 1 hop via 07. Use h for policy", f.s.pendingText());
}

TEST(BotReply, TestDoesNotReportLinkFiguresOfARepeater) {
  Fix f;
  RawPath p;
  p.add({0x3f}, 1);
  f.msg(T0, "alice: test", makeLink(p));
  EXPECT_EQ(nullptr, strstr(f.s.pendingText(), "SNR"));
  EXPECT_EQ(nullptr, strstr(f.s.pendingText(), "RSSI"));
}

TEST(BotReply, NeighborDirectReportsLinkAndMargin) {
  Fix f;
  RawPath p;
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: nbr", makeLink(p, 22, -90, -112)));
  EXPECT_STREQ("@[alice] got it: direct, SNR 5.5, RSSI -90, margin 22 dB. Use h for policy", f.s.pendingText());
}

TEST(BotReply, SnrFormatsTenthsAndSign) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: nbr", makeLink(p, -50, -120, -112));
  EXPECT_STREQ("@[a] got it: direct, SNR -12.5, RSSI -120, margin -8 dB. Use h for policy", f.s.pendingText());
  f.s.cancel();
  f.msg(T0 + 100000, "b: nbr", makeLink(p, -2, -90, -112));
  EXPECT_NE(nullptr, strstr(f.s.pendingText(), "SNR -0.5,"));
  f.s.cancel();
  f.msg(T0 + 200000, "c: nbr", makeLink(p, 0, -90, -112));
  EXPECT_NE(nullptr, strstr(f.s.pendingText(), "SNR 0.0,"));
}

TEST(BotReply, HelpListsEnabledTriggersAndLiveLimits) {
  Fix f;
  RawPath p;
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: h", makeLink(p)));
  EXPECT_STREQ("@[alice] test/ping/nbr: 1 reply/10s, 1/sender/60s, up to 4s delay, no guarantee", f.s.pendingText());
}

TEST(BotReply, HelpOnlyListsOwnStyle) {
  Fix a(STYLE_TEST);
  RawPath p;
  a.msg(T0, "x: help", makeLink(p));
  EXPECT_NE(nullptr, strstr(a.s.pendingText(), "test/ping:"));
  EXPECT_EQ(nullptr, strstr(a.s.pendingText(), "nbr"));
  Fix b(STYLE_NEIGHBOR);
  b.msg(T0, "x: help", makeLink(p));
  EXPECT_NE(nullptr, strstr(b.s.pendingText(), "nbr:"));
  EXPECT_EQ(nullptr, strstr(b.s.pendingText(), "test"));
}

TEST(BotReply, HelpReflectsChangedSettings) {
  Fix f;
  f.c.reply_gap = 30;
  f.c.sender_gap = 0;
  f.c.reply_jitter = 0;
  f.s.setConfig(f.c);
  RawPath p;
  f.msg(T0, "x: h", makeLink(p));
  EXPECT_STREQ("@[x] test/ping/nbr: 1 reply/30s, no per-sender limit, no guarantee", f.s.pendingText());
}

TEST(BotReply, LongPathShortensToFirstAndLast) {
  Fix f;
  RawPath p;
  for (uint8_t i = 1; i <= 21; i++) p.add({i, (uint8_t)(i + 1), (uint8_t)(i + 2)}, 3);
  ASSERT_EQ(V_REPLY, f.msg(T0, "alice: test", makeLink(p)));
  EXPECT_STREQ("@[alice] got it: 21 hops via 010203>..>151617. Use h for policy", f.s.pendingText());
}

TEST(BotReply, ShortPathPrintsEveryHop) {
  Fix f;
  RawPath p;
  p.add({1, 2, 3}, 3).add({4, 5, 6}, 3).add({7, 8, 9}, 3);
  f.msg(T0, "alice: test", makeLink(p));
  EXPECT_STREQ("@[alice] got it: 3 hops via 010203>040506>070809. Use h for policy", f.s.pendingText());
}

TEST(BotReply, WorstCaseFitsSendGroupMessageBudget) {
  // sendGroupMessage truncates silently at MAX_TEXT_LEN (160) minus "<name>: ".
  const char* sender32 = "SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS";
  const char* name31 = "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN";
  Config c = cfg(STYLE_MASK);
  c.reply_gap = 255;
  c.sender_gap = 3600;
  c.reply_jitter = 30;
  strcpy(c.name, name31);
  RawPath long_path;
  for (uint8_t i = 1; i <= 21; i++) long_path.add({0xaa, 0xbb, (uint8_t)i}, 3);
  RawPath direct;
  const char* bodies[] = {"test", "nbr", "h"};
  for (const char* body : bodies) {
    State s;
    s.setConfig(c);
    char text[96];
    snprintf(text, sizeof(text), "%s: %s", sender32, body);
    Verdict v = s.onMessage(T0, text, "Beebo", makeLink(strcmp(body, "test") == 0 ? long_path : direct, -50, -120, 0), 0);
    ASSERT_EQ(V_REPLY, v) << body;
    EXPECT_LE((int)strlen(s.pendingText()), maxBodyLen(name31)) << body;
    EXPECT_NE(nullptr, strstr(s.pendingText(), sender32)) << body;
  }
  EXPECT_EQ(160 - 31 - 2, maxBodyLen(name31));
}

TEST(BotReply, UsesBotNameForBudgetWhenSet) {
  EXPECT_EQ(160 - 5 - 2, maxBodyLen("Beebo"));
}

// ---- rate limiting and delay ----

TEST(BotGate, JitterSpreadsTheDelay) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p), "Beebo", 0);
  EXPECT_EQ(nullptr, f.s.due(T0 - 1));
  EXPECT_NE(nullptr, f.s.due(T0));

  Fix g;
  g.msg(T0, "a: test", makeLink(p), "Beebo", 65535);
  EXPECT_EQ(nullptr, g.s.due(T0 + 3000));
  EXPECT_NE(nullptr, g.s.due(T0 + 4000));
}

TEST(BotGate, ZeroJitterRepliesImmediately) {
  Fix f;
  f.c.reply_jitter = 0;
  f.s.setConfig(f.c);
  RawPath p;
  f.msg(T0, "a: test", makeLink(p), "Beebo", 65535);
  EXPECT_NE(nullptr, f.s.due(T0));
}

TEST(BotGate, DueReturnsTheTextOnceAndCountsTheReply) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  EXPECT_EQ(0u, f.s.replies());
  const char* t = f.s.due(T0 + 5000);
  ASSERT_NE(nullptr, t);
  EXPECT_STREQ("@[a] got it: direct. Use h for policy", t);
  EXPECT_EQ(1u, f.s.replies());
  EXPECT_EQ(nullptr, f.s.due(T0 + 6000));
  EXPECT_FALSE(f.s.pending());
}

TEST(BotGate, OnePendingReplyAtATime) {
  Fix f;
  RawPath p;
  ASSERT_EQ(V_REPLY, f.msg(T0, "a: test", makeLink(p)));
  EXPECT_EQ(V_DROP_PENDING, f.msg(T0 + 1, "b: test", makeLink(p)));
  EXPECT_EQ(1u, f.s.dropped());
}

TEST(BotGate, GlobalGapBetweenReplies) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  ASSERT_NE(nullptr, f.s.due(T0));
  EXPECT_EQ(V_DROP_GAP, f.msg(T0 + 9999, "b: test", makeLink(p)));
  EXPECT_EQ(V_REPLY, f.msg(T0 + 10000, "b: test", makeLink(p)));
}

TEST(BotGate, GapMeasuredFromSendNotFromRequest) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p), "Beebo", 65535);   // sent about T0 + 3999
  uint32_t sent = T0 + 3999;
  ASSERT_NE(nullptr, f.s.due(sent));
  EXPECT_EQ(V_DROP_GAP, f.msg(sent + 9999, "b: test", makeLink(p)));
  EXPECT_EQ(V_REPLY, f.msg(sent + 10000, "b: test", makeLink(p)));
}

TEST(BotGate, GapHonorsConfiguredValueAndSurvivesMillisWrap) {
  Fix f;
  f.c.reply_gap = 2;
  f.s.setConfig(f.c);
  RawPath p;
  uint32_t t = 0xFFFFFF00u;
  f.msg(t, "a: test", makeLink(p), "Beebo", 0);
  ASSERT_NE(nullptr, f.s.due(t));
  EXPECT_EQ(V_DROP_GAP, f.msg(t + 1999, "b: test", makeLink(p)));   // wraps past zero
  EXPECT_EQ(V_REPLY, f.msg(t + 2000, "b: test", makeLink(p)));
}

TEST(BotGate, SenderGapLimitsTheSameSender) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.s.due(T0);
  EXPECT_EQ(V_DROP_SENDER, f.msg(T0 + 20000, "a: test", makeLink(p)));
  EXPECT_EQ(V_REPLY, f.msg(T0 + 20000, "b: test", makeLink(p)));   // another sender is fine
  f.s.due(T0 + 20000);
  EXPECT_EQ(V_DROP_SENDER, f.msg(T0 + 59999, "a: test", makeLink(p)));
  EXPECT_EQ(V_REPLY, f.msg(T0 + 60000, "a: ping", makeLink(p)));   // ping counts as test
}

TEST(BotGate, ZeroSenderGapDisablesThePerSenderLimit) {
  Fix f;
  f.c.sender_gap = 0;
  f.s.setConfig(f.c);
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.s.due(T0);
  EXPECT_EQ(V_REPLY, f.msg(T0 + 10000, "a: test", makeLink(p)));
}

TEST(BotGate, HelpHasItsOwnSenderGap) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.s.due(T0);
  EXPECT_EQ(V_REPLY, f.msg(T0 + 10000, "a: h", makeLink(p)));   // asked for policy right after a test
  f.s.due(T0 + 10000);
  EXPECT_EQ(V_DROP_SENDER, f.msg(T0 + 20000, "a: help", makeLink(p)));   // but not twice
}

TEST(BotGate, NeighborHasItsOwnSenderGap) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.s.due(T0);
  EXPECT_EQ(V_REPLY, f.msg(T0 + 10000, "a: nbr", makeLink(p)));
}

TEST(BotGate, SenderTableEvictsTheOldestOfEight) {
  Fix f;
  f.c.reply_gap = 2;
  f.c.sender_gap = 3600;
  f.s.setConfig(f.c);
  RawPath p;
  uint32_t t = T0;
  char text[16];
  for (int i = 1; i <= 9; i++, t += 3000) {
    snprintf(text, sizeof(text), "s%d: test", i);
    ASSERT_EQ(V_REPLY, f.msg(t, text, makeLink(p))) << i;
    ASSERT_NE(nullptr, f.s.due(t));
  }
  // s1 was evicted by s9: it may be answered again; s2 is still remembered.
  EXPECT_EQ(V_DROP_SENDER, f.msg(t, "s2: test", makeLink(p)));
  EXPECT_EQ(V_REPLY, f.msg(t, "s1: test", makeLink(p)));
}

TEST(BotGate, OwnNameIsNeverAnswered) {
  Fix f;
  RawPath p;
  EXPECT_EQ(V_DROP_SELF, f.msg(T0, "Beebo: test", makeLink(p), "Beebo"));
  f.c.name[0] = 0;
  strcpy(f.c.name, "Beebo-Bot");
  f.s.setConfig(f.c);
  EXPECT_EQ(V_DROP_SELF, f.msg(T0, "Beebo-Bot: test", makeLink(p), "Beebo"));
  EXPECT_EQ(V_DROP_SELF, f.msg(T0, "Beebo: test", makeLink(p), "Beebo"));
  EXPECT_EQ(V_REPLY, f.msg(T0, "Beebo-bot: test", makeLink(p), "Beebo"));   // names are case-sensitive
}

TEST(BotGate, BotReplyBodiesAreIgnoredWithoutCounting) {
  Fix f;
  RawPath p;
  EXPECT_EQ(V_IGNORE, f.msg(T0, "bot: @[alice] got it: direct. Use h for policy", makeLink(p)));
  EXPECT_EQ(V_IGNORE, f.msg(T0, "bot: @[alice] test", makeLink(p)));
  EXPECT_EQ(0u, f.s.dropped());
}

TEST(BotGate, StyleOffIgnoresItsTriggers) {
  RawPath p;
  Fix a(STYLE_NEIGHBOR);
  EXPECT_EQ(V_IGNORE, a.msg(T0, "x: test", makeLink(p)));
  Fix b(STYLE_TEST);
  EXPECT_EQ(V_IGNORE, b.msg(T0, "x: nbr", makeLink(p)));
  Fix c(0);
  EXPECT_EQ(V_IGNORE, c.msg(T0, "x: h", makeLink(p)));
  EXPECT_EQ(V_IGNORE, c.msg(T0, "x: test", makeLink(p)));
  EXPECT_EQ(0u, c.s.dropped());
}

TEST(BotGate, NeighborTriggerFromMultiHopSenderIsDroppedAndCounted) {
  Fix f;
  RawPath p;
  p.add({0x3f}, 1);
  EXPECT_EQ(V_DROP_NOT_DIRECT, f.msg(T0, "x: nbr", makeLink(p)));
  EXPECT_EQ(1u, f.s.dropped());
  EXPECT_EQ(1u, f.s.droppedBy(V_DROP_NOT_DIRECT));
  EXPECT_FALSE(f.s.pending());
}

TEST(BotGate, NonTriggersAndUnparsableTextAreIgnored) {
  Fix f;
  RawPath p;
  EXPECT_EQ(V_IGNORE, f.msg(T0, "x: hello", makeLink(p)));
  EXPECT_EQ(V_IGNORE, f.msg(T0, "no prefix", makeLink(p)));
  EXPECT_EQ(0u, f.s.dropped());
}

TEST(BotGate, DroppedCountersSplitByReason) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.msg(T0, "b: test", makeLink(p));   // pending
  f.s.due(T0);
  f.msg(T0 + 1000, "c: test", makeLink(p));   // gap
  f.msg(T0 + 20000, "a: test", makeLink(p));  // sender
  f.msg(T0 + 20000, "Beebo: test", makeLink(p));   // self
  EXPECT_EQ(4u, f.s.dropped());
  EXPECT_EQ(1u, f.s.droppedBy(V_DROP_PENDING));
  EXPECT_EQ(1u, f.s.droppedBy(V_DROP_GAP));
  EXPECT_EQ(1u, f.s.droppedBy(V_DROP_SENDER));
  EXPECT_EQ(1u, f.s.droppedBy(V_DROP_SELF));
}

TEST(BotGate, CancelDropsThePendingReplyWithoutCountingIt) {
  Fix f;
  RawPath p;
  f.msg(T0, "a: test", makeLink(p));
  f.s.cancel();
  EXPECT_FALSE(f.s.pending());
  EXPECT_EQ(nullptr, f.s.due(T0 + 60000));
  EXPECT_EQ(0u, f.s.replies());
  EXPECT_EQ(V_REPLY, f.msg(T0 + 1, "b: test", makeLink(p)));   // gap not started: nothing was sent
}

// ---- reply hop cap ----

TEST(BotHopCap, IsIncomingHopsPlusOneCappedAt21) {
  RawPath direct;
  Fix a;
  a.msg(T0, "a: test", makeLink(direct));
  EXPECT_EQ(1, a.s.pendingHopCap());

  RawPath three;
  three.add({1}, 1).add({2}, 1).add({3}, 1);
  Fix b;
  b.msg(T0, "a: test", makeLink(three));
  EXPECT_EQ(4, b.s.pendingHopCap());

  RawPath many;
  for (uint8_t i = 1; i <= 21; i++) many.add({0xaa, 0xbb, i}, 3);
  Fix c;
  c.msg(T0, "a: test", makeLink(many));
  EXPECT_EQ(21, c.s.pendingHopCap());
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
