#pragma once

// Arduino-free helpers for the app feed channels (plans/ROLE_APP_FEED.md), unit-testable
// from fw/test/: reply splitting and the synthesized channel-message frame.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define FEED_PARTS_MAX                 4     // a long reply is cut after this many parts
#define FEED_TEXT_MAX                  160   // BaseChatMesh's MAX_TEXT_LEN, which a static repeater lacks
#define FEED_PART_CAP                  (FEED_TEXT_MAX + 1)
#define FEED_FRAME_CAP                 176   // MAX_FRAME_SIZE
#define FEED_RESP_CHANNEL_MSG_RECV_V3  17    // RESP_CODE_CHANNEL_MSG_RECV_V3
#define FEED_PREFIX_LEN                4     // "i/n "

enum FeedChannelId : uint8_t { FEED_STATUS = 0, FEED_CLI = 1, FEED_COUNT = 2 };

inline const char* feedDefaultName(uint8_t id) {
  return id == FEED_STATUS ? "beebo-status" : "beebo-cli";
}

// Splits `text` into at most `max_parts` parts of at most `max_text` characters each, written
// into `out` (rows of FEED_PART_CAP). Text that fits is one part with no prefix. Otherwise each
// part starts with "i/n " and is cut at the last newline, else the last space in the second half
// of its window, else at the window width. When text is left after the last part, that part
// ends in "...". Returns the number of parts (at least 1).
inline int feedSplit(const char* text, size_t max_text, char (*out)[FEED_PART_CAP], int max_parts) {
  size_t len = strlen(text);
  if (len <= max_text) {
    memcpy(out[0], text, len + 1);
    return 1;
  }
  const size_t payload = max_text - FEED_PREFIX_LEN;
  size_t starts[FEED_PARTS_MAX], takes[FEED_PARTS_MAX];
  int cnt = 0;
  size_t pos = 0;
  while (pos < len && cnt < max_parts && cnt < FEED_PARTS_MAX) {
    while (cnt > 0 && pos < len && (text[pos] == ' ' || text[pos] == '\n')) pos++;
    if (pos >= len) break;
    size_t take = len - pos;
    if (take > payload) {
      take = payload;
      size_t cut = 0;
      for (size_t k = take; k > 0; k--) {          // last newline in the window
        if (text[pos + k - 1] == '\n') { cut = k - 1; break; }
      }
      if (cut == 0) {
        for (size_t k = take; k > take / 2; k--) { // else last space in its second half
          if (text[pos + k - 1] == ' ') { cut = k - 1; break; }
        }
      }
      if (cut > 0) take = cut;
    }
    starts[cnt] = pos;
    takes[cnt] = take;
    cnt++;
    pos += take;
  }
  while (pos < len && (text[pos] == ' ' || text[pos] == '\n')) pos++;
  const bool truncated = pos < len;
  for (int i = 0; i < cnt; i++) {
    snprintf(out[i], FEED_PART_CAP, "%d/%d ", i + 1, cnt);
    memcpy(&out[i][FEED_PREFIX_LEN], &text[starts[i]], takes[i]);
    out[i][FEED_PREFIX_LEN + takes[i]] = 0;
    if (truncated && i == cnt - 1) {
      size_t n = FEED_PREFIX_LEN + takes[i];
      memcpy(&out[i][n >= 3 + FEED_PREFIX_LEN ? n - 3 : FEED_PREFIX_LEN], "...", 4);
    }
  }
  return cnt;
}

// Builds a RESP_CODE_CHANNEL_MSG_RECV_V3 frame, the one layout every channel message to the app
// uses (received off air or node-originated). `snr4` is the SNR in quarter dB, `path_len` 0xFF
// for direct. The text is clamped to `cap`. Returns the frame length.
inline size_t channelMsgV3Frame(uint8_t* out, size_t cap, uint8_t channel_idx, uint8_t path_len,
                                int8_t snr4, int16_t rssi, uint32_t timestamp, const char* text) {
  size_t i = 0;
  out[i++] = FEED_RESP_CHANNEL_MSG_RECV_V3;
  out[i++] = (uint8_t)snr4;
  memcpy(&out[i], &rssi, 2);
  i += 2;
  out[i++] = channel_idx;
  out[i++] = path_len;
  out[i++] = 0;                  // TXT_TYPE_PLAIN
  memcpy(&out[i], &timestamp, 4);
  i += 4;
  size_t tlen = strlen(text);
  if (i + tlen > cap) tlen = cap - i;
  memcpy(&out[i], text, tlen);
  return i + tlen;
}


// Channel index a feed uses. Keeps `cur` while it is in range, not the other feed's slot and not
// a slot holding a real channel (bit i of `occupied`); otherwise the highest free slot, so the
// user's own channels keep their low indexes. 0xFF when nothing is free. `slots` is at most 64.
inline uint8_t feedAssignSlot(uint8_t cur, uint8_t other, uint64_t occupied, uint8_t slots) {
  if (cur < slots && cur != other && !((occupied >> cur) & 1)) return cur;
  for (int i = slots - 1; i >= 0; i--) {
    if (i != other && !((occupied >> i) & 1)) return (uint8_t)i;
  }
  return 0xFF;
}

// What a queued app frame is, for choosing what to drop when the offline queue is full.
enum QueueKind : uint8_t { QK_OTHER = 0, QK_CHANNEL = 1, QK_FEED = 2 };

// Index of the queued frame to evict so a new one fits, or -1 to drop the new one. Feed messages
// have priority: a new feed message evicts the oldest non-feed message of any kind, else the
// oldest feed message; any other new message evicts only the oldest non-feed channel message and
// never a feed message.
inline int offlineQueueVictim(const uint8_t* kind, int n, bool incoming_feed) {
  for (int i = 0; i < n; i++) {
    if (incoming_feed ? kind[i] != QK_FEED : kind[i] == QK_CHANNEL) return i;
  }
  return incoming_feed && n > 0 ? 0 : -1;
}
