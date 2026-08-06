#pragma once

#include <Mesh.h>
#include <helpers/MonRing.h>

#ifdef ESP32
  #include <FS.h>
#endif

#define MAX_PACKET_HASHES  (128+32)

// beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- "TX reception confirmation".
// A small ring of (hash, millis-timestamp) pairs for packets THIS node has
// itself transmitted (self-originated or forwarded -- see markSelfTx(),
// called at the same 5 "packet as already sent" sites in Mesh.cpp that
// already seed the main dedup table below, for exactly the same reason:
// anticipating our own retransmission echoing back). Deliberately separate
// from _hashes -- that table has no timestamp or provenance (was it a real
// RX, or our own earlier TX?), which is exactly the gap that made a
// duplicate detection unable to confirm "this echoes a specific TX of
// ours" instead of just "we've seen this hash before" (see hasSeen()).
// Sizing (ring depth) is a first pick, not yet empirically tuned against
// real flood-convergence timescales -- an open question flagged in the plan
// doc. The echo window itself is NOT a flat constant (a 60s, then 5s guess
// were both tried and both wrong) -- it's computed per self-tx, from that
// packet's own estimated airtime, the same physical quantity
// BaseChatMesh::calcDirectTimeoutMillisFor() uses for the ack side. The two
// cases are the same shape of round trip: our TX -> neighbor receives ->
// neighbor's CSMA backoff -> neighbor transmits (rebroadcast or ack) -> we
// receive it. ECHO_TIMEOUT_BASE_MILLIS/ECHO_PERHOP_FACTOR/
// ECHO_PERHOP_EXTRA_MILLIS mirror Beebo.h's SEND_TIMEOUT_BASE_MILLIS/
// DIRECT_SEND_PERHOP_FACTOR/DIRECT_SEND_PERHOP_EXTRA_MILLIS values
// (single-hop case, i.e. path_hash_count+1 == 1) rather than reusing them
// directly -- this is core library code (src/helpers/), Beebo.h's
// constants live in the app layer (examples/multi_role/) and aren't
// visible here.
#define MAX_ECHO_HASHES           16
#define ECHO_TIMEOUT_BASE_MILLIS  500
#define ECHO_PERHOP_FACTOR        6.0f
#define ECHO_PERHOP_EXTRA_MILLIS  250

// beebo: DoS/QoS audit follow-up -- a plain "any occupied-slot eviction"
// counter fires on essentially every insert once the table has cycled once
// after boot, since the table is a plain 160-slot FIFO ring with no
// per-entry TTL: it can't tell "evicted a hash whose duplicate could still
// plausibly arrive" apart from "evicted a hash so old any real duplicate
// would have shown up long ago" -- the two are opposite in what they mean
// for QoS, so an unconditional counter says nothing useful. _dedup_evicted_
// count below only counts (and only logs, see hasSeen()) the former: gated
// by how old the evicted slot's own entry was (millis(), like
// _echo_time above -- RTC-second resolution was tried first and found
// far too coarse for real duplicate-detection timescales), against a
// configurable window (_dedup_window_ms, NodePrefs.dedup_window_ms)
// defaulting to 10s -- most duplicate arrivals in practice are fast,
// near-immediate multi-path echoes, so 10s already covers the common case
// without flagging every routine wrap as "live". Independent of the
// echo ring's per-packet echo timeout above (this gates
// _dedup_evicted_count stats on the general hash table, not self-tx
// resolution).
// DEDUP_WINDOW_MAX_MS is the separate upper bound `beebo settings
// */routing.dedup_window` can be configured up to -- deliberately not tied
// to the default, so a 10s default doesn't also cap how far a real
// deployment can widen the window if its traffic pattern warrants it.
#define DEDUP_LIVE_WINDOW_MS_DEFAULT 10000UL
#define DEDUP_WINDOW_MAX_MS 60000UL

class SimpleMeshTables : public mesh::MeshTables {
  uint8_t _hashes[MAX_PACKET_HASHES*MAX_HASH_SIZE];
  // beebo: millis() at each _hashes slot's own insertion (index-aligned
  // with _hashes/_next_idx) -- lets hasSeen() tell a live eviction apart
  // from a stale one, at millisecond resolution (RTC-second resolution was
  // tried first and found far too coarse for real duplicate-detection
  // timescales, which are sub-second). A slot only ever counts as "occupied"
  // once its actual hash bytes are non-zero (see hasSeen()'s slot_occupied
  // check), so a never-written slot's default-zero timestamp is never read
  // as an eviction age -- no explicit persistence needed for this array,
  // same as _hashes itself isn't across a plain reboot (only saveTo()'s
  // explicit snapshot survives one, and that's snapshot+restore, not a
  // rolling live value this needs to track).
  uint32_t _hash_insert_time[MAX_PACKET_HASHES];
  int _next_idx;
  uint32_t _direct_dups, _flood_dups;

  uint8_t _echo_hashes[MAX_ECHO_HASHES][MAX_HASH_SIZE];
  uint32_t _echo_time[MAX_ECHO_HASHES];
  // beebo: per-slot echo timeout (millis), computed from that packet's own
  // airtime at markSelfTx() time -- see the comment above
  // ECHO_TIMEOUT_BASE_MILLIS. Replaces a single flat window constant so
  // a short packet doesn't wait as long as a long one, and vice versa.
  uint32_t _echo_timeout[MAX_ECHO_HASHES];
  bool _echo_confirmed[MAX_ECHO_HASHES];  // already counted toward echo_success_count this generation?
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- true once this generation has
  // reached a final verdict, confirmed (_echo_confirmed) or timed out
  // unconfirmed (checkEchoTimeouts()). Mirrors the ack-table fix: a slot
  // that's still pending (neither) when the ring wraps back to reuse it is a
  // real starvation event (_echo_overflow_count), not a silent no-op.
  bool _echo_resolved[MAX_ECHO_HASHES];
  bool _echo_active[MAX_ECHO_HASHES];  // has this slot ever been assigned? (vs. pristine boot state)
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- Packet::calculateMonRingHash()
  // per slot (SHA256(payload)[0:4], NOT the calculatePacketHash() this ring
  // matches dups against) -- the correlation key EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT needs
  // to reference the origin MON_TX record (and, for a confirmed echo, the
  // MON_RX record of the specific packet that confirmed it too).
  uint32_t _echo_monring_hash[MAX_ECHO_HASHES];
  int _echo_next_idx;
  MonRing* _monring = nullptr;
  mesh::RTCClock* _rtc = nullptr;
  uint32_t _echo_success_count;
  uint32_t _echo_timeout_count;  // confirmed FAILED to be heard within the echo window (see checkEchoTimeouts())
  uint32_t _echo_overflow_count;  // ring wrapped onto a still-unresolved slot before it reached a verdict
  // Only flood-type self-transmissions are ring-tracked (see markSelfTx()) --
  // a direct/addressed packet structurally can't come back to us as a flood
  // echo, so it was never meaningful to track here; it's fully accounted for
  // by ack_success_count/ack_timeout_count instead. self_tx_direct_count is
  // kept for visibility only, not part of TuneController::txConfirmReward().
  uint32_t _echo_attempt_count;
  uint32_t _self_tx_direct_count;
  // beebo: DoS/QoS audit follow-up -- lifetime count of hasSeen() evicting a
  // slot whose hash was still within _dedup_window_ms of its own insertion
  // (see comment above DEDUP_LIVE_WINDOW_MS_DEFAULT) -- a genuine risk a
  // still-plausible duplicate goes undetected. Deliberately NOT counting
  // every occupied-slot eviction regardless of age: once the table has
  // cycled once after boot, that fires on essentially every insert and
  // says nothing about QoS (one eviction per RX packet is just how a
  // fixed-size FIFO ring behaves, not a fault). Never decremented.
  uint32_t _dedup_evicted_count = 0;
  uint32_t _dedup_window_ms = DEDUP_LIVE_WINDOW_MS_DEFAULT;

  // Slot index of `hash` among our own recent transmissions, still within
  // the echo window, or -1 -- i.e. this duplicate is a confirmed echo of a
  // specific TX we made, not just an anonymous repeat.
  int _findRecentEchoSlot(const uint8_t* hash) const {
    uint32_t now = millis();
    for (int i = 0; i < MAX_ECHO_HASHES; i++) {
      if (memcmp(hash, _echo_hashes[i], MAX_HASH_SIZE) == 0
          && (now - _echo_time[i]) <= _echo_timeout[i]) {  // unsigned sub -- millis() rollover-safe
        return i;
      }
    }
    return -1;
  }

  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- see MonRing.h's
  // EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT comment for the data[] layout.
  // SUCCESS/TIMEOUT only -- OVERFLOW goes through _emitEchoOverflowEvent()
  // instead (see MonRing.h's EVENT_ECHO_OVERFLOW comment for why it's a
  // separate event type). No-op if setMonRing() was never called (e.g.
  // native unit tests construct this class directly).
  void _emitEchoEvent(uint8_t verdict, uint32_t pkt_hash, uint32_t age_ms) {
    if (_monring == nullptr || _rtc == nullptr) return;
    EventRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.event_type = (verdict == TXCONFIRM_SUCCESS) ? EVENT_ECHO_SUCCESS : EVENT_ECHO_TIMEOUT;
    memcpy(&rec.data[1], &pkt_hash, 4);
    memcpy(&rec.data[5], &age_ms, 4);
    _monring->appendEvent(rec, (uint32_t)_rtc->getCurrentTime());
  }

  // beebo: DoS/QoS audit -- resource-exhaustion fault, see MonRing.h's
  // EVENT_ECHO_OVERFLOW comment. Same pkt_hash/age_ms shape as
  // _emitEchoEvent() but no verdict byte (the event type itself says what
  // happened).
  void _emitEchoOverflowEvent(uint32_t pkt_hash, uint32_t age_ms) {
    if (_monring == nullptr || _rtc == nullptr) return;
    EventRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.event_type = EVENT_ECHO_OVERFLOW;
    memcpy(&rec.data[1], &pkt_hash, 4);
    memcpy(&rec.data[5], &age_ms, 4);
    _monring->appendEvent(rec, (uint32_t)_rtc->getCurrentTime());
  }

public:
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- wires this table to the
  // monitoring ring so hasSeen()/checkEchoTimeouts()/markSelfTx() can log
  // EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT records. Called once from Beebo::begin() after
  // monring is allocated; the two-argument form (vs. constructor injection)
  // is needed because this table is a global static constructed in main.cpp
  // before monring exists. Safe to leave unset (native unit tests construct
  // this class directly with no MonRing at all) -- every emit site null-checks.
  void setMonRing(MonRing* monring, mesh::RTCClock* rtc) {
    _monring = monring;
    _rtc = rtc;
  }
  SimpleMeshTables() {
    memset(_hashes, 0, sizeof(_hashes));
    memset(_hash_insert_time, 0, sizeof(_hash_insert_time));
    _next_idx = 0;
    _direct_dups = _flood_dups = 0;
    memset(_echo_hashes, 0, sizeof(_echo_hashes));
    memset(_echo_time, 0, sizeof(_echo_time));
    memset(_echo_timeout, 0, sizeof(_echo_timeout));
    memset(_echo_confirmed, 0, sizeof(_echo_confirmed));
    memset(_echo_resolved, 0, sizeof(_echo_resolved));
    memset(_echo_active, 0, sizeof(_echo_active));
    memset(_echo_monring_hash, 0, sizeof(_echo_monring_hash));
    _echo_next_idx = 0;
    _echo_success_count = 0;
    _echo_timeout_count = 0;
    _echo_overflow_count = 0;
    _echo_attempt_count = 0;
    _self_tx_direct_count = 0;
  }

  // beebo: called alongside hasSeen() at Mesh.cpp's own 5 "packet as
  // already sent" call sites -- records that WE just transmitted this
  // hash (self-originated or forwarded), so a later matching RX (already
  // detected as a dup by hasSeen() below) can be recognized as an echo
  // confirming this specific TX succeeded. Direct/addressed packets are
  // counted (getSelfTxDirectCount()) but not ring-tracked: they can't come
  // back to us as a flood echo, so there's nothing for echo_success_count to
  // ever confirm about them -- ack_success_count/ack_timeout_count is
  // their whole story.
  void markSelfTx(const mesh::Packet* packet, uint32_t pkt_airtime_millis) override {
    if (packet->isRouteDirect()) {
      _self_tx_direct_count++;
      return;
    }
    int idx = _echo_next_idx;
    // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- if the slot we're about to
    // reuse was assigned but never reached a verdict (not confirmed heard,
    // not yet timed out by checkEchoTimeouts()), the ring wrapped faster
    // than that generation's own echo window -- a real starvation event,
    // not a silent no-op. Only possible if MAX_ECHO_HASHES self-tx
    // events happen inside one generation's own echo timeout.
    if (_echo_active[idx] && !_echo_resolved[idx]) {
      _echo_overflow_count++;
      _emitEchoOverflowEvent(_echo_monring_hash[idx],
                             millis() - _echo_time[idx]);
    }
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    memcpy(_echo_hashes[idx], hash, MAX_HASH_SIZE);
    _echo_time[idx] = millis();
    // beebo: same round-trip shape as BaseChatMesh::calcDirectTimeoutMillisFor()
    // (one hop: our TX already happened, then neighbor's backoff + its own
    // TX of the rebroadcast) -- see the comment above
    // ECHO_TIMEOUT_BASE_MILLIS for why the constants are a local copy
    // rather than a shared call.
    _echo_timeout[idx] = ECHO_TIMEOUT_BASE_MILLIS +
        (uint32_t)(pkt_airtime_millis * ECHO_PERHOP_FACTOR + ECHO_PERHOP_EXTRA_MILLIS);
    _echo_monring_hash[idx] = packet->calculateMonRingHash();
    _echo_confirmed[idx] = false;  // fresh generation for this slot
    _echo_resolved[idx] = false;
    _echo_active[idx] = true;
    _echo_next_idx = (idx + 1) % MAX_ECHO_HASHES;
    _echo_attempt_count++;
  }

  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- sweeps every ring slot for a
  // generation that's aged past its own per-packet echo timeout
  // (_echo_timeout[i], set in markSelfTx()) with no echo ever confirmed.
  // Mirrors Beebo::checkAckTableTimeouts() for the ack table: without this,
  // an unheard self-transmission never gets an explicit "failed" tally, it
  // just silently stops matching at lookup time (_findRecentEchoSlot()'s
  // own window check) or gets capacity-evicted even sooner by ring
  // rotation. Call once per main loop() tick.
  void checkEchoTimeouts() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_ECHO_HASHES; i++) {
      if (_echo_active[i] && !_echo_resolved[i]
          && (now - _echo_time[i]) > _echo_timeout[i]) {  // unsigned sub -- millis() rollover-safe
        _echo_timeout_count++;
        _echo_resolved[i] = true;
        _emitEchoEvent(TXCONFIRM_TIMEOUT, _echo_monring_hash[i], now - _echo_time[i]);
      }
    }
  }

  uint32_t getEchoSuccessCount() const { return _echo_success_count; }
  uint32_t getEchoTimeoutCount() const { return _echo_timeout_count; }
  uint32_t getEchoOverflowCount() const { return _echo_overflow_count; }
  uint32_t getEchoAttemptCount() const { return _echo_attempt_count; }
  uint32_t getSelfTxDirectCount() const { return _self_tx_direct_count; }
  uint32_t getDedupEvictedCount() const { return _dedup_evicted_count; }
  uint32_t getDedupWindowMs() const { return _dedup_window_ms; }
  // beebo: 0 is a real, literal value here -- it disables live-eviction
  // counting entirely (hasSeen() below only counts/logs when the window is
  // nonzero), not a "reset to default" sentinel. Range validation
  // ([0, DEDUP_WINDOW_MAX_MS]) is the caller's job (Beebo.cpp's
  // SET_DEDUP_WINDOW/tlvSetRepeaterDedupWindow); this just stores whatever
  // it's given.
  void setDedupWindowMs(uint32_t ms) {
    _dedup_window_ms = ms;
  }

#ifdef ESP32
  void restoreFrom(File f) {
    f.read(_hashes, sizeof(_hashes));
    f.read((uint8_t *) &_next_idx, sizeof(_next_idx));
  }
  void saveTo(File f) {
    f.write(_hashes, sizeof(_hashes));
    f.write((const uint8_t *) &_next_idx, sizeof(_next_idx));
  }
#endif

  bool hasSeen(const mesh::Packet* packet) override {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);

    const uint8_t* sp = _hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) {
        if (packet->isRouteDirect()) {
          _direct_dups++;   // keep some stats
        } else {
          _flood_dups++;
        }
        // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- this dup's hash also
        // matches one of OUR OWN recent transmissions -> a neighbor
        // rebroadcasting it is confirmation that specific TX was heard, not
        // just an anonymous duplicate. Counted at most once per self-tx
        // "generation" (the confirmed flag, not just the hash match) --
        // several neighbors independently echoing the same original flood
        // would otherwise inflate echo_success_count past one per transmission,
        // making it an unbounded echo-multiplicity count instead of a
        // proper 0/1-per-attempt rate comparable to ack_success_count.
        int slot = _findRecentEchoSlot(hash);
        if (slot >= 0 && !_echo_confirmed[slot]) {
          _echo_confirmed[slot] = true;
          _echo_resolved[slot] = true;
          _echo_success_count++;
          _emitEchoEvent(TXCONFIRM_SUCCESS, _echo_monring_hash[slot],
                            millis() - _echo_time[slot]);
        }
        return true;
      }
    }

    // beebo: DoS/QoS audit -- check BEFORE overwriting whether the slot
    // we're about to reuse still holds a real hash (non-zero; empty/never-
    // used slots are all-zero from the constructor/clear()) -- that's the
    // actual "no free space, something is being lost" moment, not merely
    // completing a lap of the table. See EVENT_RX_DEDUP_TABLE_FULL, MonRing.h.
    uint8_t* dest = &_hashes[_next_idx*MAX_HASH_SIZE];
    bool slot_occupied = false;
    for (int b = 0; b < MAX_HASH_SIZE; b++) {
      if (dest[b] != 0) { slot_occupied = true; break; }
    }
    // beebo: DoS/QoS audit follow-up -- age check is millis(), not RTC
    // epoch seconds (unlike the MonRing event timestamp below) -- real
    // duplicate-detection gaps are sub-second, RTC-second resolution can't
    // tell them apart. Rollover-safe unsigned subtraction, same pattern
    // the echo ring's echo-age checks use elsewhere in this file. Only
    // counts/logs if the window is nonzero (0 is a real, literal "disabled"
    // setting -- see setDedupWindowMs()'s comment, not a default sentinel)
    // AND the evicted slot's own insertion is still within it -- an
    // eviction past that window is just the table doing its job (the entry
    // had already outlived any realistic duplicate/retransmission delay),
    // not a fault.
    uint32_t now_ms = millis();
    if (slot_occupied && _dedup_window_ms > 0
        && (now_ms - _hash_insert_time[_next_idx]) <= _dedup_window_ms) {
      _dedup_evicted_count++;
      if (_monring != nullptr && _rtc != nullptr) {
        EventRecord rec;
        memset(&rec, 0, sizeof(rec));
        rec.event_type = EVENT_RX_DEDUP_TABLE_FULL;
        memcpy(&rec.data[0], &_dedup_evicted_count, 4);
        _monring->appendEvent(rec, (uint32_t)_rtc->getCurrentTime());
      }
    }
    memcpy(dest, hash, MAX_HASH_SIZE);
    _hash_insert_time[_next_idx] = now_ms;
    _next_idx = (_next_idx + 1) % MAX_PACKET_HASHES;  // cyclic table
    return false;
  }

  void clear(const mesh::Packet* packet) override {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);

    uint8_t* sp = _hashes;
    for (int i = 0; i < MAX_PACKET_HASHES; i++, sp += MAX_HASH_SIZE) {
      if (memcmp(hash, sp, MAX_HASH_SIZE) == 0) { 
        memset(sp, 0, MAX_HASH_SIZE);
        break;
      }
    }
  }

  uint32_t getNumDirectDups() const { return _direct_dups; }
  uint32_t getNumFloodDups() const { return _flood_dups; }

  void resetStats() { _direct_dups = _flood_dups = 0; }
};
