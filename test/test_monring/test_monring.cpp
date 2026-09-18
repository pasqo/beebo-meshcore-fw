#include <gtest/gtest.h>
#include <vector>
#include <helpers/MonRing.h>

namespace {

// A handful of small rings, each with its own backing buffer, sized in whole
// records so eviction math is easy to reason about in tests. init() seeds
// start_sync/start_radio/start_env from a zeroed radio/env at `now` (default
// 1000, matching the capture times most tests use), without consuming a ring
// slot -- so unless a test forces a relatch or an actual radio/env change,
// the ring starts out genuinely empty. `_base` comes out exactly equal to
// `now` at seed time (no bucket to floor to -- see _ensureSync()'s own
// comment for why a relatch only ever fires on offset overflow now).
template <uint32_t N>
struct RingFixture {
  MonRecord buf[N];
  MonRing ring;
  uint32_t seed;
  RingFixture(uint32_t now = 1000) : seed(now) {
    ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), now, now, RadioRecord{}, EnvRecord{});
    // MON_CAP_ENV is opt-in in production (excluded from the default config),
    // but most of these tests exercise sampleEnv() directly, so enable every
    // kind here; tests that care about masking set their own config.
    ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
  }

  // beebo: append*/noteRadio/sampleEnv() now take an already-resolved epoch-
  // ms instant directly (RTCClock::nowMillis() in production), not a raw
  // millis() reading MonRing itself used to re-anchor internally. Almost
  // every test here was written against the OLD contract, passing small
  // millis()-like literals meant to be read relative to this fixture's own
  // construction seed -- ms() reproduces that exact same arithmetic
  // (matching the old nowEpochMs() formula) so those literals keep meaning
  // exactly what they did before, with no per-call-site rescaling needed.
  // Only tests that used to re-anchor mid-test via setTimeAnchor()/clear()
  // to force an overflow need their own direct absolute literal instead --
  // ms() only knows this fixture's ORIGINAL seed.
  uint64_t ms(uint32_t millis_like) const {
    return (uint64_t)seed * 1000 + (uint64_t)(millis_like - seed);
  }
};

RxRecord makeRx(uint32_t hash = 0x11223344) {
  RxRecord rx; memset(&rx, 0, sizeof(rx));
  rx.pkt_hash = hash;
  rx.snr = 40; rx.rssi = -80;
  return rx;
}

TxRecord makeTx(uint32_t hash = 0xAABBCCDD) {
  TxRecord tx; memset(&tx, 0, sizeof(tx));
  tx.pkt_hash = hash;
  tx.result = TXR_OK;
  return tx;
}

EnvRecord makeEnv(int8_t noise_floor = -90) {
  EnvRecord env; memset(&env, 0, sizeof(env));
  env.noise_floor = noise_floor;
  return env;
}

TuneRecord makeTune(uint8_t param_id = TUNE_RX_DELAY_BASE, int16_t old_value = 4,
                     int16_t proposed_value = 6) {
  TuneRecord tune; memset(&tune, 0, sizeof(tune));
  tune.param_id = param_id;
  tune.old_value = old_value;
  tune.proposed_value = proposed_value;
  return tune;
}

// beebo: EVENT_TX_POOL_FULL/EVENT_TX_CAD_TIMEOUT/EVENT_RX_START_TIMEOUT (split
// out of the old single EVENT_FAULT+bitmask, see MonRing.h) all share the
// same cumulative-count shape as EVENT_RX_POOL_FULL etc.
EventRecord makeFaultEvent(uint8_t event_type, uint32_t cumulative) {
  EventRecord event; memset(&event, 0, sizeof(event));
  event.event_type = event_type;
  memcpy(&event.data[0], &cumulative, 4);
  return event;
}

SettingRecord makeSettingRecord(uint8_t key, uint32_t old_raw, uint32_t new_raw, uint8_t source) {
  SettingRecord rec; memset(&rec, 0, sizeof(rec));
  rec.setting = key;
  rec.old_value = old_raw;
  rec.new_value = new_raw;
  rec.source = source;
  return rec;
}

CommandRecord makeCommandRecord(uint16_t command_id) {
  CommandRecord rec; memset(&rec, 0, sizeof(rec));
  rec.source = EVENT_SOURCE_BINARY;
  memcpy(&rec.command[0], &command_id, 2);
  return rec;
}

// beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 follow-up -- EVENT_ACK_SUCCESS/
// EVENT_ACK_TIMEOUT/EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT share this same
// pkt_hash/age_ms layout with EVENT_ACK_OVERFLOW/EVENT_ECHO_OVERFLOW (no
// verdict byte -- the event type itself says what happened, see
// MonRing.h's own comment).
EventRecord makeOverflowEvent(uint8_t event_type, uint32_t pkt_hash, uint32_t age_ms) {
  EventRecord event; memset(&event, 0, sizeof(event));
  event.event_type = event_type;
  memcpy(&event.data[1], &pkt_hash, 4);
  memcpy(&event.data[5], &age_ms, 4);
  return event;
}

RadioRecord makeRadio(uint32_t freq = 915000000, uint8_t sf = 7, uint16_t bw = 2500,
                       uint8_t cr = 5, int8_t tx_power = 20, uint8_t flags = 0) {
  RadioRecord radio; memset(&radio, 0, sizeof(radio));
  radio.freq = freq; radio.sf = sf; radio.bw = bw; radio.cr = cr;
  radio.tx_power = tx_power; radio.flags = flags;
  return radio;
}

}  // namespace

TEST(MonRingGlobalTimeAnchor, InitSetsSharedGlobalNotJustPrivateState) {
  // beebo: g_time_anchor_epoch_sec/g_time_anchor_millis (MonRing.h) are
  // shared, ring-agnostic globals now -- not MonRing's own private state --
  // so DebugLog's DLOG live push (DebugLog.h's writeHeader()) can read
  // the exact same anchor directly, with no dependency on MonRing's wire
  // protocol (MLOG) at all. init()/setTimeAnchor() must write through to
  // these globals, not some now-removed private copy.
  MonRing ring;
  uint8_t buf[1024];
  ring.init(buf, sizeof(buf), 5000, 6000, RadioRecord{}, EnvRecord{});
  EXPECT_EQ(5000u, g_time_anchor_epoch_sec);
  EXPECT_EQ(6000u, g_time_anchor_millis);
  EXPECT_TRUE(globalTimeAnchorValid());
  EXPECT_TRUE(ring.timeAnchorValid());

  ring.setTimeAnchor(7777, 8888);
  EXPECT_EQ(7777u, g_time_anchor_epoch_sec);
  EXPECT_EQ(8888u, g_time_anchor_millis);
}

TEST(MonRing, InitRejectsNullOrUndersizedBuffer) {
  MonRing ring;
  EXPECT_FALSE(ring.init(nullptr, 1024, 1000, 1000, RadioRecord{}, EnvRecord{}));
  uint8_t tiny[4];
  EXPECT_FALSE(ring.init(tiny, sizeof(tiny), 1000, 1000, RadioRecord{}, EnvRecord{}));
  EXPECT_FALSE(ring.allocated());
}

TEST(MonRing, AppendRxAssignsSequentialSeq) {
  RingFixture<8> f;  // seeded at now=1000
  uint32_t seq0 = f.ring.appendRx(makeRx(), f.ms(1000));
  uint32_t seq1 = f.ring.appendRx(makeRx(), f.ms(1001));
  // init() already latched the base, so appends don't auto-store a SYNC
  // record -- that only happens on a real relatch (see
  // EnsureSyncRelatchesAfterPeriodElapses).
  EXPECT_EQ(0u, seq0);
  EXPECT_EQ(1u, seq1);
  EXPECT_EQ(2u, f.ring.nextSeq());
  EXPECT_EQ(0u, f.ring.oldestSeq());
  EXPECT_EQ(2u, f.ring.count());
}

TEST(MonRing, AppendNoOpWhenDisabledOrKindMasked) {
  RingFixture<8> f;
  f.ring.setConfig(MON_CAP_ALL);  // clear MON_CAP_ENABLED bit
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), f.ms(1000)));
  EXPECT_EQ(0u, f.ring.count());

  f.ring.setConfig(MON_CAP_ENABLED | MON_CAP_TX);  // RX masked out
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), f.ms(1000)));
  EXPECT_EQ(0u, f.ring.count());
}

TEST(MonRing, SerializeForwardWalkFiltersByAfterSeq) {
  RingFixture<8> f;
  f.ring.appendRx(makeRx(1), f.ms(1000));  // seq0 = RX
  f.ring.appendRx(makeRx(2), f.ms(1001));  // seq1 = RX
  f.ring.appendRx(makeRx(3), f.ms(1002));  // seq2 = RX

  MonRecord out[8];
  uint32_t returned = 0;
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  EXPECT_EQ(3u, returned);
  EXPECT_EQ((int)(3 * sizeof(MonRecord)), bytes);
  EXPECT_EQ(MON_RX, out[0].kind);
  EXPECT_EQ(1u, out[0].rx.pkt_hash);

  // after_seq=1 should skip the first rx record.
  returned = 0;
  bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 1, &returned);
  EXPECT_EQ(2u, returned);
  EXPECT_EQ(MON_RX, out[0].kind);
  EXPECT_EQ(2u, out[0].rx.pkt_hash);
  EXPECT_EQ(3u, out[1].rx.pkt_hash);
}

TEST(MonRing, SerializeClampsToMaxLenInWholeRecords) {
  RingFixture<8> f;
  for (int i = 0; i < 5; i++) f.ring.appendRx(makeRx(i), f.ms(1000 + i));

  MonRecord out[8];
  uint32_t returned = 0;
  // Room for exactly 2 records (plus a few leftover bytes that must not
  // produce a partial record).
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(MonRecord) * 2 + 4, 0, &returned);
  EXPECT_EQ(2u, returned);
  EXPECT_EQ((int)(2 * sizeof(MonRecord)), bytes);
}

TEST(MonRing, SerializeAfterSeqPastHeadReturnsNothing) {
  RingFixture<8> f;
  f.ring.appendRx(makeRx(), f.ms(1000));
  MonRecord out[8];
  uint32_t returned = 0;
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), f.ring.nextSeq(), &returned);
  EXPECT_EQ(0, bytes);
  EXPECT_EQ(0u, returned);
}

TEST(MonRing, EvictionReanchorsStartSyncAndOldestSeqAdvances) {
  // Capacity 4. init() at now=1000 seeds start_sync but stores no record.
  // Forcing an overflow (a now_ms far past the seed) makes the very next
  // capture relatch (store a real SYNC record), which then fills the ring
  // and gets evicted.
  RingFixture<4> f;
  f.ring.appendRx(makeRx(1), 100001002ULL);  // forced overflow: seq0=SYNC(100001) seq1=RX
  f.ring.appendRx(makeRx(2), 100001002ULL);  // same base: seq2=RX
  f.ring.appendRx(makeRx(3), 100001002ULL);  // seq3=RX  (ring now full: 0,1,2,3)
  EXPECT_EQ(0u, f.ring.oldestSeq());

  f.ring.appendRx(makeRx(4), 100001002ULL);  // seq4=RX, evicts seq0 (SYNC)
  EXPECT_EQ(1u, f.ring.oldestSeq());
  EXPECT_EQ(5u, f.ring.nextSeq());
  EXPECT_EQ(4u, f.ring.count());
  EXPECT_EQ(100001u, f.ring.startTime());  // reanchored from the evicted SYNC
}

TEST(MonRing, EmitStartRefSyncAlwaysValidFromBootSeedBeforeAnyEviction) {
  // start_sync is always populated once init() has run -- even before
  // anything has actually changed or been evicted -- so emitStartRef()
  // must report a value for it immediately. MON_RADIO/MON_ENV are both
  // withheld until a real noteRadio()/sampleEnv() call has actually
  // happened -- see EmitStartRefRadioWithheldUntilFirstRealSample/
  // EmitStartRefEnvWithheldUntilFirstRealSample below: init()'s boot-time
  // radio/env snapshots aren't genuine samples (startMonRing()'s
  // placeholder RadioRecord{} is all-zero; radio noise-floor calibration/
  // MCU temp aren't settled that early either), so neither must be handed
  // out as if it were one. Confirmed as a real bug on hardware
  // (2026-09-11): before MON_RADIO got this same treatment, a fresh MLOG
  // replay-on-enable kept injecting the all-zero boot placeholder as
  // "the current radio config" on every reconnect, long after a real
  // MON_RADIO record already existed further into the ring.
  RingFixture<8> f;
  MonRecord dest;
  ASSERT_TRUE(f.ring.emitStartRef(MON_SYNC, &dest));
  EXPECT_EQ(MON_SYNC, dest.kind);
  EXPECT_EQ(1000u, dest.sync.timestamp);  // the init() boot seed
  EXPECT_EQ(MONRING_ABI_VERSION, dest.sync.abi_version);
  EXPECT_FALSE(f.ring.emitStartRef(MON_RADIO, &dest));
  EXPECT_FALSE(f.ring.emitStartRef(MON_ENV, &dest));
}

TEST(MonRing, EmitStartRefRadioWithheldUntilFirstRealSample) {
  // Mirrors EmitStartRefEnvWithheldUntilFirstRealSample exactly -- before
  // any real noteRadio() call, MON_RADIO must not be injected at all
  // (init()'s boot-time seed is startMonRing()'s all-zero placeholder,
  // not a real config). Once a real config lands, it must be handed back
  // immediately -- not just after some later eviction reanchors it.
  RingFixture<8> f;
  MonRecord dest;
  EXPECT_FALSE(f.ring.emitStartRef(MON_RADIO, &dest));

  f.ring.noteRadio(makeRadio(915000000), f.ms(1000));

  ASSERT_TRUE(f.ring.emitStartRef(MON_RADIO, &dest));
  EXPECT_EQ(MON_RADIO, dest.kind);
  EXPECT_EQ(915000000u, dest.radio.freq);
}

TEST(MonRing, EmitStartRefEnvWithheldUntilFirstRealSample) {
  // Before any real sampleEnv() call, MON_ENV must not be injected at all
  // -- init()'s boot-time seed is not a genuine reading (see
  // plans/CPU_UTILIZATION.md's "Fixed-cadence sampling" section: noise
  // floor calibrates in the background, MCU temp isn't settled that
  // early either). Once a real sample lands, it must be handed back
  // immediately -- not just after some later eviction reanchors it.
  RingFixture<8> f;
  MonRecord dest;
  EXPECT_FALSE(f.ring.emitStartRef(MON_ENV, &dest));

  f.ring.sampleEnv(makeEnv(-77), f.ms(1000));

  ASSERT_TRUE(f.ring.emitStartRef(MON_ENV, &dest));
  EXPECT_EQ(MON_ENV, dest.kind);
  EXPECT_EQ(-77, dest.env.noise_floor);
}

TEST(MonRing, ClearReseedsRadioAsAlreadyValid) {
  // Mirrors ClearReseedsEnvAsAlreadyValid -- clear()'s radio snapshot is a
  // live read taken at clear time, already a genuine sample, so
  // emitStartRef() must return it right away even with no prior noteRadio()
  // call in this ring's lifetime.
  RingFixture<8> f;
  f.ring.clear(2000, 2000, makeRadio(915000000), makeEnv(-60));

  MonRecord dest;
  ASSERT_TRUE(f.ring.emitStartRef(MON_RADIO, &dest));
  EXPECT_EQ(915000000u, dest.radio.freq);
}

TEST(MonRing, ClearReseedsEnvAsAlreadyValid) {
  // Unlike init()'s boot-time seed, clear()'s env snapshot is a live read
  // taken at clear time -- already a genuine sample -- so emitStartRef()
  // must return it right away, not withhold it pending a fresh
  // sampleEnv() call.
  RingFixture<8> f;
  f.ring.sampleEnv(makeEnv(-80), f.ms(1000));  // establish a real sample pre-clear
  f.ring.clear(2000, 2000, makeRadio(), makeEnv(-60));

  MonRecord dest;
  ASSERT_TRUE(f.ring.emitStartRef(MON_ENV, &dest));
  EXPECT_EQ(-60, dest.env.noise_floor);
}

TEST(MonRing, EmitStartRefAllThreeKindsAfterFullWrap) {
  RingFixture<3> f;
  f.ring.noteRadio(makeRadio(), f.ms(1000));  // seq0=RADIO (base already latched by init())
  f.ring.sampleEnv(makeEnv(), f.ms(1000));    // seq1=ENV
  f.ring.appendTx(makeTx(), f.ms(1001));      // seq2=TX (ring full: RADIO,ENV,TX)
  f.ring.appendTx(makeTx(), f.ms(1002));      // evicts RADIO(0)
  f.ring.appendTx(makeTx(), f.ms(1003));      // evicts ENV(1)

  MonRecord dest;
  ASSERT_TRUE(f.ring.emitStartRef(MON_SYNC, &dest));   // never evicted -- boot seed persists
  EXPECT_EQ(MON_SYNC, dest.kind);
  ASSERT_TRUE(f.ring.emitStartRef(MON_RADIO, &dest));  // reanchored from the evicted real record
  EXPECT_EQ(MON_RADIO, dest.kind);
  ASSERT_TRUE(f.ring.emitStartRef(MON_ENV, &dest));    // reanchored from the evicted real record
  EXPECT_EQ(MON_ENV, dest.kind);

  // The real record now occupying the oldest slot is a TX, not a reference
  // kind -- confirming the ring genuinely has nothing left to peek for these.
  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_TX, peeked.kind);
}

namespace {

// Mirrors the exact per-slot loop in MyMesh.cpp's fillMonRingFrame(): walk
// the fixed sync/radio/env prefix, one slot at a time, against the real
// record stream starting at `from`. A slot matched against the peeked real
// record advances the peek cursor and is left for serialize() to emit as-is;
// an unmatched slot is filled from the stashed start-ref instead, without
// advancing the cursor, so the same real record can still satisfy a later
// slot. Returns the ordered list of injected kinds (empty if the real
// stream's leading 3 records already are sync/radio/env in order).
std::vector<uint8_t> injectionPlan(const MonRing &ring, uint32_t from) {
  std::vector<uint8_t> injected;
  uint32_t peek_pos = from;
  static const uint8_t kSlotKind[3] = { MON_SYNC, MON_RADIO, MON_ENV };
  for (int s = 0; s < 3; s++) {
    MonRecord rec;
    if (ring.peek(peek_pos, &rec) && rec.kind == kSlotKind[s]) {
      peek_pos++;
    } else {
      MonRecord dest;
      if (ring.emitStartRef(kSlotKind[s], &dest)) injected.push_back(kSlotKind[s]);
    }
  }
  return injected;
}

}  // namespace

TEST(MonRing, InjectionPlanEmptyWhenNotWrapped) {
  // Non-wrapping ring: the real sync/radio/env are all resident in order (a
  // relatch forces a real SYNC ahead of the radio/env that depend on it), so
  // nothing needs to be synthesized.
  RingFixture<8> f;
  f.ring.noteRadio(makeRadio(), 100001002ULL);  // forced overflow: seq0=SYNC seq1=RADIO
  f.ring.sampleEnv(makeEnv(), 100001002ULL);    // seq2=ENV
  f.ring.appendRx(makeRx(), 100001002ULL);      // seq3=RX

  EXPECT_TRUE(injectionPlan(f.ring, f.ring.oldestSeq()).empty());
}

TEST(MonRing, InjectionPlanEvictedSyncOnly) {
  // Ring wraps just enough to evict the real SYNC(0); RADIO(1) and ENV(2)
  // are still resident, so only sync needs to be synthesized ahead of them.
  // A now_ms far past the seed forces exactly one overflow-driven relatch
  // on the next append; every call after that stays close enough in time
  // that no further relatch fires on its own (a relatch depends purely on
  // offset overflow -- see _ensureSync()'s own comment -- not on a
  // separately adjustable bucket width).
  RingFixture<3> f;
  f.ring.noteRadio(makeRadio(), 100001002ULL);  // forced overflow: relatch (seq0=SYNC seq1=RADIO)
  f.ring.sampleEnv(makeEnv(), 100001002ULL);    // no relatch: seq2=ENV (ring full)
  f.ring.appendTx(makeTx(), 100001003ULL);      // no relatch: evicts SYNC(0)

  auto plan = injectionPlan(f.ring, f.ring.oldestSeq());
  ASSERT_EQ(1u, plan.size());
  EXPECT_EQ(MON_SYNC, plan[0]);

  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_RADIO, peeked.kind);
}

TEST(MonRing, InjectionPlanEvictedSyncAndRadio) {
  // Ring wraps further, evicting SYNC(0) and RADIO(1); only ENV(2) survives.
  RingFixture<4> f;
  f.ring.noteRadio(makeRadio(), 100000002ULL);  // forced overflow: relatch (seq0=SYNC seq1=RADIO)
  f.ring.sampleEnv(makeEnv(), 100000002ULL);    // seq2=ENV
  f.ring.appendTx(makeTx(), 100000002ULL);      // seq3=TX (ring full)
  f.ring.appendTx(makeTx(), 100000003ULL);      // evicts SYNC(0)
  f.ring.appendTx(makeTx(), 100000004ULL);      // evicts RADIO(1)

  auto plan = injectionPlan(f.ring, f.ring.oldestSeq());
  ASSERT_EQ(2u, plan.size());
  EXPECT_EQ(MON_SYNC, plan[0]);
  EXPECT_EQ(MON_RADIO, plan[1]);

  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_ENV, peeked.kind);
}

TEST(MonRing, InjectionPlanOldestRealRecordIsTx) {
  // Full wrap: SYNC/RADIO/ENV all evicted, oldest surviving real record is a
  // TX. All three slots must be synthesized ahead of it.
  RingFixture<3> f;
  f.ring.noteRadio(makeRadio(), 100000002ULL);  // forced overflow: relatch
  f.ring.sampleEnv(makeEnv(), 100000002ULL);
  f.ring.appendTx(makeTx(), 100000003ULL);
  f.ring.appendTx(makeTx(), 100000004ULL);
  f.ring.appendTx(makeTx(), 100000005ULL);

  auto plan = injectionPlan(f.ring, f.ring.oldestSeq());
  ASSERT_EQ(3u, plan.size());
  EXPECT_EQ(MON_SYNC, plan[0]);
  EXPECT_EQ(MON_RADIO, plan[1]);
  EXPECT_EQ(MON_ENV, plan[2]);

  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_TX, peeked.kind);
}

TEST(MonRing, InjectionPlanOldestRealRecordIsRx) {
  // Same full wrap, but the oldest surviving real record is an RX instead.
  RingFixture<3> f;
  f.ring.noteRadio(makeRadio(), 100000002ULL);  // forced overflow: relatch
  f.ring.sampleEnv(makeEnv(), 100000002ULL);
  f.ring.appendRx(makeRx(), 100000003ULL);
  f.ring.appendRx(makeRx(), 100000004ULL);
  f.ring.appendRx(makeRx(), 100000005ULL);

  auto plan = injectionPlan(f.ring, f.ring.oldestSeq());
  ASSERT_EQ(3u, plan.size());
  EXPECT_EQ(MON_SYNC, plan[0]);
  EXPECT_EQ(MON_RADIO, plan[1]);
  EXPECT_EQ(MON_ENV, plan[2]);

  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_RX, peeked.kind);
}

TEST(MonRing, InjectionPlanEnvReappearsBeforeRadioAfterWrap) {
  // RADIO changes far less often than ENV, so after enough wrapping the
  // resident record order can genuinely be [ENV, RADIO, RX, RX, ...]: the
  // original SYNC/RADIO have been evicted, but a later RADIO re-latch (from
  // a config change) is still resident right after the surviving ENV. Only
  // sync and radio need synthesizing -- the ENV slot must match the real
  // ENV already there, and the real RADIO right after it must not be
  // duplicated by a second injected copy.
  RingFixture<4> f;
  f.ring.noteRadio(makeRadio(915000000), 100000002ULL);  // forced overflow: relatch (seq0=SYNC seq1=RADIO(915))
  f.ring.sampleEnv(makeEnv(), 100000002ULL);              // seq2=ENV
  f.ring.noteRadio(makeRadio(868000000), 100000003ULL);   // seq3=RADIO(868) (ring full)
  f.ring.appendRx(makeRx(), 100000004ULL);  // evicts SYNC(0)
  f.ring.appendRx(makeRx(), 100000005ULL);  // evicts RADIO(1) -- original 915 config

  // Ring now holds ENV(2), RADIO(3, 868), RX(4), RX(5); oldest is ENV.
  MonRecord peeked;
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_ENV, peeked.kind);

  auto plan = injectionPlan(f.ring, f.ring.oldestSeq());
  ASSERT_EQ(2u, plan.size());
  EXPECT_EQ(MON_SYNC, plan[0]);
  EXPECT_EQ(MON_RADIO, plan[1]);  // slot2 (ENV) matched the real record, not injected

  // The real ENV, then the real (re-latched) RADIO, must still be next --
  // the peek cursor must not have skipped past either.
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq(), &peeked));
  EXPECT_EQ(MON_ENV, peeked.kind);
  ASSERT_TRUE(f.ring.peek(f.ring.oldestSeq() + 1, &peeked));
  EXPECT_EQ(MON_RADIO, peeked.kind);
  EXPECT_EQ(868000000u, peeked.radio.freq);
}

TEST(MonRing, EnsureSyncRelatchesOnceOffsetOverflows) {
  RingFixture<8> f(1000);  // seeded at now=1000, base=1000
  f.ring.appendRx(makeRx(), f.ms(1000));         // offset=0: no relatch (seq0=RX)
  f.ring.appendRx(makeRx(), f.ms(1000 + 65535));  // offset=65535: still in-window (seq1=RX)
  EXPECT_EQ(2u, f.ring.nextSeq());

  f.ring.appendRx(makeRx(), f.ms(1000 + 65536));  // offset=65536: overflows -- relatch (seq2=SYNC seq3=RX)
  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(4u, returned);
  EXPECT_EQ(MON_RX, out[0].kind);
  EXPECT_EQ(MON_RX, out[1].kind);
  EXPECT_EQ(MON_SYNC, out[2].kind);  // relatch inserted before the 3rd RX
  EXPECT_EQ(1065u, out[2].sync.timestamp);  // floor((1000000+65536)/1000), the exact current second
  EXPECT_EQ(MON_RX, out[3].kind);
}

// beebo: regression for a real bug -- Beebo::applyClockSync() runs an RTC
// correction on every connect and every ~5-10s keepalive tick while a
// session is open. Two earlier fix attempts tied relatching to real
// elapsed millis() since the last write, layered on top of an overflow
// check -- both still turned a MonRing ring almost entirely into MON_SYNC
// spam on real hardware (confirmed 2026-09-17: 1,100 of ~3,400 resident
// records on one device over 4 hours with nothing connected in between, 37
// of ~200 on another in 4 minutes, then still 15 in 7 minutes after the
// first fix attempt) -- one attempt via an unsigned-subtraction underflow
// being misread as an overflow, the other via an unthrottled
// anchor-landed-behind-base branch. The actual, much simpler fix: relatch
// is driven *purely* by whether the current offset would overflow its
// uint16_t wire budget -- nothing else, and (now that append*() takes an
// already-resolved epoch-ms instant directly, not a millis() reading
// MonRing re-anchors itself) there is no separate anchor state left for a
// clock correction to disturb independently of that value. Verifies many
// small forward steps, each individually harmless, don't force a relatch
// as long as the running offset stays inside its window, regardless of how
// many land.
TEST(MonRing, EnsureSyncToleratesManySmallCorrectionsWithoutOverflow) {
  RingFixture<16> f(0);  // anchor: epoch_sec=0, millis=0 -- 16 slots: 13 records inserted below
  f.ring.appendRx(makeRx(), f.ms(0));  // offset=0 -- no relatch (base already seeded here)
  EXPECT_EQ(1u, f.ring.nextSeq());

  // Ten separate small forward steps (500ms apart, as a real crystal-drift
  // correction might advance the clock) -- none should individually or
  // cumulatively force a relatch, since the running offset (500, 1000, ...,
  // 5000ms) never reaches anywhere near 65536 regardless of how many
  // separate steps land along the way.
  for (uint32_t i = 1; i <= 10; i++) {
    f.ring.appendRx(makeRx(), f.ms(i * 500));
  }
  EXPECT_EQ(11u, f.ring.nextSeq());  // 11 RX, still zero SYNC records

  // A step that finally pushes the offset to/past the overflow threshold
  // (base=0, so offset==the epoch_ms directly) -- exactly one relatch, not
  // one per step that came before it.
  f.ring.appendRx(makeRx(), f.ms(66000));
  MonRecord out[16];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(13u, returned);  // 11 RX + 1 SYNC + 1 RX
  EXPECT_EQ(MON_SYNC, out[11].kind);
  EXPECT_EQ(66u, out[11].sync.timestamp);  // the exact current second
  EXPECT_EQ(MON_RX, out[12].kind);
}

// beebo: regression for a real bug -- endTime() (GET_MONRING's "end" header
// field, shown by `beebo monitor` as the ring's newest-record time) used to
// be set directly to `now`, which was epoch seconds before
// plans/MONITORING_UNIFICATION.md Design #1's time-model change but is
// millis() after it -- appendXxx() was never updated, so endTime() started
// returning a raw millis() value (e.g. "end: 31978618") instead of a real
// epoch. Verifies it's derived through the same anchor math _ensureSync()
// uses, not the raw millis() parameter.
TEST(MonRing, EndTimeIsRealEpochNotRawMillis) {
  RingFixture<8> f(1000);  // anchor: epoch_sec=1000, millis=1000
  f.ring.appendRx(makeRx(), f.ms(5000));  // +4000ms elapsed -> epoch 1004, not raw 5000
  EXPECT_EQ(1004u, f.ring.endTime());
}

// beebo: noteRadio()/sampleEnv() are the two appendXxx()-family members that
// don't share the common "assign a fresh RxRecord/TxRecord/etc. and append"
// shape (they diff-and-store-on-change instead) -- confirmed by direct code
// read they were the only two NOT updating _end_time at all, so the ring's
// reported "end" time went stale (kept showing whatever RX/TX/etc. record
// last advanced it) whenever the actual newest resident record was a
// RADIO/ENV change instead.
TEST(MonRing, EndTimeAdvancesOnRadioAndEnvChangesToo) {
  RingFixture<8> f(1000);
  RadioRecord radio{}; radio.freq = 915000000;
  f.ring.noteRadio(radio, f.ms(5000));
  EXPECT_EQ(1004u, f.ring.endTime());

  EnvRecord env{}; env.noise_floor = -90;
  f.ring.sampleEnv(env, f.ms(9000));  // +8000ms elapsed -> epoch 1008
  EXPECT_EQ(1008u, f.ring.endTime());
}

// beebo: setTimeAnchor() must not touch ring contents/counts -- only the
// anchor used for the NEXT _ensureSync() computation. Separate from the
// rollover test above, which exercises setTimeAnchor() as a side effect of
// re-anchoring millis(); this isolates the "no side effects on the ring
// itself" contract directly.
TEST(MonRing, TimeAnchorValidFalseUntilRealEpochSet) {
  // Regression test for a real bug found on hardware (2026-09-11):
  // startMonRing()'s boot-time init() call always seeds anchor_epoch_sec
  // as the placeholder 0 -- timeAnchorValid() must report false until a
  // real setTimeAnchor() call corrects it, so callers appending events with
  // an absolute-timestamp requirement (Beebo::forwardDebugToMonRing()) know
  // to hold off rather than store a record timestamped against epoch 0.
  RingFixture<8> f(/*now=*/0);
  EXPECT_FALSE(f.ring.timeAnchorValid());

  f.ring.setTimeAnchor(1789184340, 0);
  EXPECT_TRUE(f.ring.timeAnchorValid());
}

TEST(MonRing, SetTimeAnchorDoesNotTouchRingContents) {
  // beebo: setTimeAnchor() only ever feeds the DebugLog-facing global
  // anchor now (g_time_anchor_*) -- append*/noteRadio/sampleEnv() take an
  // already-resolved epoch-ms instant directly and never consult it, so
  // there's no "next append resolves against the new anchor" behavior left
  // to verify (that used to be this test's second half); this just pins
  // down that a bare setTimeAnchor() call is inert as far as the ring
  // itself is concerned.
  RingFixture<8> f(1000);
  f.ring.appendRx(makeRx(), f.ms(1000));
  uint32_t count_before = f.ring.count();
  uint32_t next_seq_before = f.ring.nextSeq();

  f.ring.setTimeAnchor(5000, 1000);

  EXPECT_EQ(count_before, f.ring.count());
  EXPECT_EQ(next_seq_before, f.ring.nextSeq());
}

TEST(MonRing, NoteRadioAndSampleEnvAreNoOpsWhenUnchanged) {
  RingFixture<8> f;
  f.ring.noteRadio(makeRadio(), f.ms(1000));
  uint32_t after_first = f.ring.nextSeq();
  f.ring.noteRadio(makeRadio(), f.ms(1001));  // identical config
  EXPECT_EQ(after_first, f.ring.nextSeq());  // no new record stored

  f.ring.noteRadio(makeRadio(915000000, 7, 2500, 5, /*tx_power=*/21), f.ms(1002));  // tx_power changed
  EXPECT_GT(f.ring.nextSeq(), after_first);
}

TEST(MonRing, PauseForReadMakesAppendsNoOps) {
  RingFixture<8> f;
  f.ring.appendRx(makeRx(), f.ms(1000));
  uint32_t before = f.ring.nextSeq();

  f.ring.pauseForRead();
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), f.ms(1001)));
  f.ring.appendTx(makeTx(), f.ms(1001));
  EXPECT_EQ(before, f.ring.nextSeq());  // both calls were no-ops

  f.ring.resumeAfterRead();
  f.ring.appendRx(makeRx(), f.ms(1002));
  EXPECT_GT(f.ring.nextSeq(), before);
}

TEST(MonRing, ResumeAfterReadLeavesManuallyPausedRingPaused) {
  RingFixture<8> f;
  f.ring.setConfig(MON_CAP_ALL);  // manually disabled (MON_CAP_ENABLED clear)
  EXPECT_FALSE(f.ring.enabled());

  f.ring.pauseForRead();
  f.ring.resumeAfterRead();
  EXPECT_FALSE(f.ring.enabled());  // still disabled, not force-re-enabled
}

TEST(MonRing, ClearResetsCountersAndReseedsStartRefs) {
  RingFixture<4> f;
  f.ring.appendRx(makeRx(), 100001002ULL);  // forced overflow: relatch: seq0=SYNC(100001) seq1=RX
  f.ring.appendRx(makeRx(), 100001002ULL);  // seq2=RX
  f.ring.appendRx(makeRx(), 100001002ULL);  // seq3=RX  (ring full)
  f.ring.appendRx(makeRx(), 100001002ULL);  // wraps, reanchors start_sync
  ASSERT_EQ(100001u, f.ring.startTime());

  f.ring.clear(2000, 2000, makeRadio(), makeEnv());
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.nextSeq());
  EXPECT_EQ(0u, f.ring.oldestSeq());
  EXPECT_EQ(2000u, f.ring.startTime());  // reseeded from the clear() call, not zero
  EXPECT_EQ(0u, f.ring.rxCount());
}

TEST(MonRing, ClearReseedsBaseSoImmediateCaptureNeedsNoRelatch) {
  // Regression coverage for the bug this design fixes: clear() must leave the
  // ring with a valid sync reference immediately, without relying on the next
  // capture to manufacture one via relatch -- which noteRadio()/sampleEnv()
  // would skip entirely if the config happens to be unchanged since before
  // the clear.
  RingFixture<8> f;
  f.ring.appendRx(makeRx(), f.ms(1000));
  f.ring.clear(2000, 2000, makeRadio(), makeEnv());
  f.ring.appendRx(makeRx(), 2000001ULL);  // well within the fresh sync period

  EXPECT_EQ(0u, f.ring.syncCount());  // no real SYNC record needed -- start_sync covers it
  EXPECT_EQ(2000u, f.ring.startTime());
}

TEST(MonRing, AppendRxWithResolvedDispositionRoundTrips) {
  RingFixture<8> f;
  RxRecord rx = makeRx();
  MonRing::applyDisposition(rx.disp, /*RX_DISP_FORWARDED=*/2);
  uint32_t seq = f.ring.appendRx(rx, f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), seq, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(RXRT_FORWARDED, (out[0].rx.disp & RXREC_RT_MASK) >> RXREC_RT_SHIFT);
}

TEST(MonRing, QueueFullCorrectionAddsStickyBitWithoutClobberingForwarded) {
  // RX_DISP_QUEUE_FULL corrects a FORWARDED decision that failed to actually
  // enqueue (send_queue full) -- routing stays FORWARDED (the decision was
  // real), the new sticky bit records that it didn't actually go out.
  uint8_t disp = 0;
  MonRing::applyDisposition(disp, /*RX_DISP_FORWARDED=*/2);
  MonRing::applyDisposition(disp, /*RX_DISP_QUEUE_FULL=*/17);
  EXPECT_EQ(RXRT_FORWARDED, (disp & RXREC_RT_MASK) >> RXREC_RT_SHIFT);
  EXPECT_TRUE(disp & RXREC_QUEUE_FULL);
}

TEST(MonRing, BumpRxDropCountNeverDecrementsOnEviction) {
  // POOL_FULL/PARSE_ERR never reach the ring (no Packet to build a record
  // from), so they're plain lifetime counters: they must survive eviction of
  // unrelated resident records untouched.
  RingFixture<2> f;
  EXPECT_EQ(1u, f.ring.bumpRxDropCount(/*RX_DISP_PARSE_ERR=*/12));
  f.ring.appendRx(makeRx(), f.ms(1000));  // seq0=RX
  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full)
  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 (the RX record)
  EXPECT_EQ(1u, f.ring.rxParseErrorCount());
  EXPECT_EQ(0u, f.ring.rxPoolExhaustedCount());
}

TEST(MonRing, RxCountDecrementsOnEviction) {
  // rx_count must track only what's currently resident: bumped at append
  // time, un-bumped when the ring evicts that same record. Using TX appends
  // (not RX) to evict the RX keeps rx_count's decrement from being masked by
  // a same-tick increment.
  RingFixture<2> f;
  RxRecord rx = makeRx();
  MonRing::applyDisposition(rx.disp, /*RX_DISP_FORWARDED=*/2);
  f.ring.appendRx(rx, f.ms(1000));  // seq0=RX
  EXPECT_EQ(1u, f.ring.rxCount());

  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full: [RX0, TX1])
  EXPECT_EQ(1u, f.ring.rxCount());

  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- the disposed RX itself
  EXPECT_EQ(0u, f.ring.rxCount());
}

TEST(MonRing, TxCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendTx(makeTx(), f.ms(1000));  // seq0=TX (TXR_OK)
  EXPECT_EQ(1u, f.ring.txCount());

  f.ring.appendRx(makeRx(), f.ms(1001));  // seq1=RX (ring full: [TX0, RX1])
  EXPECT_EQ(1u, f.ring.txCount());

  f.ring.appendRx(makeRx(), f.ms(1002));  // evicts seq0 -- the TX itself
  EXPECT_EQ(0u, f.ring.txCount());
}

TEST(MonRing, AppendTuneAssignsSeqAndRoundTripsFields) {
  RingFixture<8> f;
  TuneRecord tune = makeTune(TUNE_TX_DELAY_FACTOR, /*old_value=*/10, /*proposed_value=*/14);
  tune.applied = 0;
  tune.reward_before = 8000;
  f.ring.appendTune(tune, f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ((int)sizeof(MonRecord), bytes);
  EXPECT_EQ(MON_TUNE, out[0].kind);
  EXPECT_EQ(TUNE_TX_DELAY_FACTOR, out[0].tune.param_id);
  EXPECT_EQ(0, out[0].tune.applied);
  EXPECT_EQ(10, out[0].tune.old_value);
  EXPECT_EQ(14, out[0].tune.proposed_value);
  EXPECT_EQ(8000u, out[0].tune.reward_before);
  EXPECT_EQ(1u, f.ring.tuneCount());
}

TEST(MonRing, AppendTuneNoOpWhenMasked) {
  RingFixture<8> f;
  f.ring.setConfig(MON_CAP_ENABLED | MON_CAP_TX);  // TUNE masked out
  f.ring.appendTune(makeTune(), f.ms(1000));
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.tuneCount());
}

TEST(MonRing, TuneCapExcludedFromDefaultConfig) {
  // Mirrors MON_CAP_ENV's default-off treatment: MON_TUNE has no
  // controller wired up yet, so it should not be captured unless
  // explicitly opted in via setConfig(), the same way tests must opt ENV in.
  MonRing ring;
  MonRecord buf[8];
  ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), 1000, 1000, RadioRecord{}, EnvRecord{});
  EXPECT_EQ(0u, ring.config() & MON_CAP_TUNE);
}

TEST(MonRing, TuneCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendTune(makeTune(), f.ms(1000));  // seq0=TUNE
  EXPECT_EQ(1u, f.ring.tuneCount());

  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full: [TUNE0, TX1])
  EXPECT_EQ(1u, f.ring.tuneCount());

  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- the TUNE record itself
  EXPECT_EQ(0u, f.ring.tuneCount());
}

TEST(MonRing, AppendEventAssignsSeqAndRoundTripsFaultPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeFaultEvent(EVENT_RX_START_TIMEOUT, /*cumulative=*/1), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ((int)sizeof(MonRecord), bytes);
  EXPECT_EQ(MON_EVENT, out[0].kind);
  EXPECT_EQ(EVENT_RX_START_TIMEOUT, out[0].event.event_type);
  uint32_t cumulative = 0;
  memcpy(&cumulative, &out[0].event.data[0], 4);
  EXPECT_EQ(1u, cumulative);
  EXPECT_EQ(1u, f.ring.eventCount());
}

TEST(MonRing, FaultEventTypesAreDistinctPerCause) {
  // beebo: the whole point of the split -- TX_POOL_FULL, TX_CAD_TIMEOUT, and
  // RX_START_TIMEOUT are three different event types, not one shared type
  // with a reason byte.
  EXPECT_NE(EVENT_TX_POOL_FULL, EVENT_TX_CAD_TIMEOUT);
  EXPECT_NE(EVENT_TX_CAD_TIMEOUT, EVENT_RX_START_TIMEOUT);
  EXPECT_NE(EVENT_TX_POOL_FULL, EVENT_RX_START_TIMEOUT);
}

TEST(MonRing, AppendSettingRoundTripsPayload) {
  // beebo: EVENT_SETTING_CHANGED moved to its own MON_SETTING kind/struct --
  // this record has a fixed shape every instance shares, unlike the
  // genuinely one-off fault/overflow/wrap events MON_EVENT still carries.
  RingFixture<8> f;
  f.ring.appendSetting(makeSettingRecord(SETTING_TUNE_ENABLED, 0, 1, EVENT_SOURCE_BINARY), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_SETTING, out[0].kind);
  EXPECT_EQ(SETTING_TUNE_ENABLED, out[0].setting.setting);
  EXPECT_EQ(EVENT_SOURCE_BINARY, out[0].setting.source);
  EXPECT_EQ(0u, out[0].setting.old_value);
  EXPECT_EQ(1u, out[0].setting.new_value);
  EXPECT_EQ(1u, f.ring.settingCount());
}

TEST(MonRing, AppendCommandRoundTripsPayload) {
  // beebo: EVENT_COMMAND_RUN moved to its own MON_COMMAND kind/struct, same
  // reasoning as MON_SETTING above.
  RingFixture<8> f;
  uint16_t command_id = (222 << 8) | 215;  // CMD_BEEBO<<8 | BEEBO_CMD_SET_TUNE_ENABLED
  f.ring.appendCommand(makeCommandRecord(command_id), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_COMMAND, out[0].kind);
  EXPECT_EQ(EVENT_SOURCE_BINARY, out[0].command.source);
  uint16_t decoded = 0;
  memcpy(&decoded, &out[0].command.command[0], 2);
  EXPECT_EQ(command_id, decoded);
  EXPECT_EQ(1u, f.ring.commandCount());
}

TEST(MonRing, AppendEventRoundTripsAckSuccessPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ACK_SUCCESS,
                                        /*pkt_hash=*/0x2dac9975, /*age_ms=*/560), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_EVENT, out[0].kind);
  EXPECT_EQ(EVENT_ACK_SUCCESS, out[0].event.event_type);
  uint32_t pkt_hash = 0, age_ms = 0;
  memcpy(&pkt_hash, &out[0].event.data[1], 4);
  memcpy(&age_ms, &out[0].event.data[5], 4);
  EXPECT_EQ(0x2dac9975u, pkt_hash);
  EXPECT_EQ(560u, age_ms);
}

TEST(MonRing, AppendEventRoundTripsAckTimeoutPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ACK_TIMEOUT,
                                        /*pkt_hash=*/0x11223344, /*age_ms=*/4200), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(EVENT_ACK_TIMEOUT, out[0].event.event_type);
  uint32_t pkt_hash = 0, age_ms = 0;
  memcpy(&pkt_hash, &out[0].event.data[1], 4);
  memcpy(&age_ms, &out[0].event.data[5], 4);
  EXPECT_EQ(0x11223344u, pkt_hash);
  EXPECT_EQ(4200u, age_ms);
}

TEST(MonRing, AppendEventRoundTripsAckOverflowPayload) {
  // Resource-exhaustion fault, a dedicated event type rather than a third
  // EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT-adjacent verdict -- see MonRing.h's
  // EVENT_ACK_OVERFLOW comment.
  RingFixture<8> f;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ACK_OVERFLOW,
                                        /*pkt_hash=*/0xAABBCCDD, /*age_ms=*/9001), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(EVENT_ACK_OVERFLOW, out[0].event.event_type);
  uint32_t pkt_hash = 0, age_ms = 0;
  memcpy(&pkt_hash, &out[0].event.data[1], 4);
  memcpy(&age_ms, &out[0].event.data[5], 4);
  EXPECT_EQ(0xAABBCCDDu, pkt_hash);
  EXPECT_EQ(9001u, age_ms);
}

TEST(MonRing, AppendEventRoundTripsRptOverflowPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ECHO_OVERFLOW,
                                        /*pkt_hash=*/0x99887766, /*age_ms=*/12000), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(EVENT_ECHO_OVERFLOW, out[0].event.event_type);
  uint32_t pkt_hash = 0, age_ms = 0;
  memcpy(&pkt_hash, &out[0].event.data[1], 4);
  memcpy(&age_ms, &out[0].event.data[5], 4);
  EXPECT_EQ(0x99887766u, pkt_hash);
  EXPECT_EQ(12000u, age_ms);
}

TEST(MonRing, AppendEventRoundTripsDedupTableFullPayload) {
  RingFixture<8> f;
  EventRecord event; memset(&event, 0, sizeof(event));
  event.event_type = EVENT_RX_DEDUP_TABLE_FULL;
  uint32_t cumulative = 7;
  memcpy(&event.data[0], &cumulative, 4);
  f.ring.appendEvent(event, f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(EVENT_RX_DEDUP_TABLE_FULL, out[0].event.event_type);
  uint32_t decoded = 0;
  memcpy(&decoded, &out[0].event.data[0], 4);
  EXPECT_EQ(7u, decoded);
}

TEST(MonRing, AppendEventRoundTripsForwardDenyPayloads) {
  // beebo: allowPacketForward()'s three flood-decline causes (hop-limit,
  // region, loop) -- just the declining packet's own hash, one event type
  // per cause since they're split out of the single RX_DISP_NO_FORWARD
  // routing bit specifically so a downloaded trace can tell them apart AND
  // correlate the specific packet. No cumulative count in the payload --
  // that lifetime total lives in the GET_MONRING header instead (Beebo's
  // own _max_hop_no_fwd_count/etc. member fields), not duplicated here.
  const uint8_t types[] = {EVENT_MAX_HOP_NO_FWD, EVENT_REGION_NO_FWD, EVENT_LOOP_NO_FWD};
  for (uint8_t type : types) {
    RingFixture<8> f;
    EventRecord event; memset(&event, 0, sizeof(event));
    event.event_type = type;
    uint32_t pkt_hash = 0x2dac9975;
    memcpy(&event.data[1], &pkt_hash, 4);
    f.ring.appendEvent(event, f.ms(1000));

    MonRecord out[8];
    uint32_t returned = 0;
    f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
    ASSERT_EQ(1u, returned);
    EXPECT_EQ(type, out[0].event.event_type);
    uint32_t decoded_hash = 0;
    memcpy(&decoded_hash, &out[0].event.data[1], 4);
    EXPECT_EQ(pkt_hash, decoded_hash);
  }
}

TEST(MonRing, AppendEventRoundTripsEchoSuccessPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ECHO_SUCCESS,
                                        /*pkt_hash=*/0x2dac9975, /*age_ms=*/3144), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_EVENT, out[0].kind);
  EXPECT_EQ(EVENT_ECHO_SUCCESS, out[0].event.event_type);
  uint32_t pkt_hash = 0, age_ms = 0;
  memcpy(&pkt_hash, &out[0].event.data[1], 4);
  memcpy(&age_ms, &out[0].event.data[5], 4);
  EXPECT_EQ(0x2dac9975u, pkt_hash);
  EXPECT_EQ(3144u, age_ms);
}

TEST(MonRing, AckSuccessAndEchoSuccessShareTheSamePktHashForOneTransmission) {
  // Regression coverage for the actual correlation guarantee this design
  // depends on: an ack success and an echo success for the SAME original
  // transmission must carry the identical pkt_hash, so a downloaded trace
  // can join them (and the origin MON_TX record) without any other key.
  RingFixture<8> f;
  uint32_t shared_hash = 0x2dac9975;
  f.ring.appendEvent(makeOverflowEvent(EVENT_ACK_SUCCESS, shared_hash, 560), f.ms(1000));
  f.ring.appendEvent(makeOverflowEvent(EVENT_ECHO_SUCCESS, shared_hash, 3144), f.ms(1002));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(2u, returned);
  uint32_t ack_hash = 0, heard_hash = 0;
  memcpy(&ack_hash, &out[0].event.data[1], 4);
  memcpy(&heard_hash, &out[1].event.data[1], 4);
  EXPECT_EQ(ack_hash, heard_hash);
  EXPECT_EQ(shared_hash, ack_hash);
}

TEST(MonRing, AppendEventNoOpWhenMasked) {
  RingFixture<8> f;
  f.ring.setConfig(MON_CAP_ENABLED | MON_CAP_TX);  // EVENT masked out
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), f.ms(1000));
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.eventCount());
}

TEST(MonRing, EventTypeMaskDefaultsToCaptureEverything) {
  RingFixture<8> f;
  EXPECT_EQ(0xFFFFFFFFu, f.ring.eventTypeMask());
}

TEST(MonRing, AppendEventNoOpWhenEventTypeMasked) {
  // beebo: per-EVENT_TYPE capture mask -- a second, finer filter under
  // MON_CAP_EVENT, letting a specific high-volume event type (e.g.
  // EVENT_ACK_SUCCESS) be excluded without losing every other event type.
  RingFixture<8> f;
  f.ring.setEventTypeMask(~(1u << EVENT_TX_POOL_FULL));
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), f.ms(1000));
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.eventCount());

  // A type NOT excluded from the mask still captures normally.
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_CAD_TIMEOUT, 1), f.ms(1000));
  EXPECT_EQ(1u, f.ring.count());
  EXPECT_EQ(1u, f.ring.eventCount());
}

TEST(MonRing, EventCapIncludedInDefaultConfig) {
  // Unlike MON_CAP_ENV/MON_CAP_TUNE (opt-in), EVENT capture is on by
  // default -- fault history matters out of the box, not just for an
  // experimental feature someone has to remember to enable.
  MonRing ring;
  MonRecord buf[8];
  ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), 1000, 1000, RadioRecord{}, EnvRecord{});
  EXPECT_NE(0u, ring.config() & MON_CAP_EVENT);
}

TEST(MonRing, EventCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), f.ms(1000));  // seq0=EVENT
  EXPECT_EQ(1u, f.ring.eventCount());

  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full: [EVENT0, TX1])
  EXPECT_EQ(1u, f.ring.eventCount());

  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- the EVENT record itself
  EXPECT_EQ(0u, f.ring.eventCount());
}

// --------------------------------------------------------------------------
// QoS / SoH objective functions (DYNAMIC_OPTIMIZER_PLAN.md item 10, reward
// redesigned into a "confirm ratio" (QoS) + raw "routed count" pair per the
// plan's goodput reward redesign, 2026-08-24 -- see MonRing.h's comments)
// --------------------------------------------------------------------------

TEST(MonRing, ComputeQosNoExposureIsZeroNotUndefined) {
  MonRing::QosStats s{0, 0, 0, 0};
  EXPECT_EQ(0u, MonRing::computeQos(s));
}

TEST(MonRing, ComputeQosAllConfirmedIsFullScore) {
  MonRing::QosStats s{10, 0, 0, 0};  // 10 ack successes, nothing else
  EXPECT_EQ(10000u, MonRing::computeQos(s));
}

TEST(MonRing, ComputeQosHalfConfirmedIsHalfScore) {
  MonRing::QosStats s{5, 5, 0, 0};  // 5 success, 5 timeout
  EXPECT_EQ(5000u, MonRing::computeQos(s));
}

TEST(MonRing, ComputeRosSumsAckAndEchoSuccess) {
  // The goodput reward's raw-volume half -- deliberately NOT a ratio, and
  // deliberately NOT touched by ack_timeout/echo_attempt (those widen QoS's
  // denominator, not this count).
  MonRing::QosStats s{5, 100, 50, 3};  // 5 ack + 3 echo successes = 8 routed
  EXPECT_EQ(8u, MonRing::computeRos(s));
}

TEST(MonRing, ComputeRosZeroWhenNothingConfirmed) {
  MonRing::QosStats s{0, 10, 20, 0};  // plenty of attempts, no confirmations
  EXPECT_EQ(0u, MonRing::computeRos(s));
}

TEST(MonRing, ComputeQosIsBlindToVolumeByDesign) {
  // The exact gap the goodput redesign documents: a node confirming 1/1 and
  // one confirming 1000/1000 score identically on QoS alone -- ros_count
  // is what distinguishes them (see the two tests above).
  MonRing::QosStats low_volume{1, 0, 0, 0};
  MonRing::QosStats high_volume{1000, 0, 0, 0};
  EXPECT_EQ(MonRing::computeQos(low_volume), MonRing::computeQos(high_volume));
  EXPECT_NE(MonRing::computeRos(low_volume), MonRing::computeRos(high_volume));
}

TEST(MonRing, ComputeQosExposureIsZeroWhenNothingAttempted) {
  MonRing::QosStats s{0, 0, 0, 0};
  EXPECT_EQ(0u, MonRing::computeQosExposure(s));
}

TEST(MonRing, ComputeQosExposureDistinguishesNoAttemptsFromAllFailed) {
  // The gap computeQos() alone can't see: both read as qos=0, but only one
  // of them actually has confirmable attempts behind that 0%.
  MonRing::QosStats no_attempts{0, 0, 0, 0};
  MonRing::QosStats all_failed{0, 10, 0, 0};
  EXPECT_EQ(0u, MonRing::computeQos(no_attempts));
  EXPECT_EQ(0u, MonRing::computeQos(all_failed));
  EXPECT_EQ(0u, MonRing::computeQosExposure(no_attempts));
  EXPECT_EQ(10u, MonRing::computeQosExposure(all_failed));
}

TEST(MonRing, ComputeSohNoActivityIsFullScoreNotUnhealthy) {
  MonRing::SohStats s{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*rx_activity=*/0, /*tx_activity=*/0};
  EXPECT_EQ(10000u, MonRing::computeSoh(s));
}

TEST(MonRing, ComputeSohZeroFaultsIsFullScore) {
  MonRing::SohStats s{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*rx_activity=*/1000, /*tx_activity=*/500};
  EXPECT_EQ(10000u, MonRing::computeSoh(s));
}

TEST(MonRing, ComputeSohFaultsScaleDownByRateNotRawCount) {
  // 10 faults out of 1000 total activity -- 1% fault rate -> 99% SoH,
  // regardless of which specific fault-count field(s) contributed.
  MonRing::SohStats s{10, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*rx_activity=*/1000, /*tx_activity=*/0};
  EXPECT_EQ(9900u, MonRing::computeSoh(s));
}

TEST(MonRing, ComputeSohFaultsExceedingActivityFloorsAtZero) {
  MonRing::SohStats s{50, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*rx_activity=*/10, /*tx_activity=*/0};
  EXPECT_EQ(0u, MonRing::computeSoh(s));
}

TEST(MonRing, QosAndSohAreIndependentAxes) {
  // The whole point: a node can be perfectly healthy (no internal faults)
  // while QoS is mediocre for reasons outside its control (e.g. a lot of
  // ACK timeouts from real RF conditions, not resource exhaustion).
  MonRing::SohStats soh{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /*rx_activity=*/1000, /*tx_activity=*/500};
  MonRing::QosStats qos{2, 8, 0, 0};  // mostly timeouts -- poor delivery
  EXPECT_EQ(10000u, MonRing::computeSoh(soh));  // fully healthy
  EXPECT_EQ(2000u, MonRing::computeQos(qos));   // but only 20% QoS
}

TEST(MonRing, ComputeBusyPctSplitsWindowProportionally) {
  uint16_t rx, tx, lx, sys;
  // 200ms RX + 100ms TX + 50ms LX + 30ms SYS out of a 1000ms window ->
  // 2000/1000/500/300 (0.01% precision), 62% idle.
  MonRing::computeBusyPct(200000, 100000, 50000, 30000, 1000000, rx, tx, lx, sys);
  EXPECT_EQ(2000, rx);
  EXPECT_EQ(1000, tx);
  EXPECT_EQ(500, lx);
  EXPECT_EQ(300, sys);
}

TEST(MonRing, ComputeBusyPctAllIdleIsAllZero) {
  uint16_t rx, tx, lx, sys;
  MonRing::computeBusyPct(0, 0, 0, 0, 1000000, rx, tx, lx, sys);
  EXPECT_EQ(0, rx);
  EXPECT_EQ(0, tx);
  EXPECT_EQ(0, lx);
  EXPECT_EQ(0, sys);
}

TEST(MonRing, ComputeBusyPctZeroWindowIsAllZeroNotDivByZero) {
  uint16_t rx, tx, lx, sys;
  MonRing::computeBusyPct(500000, 500000, 500000, 500000, 0, rx, tx, lx, sys);
  EXPECT_EQ(0, rx);
  EXPECT_EQ(0, tx);
  EXPECT_EQ(0, lx);
  EXPECT_EQ(0, sys);
}

TEST(MonRing, ComputeBusyPctFullyBusyLeavesNoIdle) {
  uint16_t rx, tx, lx, sys;
  // Exactly the whole window split four ways -- sums to exactly 10000, no clamp needed.
  MonRing::computeBusyPct(400000, 300000, 200000, 100000, 1000000, rx, tx, lx, sys);
  EXPECT_EQ(4000, rx);
  EXPECT_EQ(3000, tx);
  EXPECT_EQ(2000, lx);
  EXPECT_EQ(1000, sys);
  EXPECT_EQ(10000, rx + tx + lx + sys);
}

TEST(MonRing, ComputeBusyPctOverBudgetClampsProportionally) {
  uint16_t rx, tx, lx, sys;
  // A stray timing overlap pushes the raw sum past 10000 (15000) -- clamp
  // renormalizes down to 10000 while preserving the 6:3:3:3 ratio (verify
  // the clamp keeps sum == 10000 and ratios preserved rather than any
  // specific naive expectation).
  MonRing::computeBusyPct(600000, 300000, 300000, 300000, 1000000, rx, tx, lx, sys);
  EXPECT_EQ(10000, rx + tx + lx + sys);
  EXPECT_EQ(4000, rx);   // 6000 * 10000/15000 = 4000
  EXPECT_EQ(2000, tx);   // 3000 * 10000/15000 = 2000
  EXPECT_EQ(2000, lx);   // 3000 * 10000/15000 = 2000
  EXPECT_EQ(2000, sys);  // 3000 * 10000/15000 = 2000
}

TEST(MonRing, ComputeRoutePctHalfWindowIsHalfScale) {
  // 30s busy out of a 60s window -> 5000 (0.01% precision, half of 10000).
  EXPECT_EQ(5000, MonRing::computeRoutePct(30000000, 60000000));
}

TEST(MonRing, ComputeRoutePctZeroIsZero) {
  EXPECT_EQ(0, MonRing::computeRoutePct(0, 60000000));
}

TEST(MonRing, ComputeRoutePctZeroWindowIsZeroNotDivByZero) {
  EXPECT_EQ(0, MonRing::computeRoutePct(30000000, 0));
}

TEST(MonRing, ComputeRoutePctFullWindowIsMaxScale) {
  EXPECT_EQ(10000, MonRing::computeRoutePct(60000000, 60000000));
}

TEST(MonRing, ComputeRoutePctOverBudgetClampsAtMax) {
  // A stray timing overlap pushes busy_us past the window -- clamp at
  // 10000 rather than overflowing the wire field's uint16 range.
  EXPECT_EQ(10000, MonRing::computeRoutePct(90000000, 60000000));
}

RouteRecord makeRoute(uint16_t rx_busy = 1234, uint16_t tx_busy = 567,
                      uint16_t tx_wait_airtime = 8901, uint16_t tx_wait_cad = 234) {
  RouteRecord route{};
  route.rx_busy = rx_busy;
  route.tx_busy = tx_busy;
  route.tx_wait_airtime = tx_wait_airtime;
  route.tx_wait_cad = tx_wait_cad;
  return route;
}

TEST(MonRing, AppendRouteRoundTripsAllFields) {
  RingFixture<8> f;
  f.ring.appendRoute(makeRoute(), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_ROUTE, out[0].kind);
  EXPECT_EQ(1234, out[0].route.rx_busy);
  EXPECT_EQ(0, out[0].route.rx_wait_relay);
  EXPECT_EQ(567, out[0].route.tx_busy);
  EXPECT_EQ(8901, out[0].route.tx_wait_airtime);
  EXPECT_EQ(234, out[0].route.tx_wait_cad);
  EXPECT_EQ(1u, f.ring.routeCount());
}

TEST(MonRing, AppendRouteNoOpWhenCapMissing) {
  RingFixture<8> f;
  f.ring.setConfig((MON_CAP_ALL & ~MON_CAP_EVENT) | MON_CAP_ENABLED);
  f.ring.appendRoute(makeRoute(), f.ms(1000));
  EXPECT_EQ(0u, f.ring.routeCount());
}

TEST(MonRing, RouteCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendRoute(makeRoute(), f.ms(1000));  // seq0=ROUTE
  EXPECT_EQ(1u, f.ring.routeCount());

  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full: [ROUTE0, TX1])
  EXPECT_EQ(1u, f.ring.routeCount());

  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- the ROUTE record itself
  EXPECT_EQ(0u, f.ring.routeCount());
}

// beebo: MON_DEBUG (folded-in RLOG, plans/MONITORING_UNIFICATION.md).
DebugRecord makeDebug(uint8_t type = 5, uint8_t file_id = 2, uint16_t line = 63,
                       int32_t detail = 0x1234) {
  DebugRecord d; memset(&d, 0, sizeof(d));
  d.type = type; d.file_id = file_id; d.line = line; d.detail = detail;
  return d;
}

TEST(MonRing, AppendDebugRoundTripsAllFields) {
  RingFixture<8> f;
  f.ring.appendDebug(makeDebug(), f.ms(1000));

  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(1u, returned);
  EXPECT_EQ(MON_DEBUG, out[0].kind);
  EXPECT_EQ(5, out[0].debug.type);
  EXPECT_EQ(2, out[0].debug.file_id);
  EXPECT_EQ(63, out[0].debug.line);
  EXPECT_EQ(0x1234, out[0].debug.detail);
  EXPECT_EQ(1u, f.ring.debugCount());
}

TEST(MonRing, AppendDebugNoOpWhenCapMissing) {
  RingFixture<8> f;
  f.ring.setConfig((MON_CAP_ALL & ~MON_CAP_EVENT) | MON_CAP_ENABLED);
  f.ring.appendDebug(makeDebug(), f.ms(1000));
  EXPECT_EQ(0u, f.ring.debugCount());
}

TEST(MonRing, DebugCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendDebug(makeDebug(), f.ms(1000));  // seq0=DEBUG
  EXPECT_EQ(1u, f.ring.debugCount());

  f.ring.appendTx(makeTx(), f.ms(1001));  // seq1=TX (ring full: [DEBUG0, TX1])
  EXPECT_EQ(1u, f.ring.debugCount());

  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- the DEBUG record itself
  EXPECT_EQ(0u, f.ring.debugCount());
}

// beebo: regression test for the eviction switch's continuation-bit
// masking fix (plans/MONITORING_UNIFICATION.md Design #5) -- a record
// whose stored `.kind` byte has RLOG_CONT_BIT set must still decrement
// the correct per-kind resident counter on eviction, not fall through to
// `default: break` and leak the counter. No production code sets this bit
// yet (no multi-record writer exists), so this stores the bit directly to
// exercise the eviction path in isolation.
TEST(MonRing, EvictionMasksContinuationBitBeforeDispatch) {
  RingFixture<2> f;
  f.ring.appendDebug(makeDebug(), f.ms(1000));  // seq0=DEBUG, stored at buf[0]
  EXPECT_EQ(1u, f.ring.debugCount());

  // Simulate a continuation slot of a (hypothetical) multi-record DEBUG
  // event occupying seq0's raw storage -- appendDebug() itself always
  // clears this bit (single-slot events only), so flip it directly on the
  // backing buffer to exercise the eviction masking fix in isolation.
  f.buf[0].kind |= RLOG_CONT_BIT;
  ASSERT_EQ(MON_DEBUG | RLOG_CONT_BIT, f.buf[0].kind);

  f.ring.appendTx(makeTx(), f.ms(1001));  // ring full: [DEBUG0(cont-bit set), TX1]
  f.ring.appendTx(makeTx(), f.ms(1002));  // evicts seq0 -- must still decrement debugCount()

  EXPECT_EQ(0u, f.ring.debugCount());
}

// beebo: MLOG live-sink hook (plans/MLOG_LIVE_STREAM.md) -- a plain
// function pointer, no per-call-site state, so these tests route through a
// static capture vector rather than a lambda-with-capture (LiveSink can't
// bind one, matching this codebase's no-dynamic-allocation convention).
namespace {
std::vector<MonRecord> g_live_sink_calls;
void captureLiveSink(const MonRecord &rec) { g_live_sink_calls.push_back(rec); }
}  // namespace

TEST(MonRingLiveSink, FiresOncePerStoreForEveryAppendKind) {
  g_live_sink_calls.clear();
  RingFixture<8> f;
  f.ring.setLiveSink(&captureLiveSink);

  f.ring.appendRx(makeRx(), f.ms(1000));
  f.ring.appendTx(makeTx(), f.ms(1001));
  f.ring.sampleEnv(makeEnv(), f.ms(1002));

  ASSERT_EQ(3u, g_live_sink_calls.size());
  EXPECT_EQ(MON_RX, g_live_sink_calls[0].kind);
  EXPECT_EQ(MON_TX, g_live_sink_calls[1].kind);
  EXPECT_EQ(MON_ENV, g_live_sink_calls[2].kind);
}

TEST(MonRingLiveSink, NeverFiresWhenNoSinkSet) {
  g_live_sink_calls.clear();
  RingFixture<8> f;   // setLiveSink() never called -- native suite's default
  f.ring.appendRx(makeRx(), f.ms(1000));
  f.ring.appendTx(makeTx(), f.ms(1001));
  EXPECT_TRUE(g_live_sink_calls.empty());
}

TEST(MonRingLiveSink, DisabledKindNeverReachesSink) {
  // beebo: a MON_CAP_* gated-off kind never reaches _store() at all (design
  // decision 4) -- appendXxx() itself is the gate, live-relay included.
  g_live_sink_calls.clear();
  RingFixture<8> f;
  f.ring.setLiveSink(&captureLiveSink);
  f.ring.setConfig(MON_CAP_ENABLED);   // every per-kind bit off, RX included
  f.ring.appendRx(makeRx(), f.ms(1000));
  EXPECT_TRUE(g_live_sink_calls.empty());
}

TEST(MonRingMlogReplay, EmptyRingBeginReplayIsAlreadyDone) {
  RingFixture<8> f;
  f.ring.beginMlogReplay();
  EXPECT_FALSE(f.ring.isMlogReplaying());
  EXPECT_FALSE(f.ring.mlogReplayStep());
}

TEST(MonRingMlogReplay, OneRecordPerStepOldestFirst) {
  g_live_sink_calls.clear();
  RingFixture<8> f;
  f.ring.appendRx(makeRx(0x1), f.ms(1000));
  f.ring.appendTx(makeTx(0x2), f.ms(1001));
  f.ring.appendRx(makeRx(0x3), f.ms(1002));
  g_live_sink_calls.clear();   // the appends above didn't have a sink attached yet

  f.ring.setLiveSink(&captureLiveSink);
  f.ring.beginMlogReplay();
  EXPECT_TRUE(f.ring.isMlogReplaying());

  // beginMlogReplay() itself immediately injects the SYNC start-ref (mirrors
  // GET_MONRING's own first-page splice) before the real per-seq walk below
  // -- RADIO/ENV aren't injected (no noteRadio()/sampleEnv() call in this
  // fixture, see emitStartRef()'s own not-yet-sampled gates for both).
  ASSERT_EQ(1u, g_live_sink_calls.size());
  EXPECT_EQ(MON_SYNC, g_live_sink_calls[0].kind);

  ASSERT_TRUE(f.ring.mlogReplayStep());
  ASSERT_TRUE(f.ring.mlogReplayStep());
  EXPECT_FALSE(f.ring.mlogReplayStep());   // 3rd (last) record: replay completes
  EXPECT_FALSE(f.ring.isMlogReplaying());

  ASSERT_EQ(4u, g_live_sink_calls.size());
  EXPECT_EQ(MON_RX, g_live_sink_calls[1].kind);
  EXPECT_EQ(0x1u, g_live_sink_calls[1].rx.pkt_hash);
  EXPECT_EQ(MON_TX, g_live_sink_calls[2].kind);
  EXPECT_EQ(0x2u, g_live_sink_calls[2].tx.pkt_hash);
  EXPECT_EQ(MON_RX, g_live_sink_calls[3].kind);
  EXPECT_EQ(0x3u, g_live_sink_calls[3].rx.pkt_hash);

  // Further calls are no-ops once done.
  EXPECT_FALSE(f.ring.mlogReplayStep());
  EXPECT_EQ(4u, g_live_sink_calls.size());
}

TEST(MonRingMlogReplay, StartRefAlwaysInjectedForSyncEvenWhenRealRecordCoversSlot) {
  // beebo: unlike RADIO/ENV, MON_SYNC is injected unconditionally even when
  // the ring's own oldest resident record already IS a MON_SYNC -- found on
  // real hardware (2026-09-11) that skipping it left a brand-new live
  // noteRadio()/sampleEnv() append's abs_time unresolved whenever it raced
  // ahead of that real SYNC record still queued in mlogReplayStep()'s paced
  // (one-per-tick) walk. A duplicate real SYNC arriving later via that walk
  // is harmless, so there's no reason to withhold the synthetic one here.
  g_live_sink_calls.clear();
  RingFixture<8> f;
  f.ring.appendRx(makeRx(0x1), 100004000ULL);   // forced overflow -> relatches -> real MON_SYNC, then RX
  g_live_sink_calls.clear();

  f.ring.setLiveSink(&captureLiveSink);
  f.ring.beginMlogReplay();

  // SYNC is injected despite the real record already covering that slot;
  // RADIO still isn't (no real noteRadio() call to draw from, see
  // emitStartRef()'s own not-yet-sampled gate).
  ASSERT_EQ(1u, g_live_sink_calls.size());
  EXPECT_EQ(MON_SYNC, g_live_sink_calls[0].kind);
}

TEST(MonRingMlogReplay, StartRefInjectedEvenWhenRingEmpty) {
  // beebo: start_sync is populated from init() onward, so a freshly-enabled
  // MLOG stream against a totally empty ring still gets an immediate time
  // base rather than waiting for the ring to hold anything. RADIO/ENV are
  // still withheld either way (see emitStartRef()'s own not-yet-sampled
  // gates) -- neither has a real noteRadio()/sampleEnv() call in this
  // fixture, empty ring or not.
  g_live_sink_calls.clear();
  RingFixture<8> f;   // never appended to -- count() == 0
  f.ring.setLiveSink(&captureLiveSink);
  f.ring.beginMlogReplay();
  EXPECT_FALSE(f.ring.isMlogReplaying());   // nothing real to replay
  ASSERT_EQ(1u, g_live_sink_calls.size());
  EXPECT_EQ(MON_SYNC, g_live_sink_calls[0].kind);
}

// beebo: two-phase replay (plans/MLOG_LIVE_STREAM.md) -- requestMlogReplay()
// only arms a pending flag; beginMlogReplay() (and its synchronous start-ref
// push) doesn't fire until the caller actually calls it, letting Beebo.cpp
// defer that call until DLOG/RLOG's own replay has drained, so MLOG's
// "now"-timestamped lines never print ahead of DLOG/RLOG's replayed
// (chronologically earlier) backlog.
TEST(MonRingMlogReplay, StartRefInjectsRadioOnceReallySampled) {
  // Regression test for a real bug found on hardware (2026-09-11): once a
  // real noteRadio() call has happened, its config must be injected as the
  // RADIO start-ref on every subsequent beginMlogReplay() -- confirming the
  // withheld-until-sampled behavior above doesn't withhold it forever.
  g_live_sink_calls.clear();
  RingFixture<8> f;
  // beebo: an RX ahead of the RADIO record means oldestSeq() itself is
  // neither a SYNC nor a RADIO record, so beginMlogReplay()'s "already
  // covers this slot" skip doesn't fire for either -- both get injected as
  // synthetic start-refs (see beginMlogReplay()'s own comment), letting
  // this test check RADIO's injected value directly.
  f.ring.appendRx(makeRx(0x1), f.ms(1000));
  f.ring.noteRadio(makeRadio(915000000), f.ms(1001));
  g_live_sink_calls.clear();

  f.ring.setLiveSink(&captureLiveSink);
  f.ring.beginMlogReplay();

  bool saw_radio = false;
  for (auto &rec : g_live_sink_calls) {
    if (rec.kind == MON_RADIO) { saw_radio = true; EXPECT_EQ(915000000u, rec.radio.freq); }
  }
  EXPECT_TRUE(saw_radio);
}

TEST(MonRingMlogReplay, RequestOnlyArmsPendingDoesNotPushYet) {
  g_live_sink_calls.clear();
  RingFixture<8> f;
  f.ring.setLiveSink(&captureLiveSink);
  EXPECT_FALSE(f.ring.mlogReplayPending());

  f.ring.requestMlogReplay();

  EXPECT_TRUE(f.ring.mlogReplayPending());
  EXPECT_FALSE(f.ring.isMlogReplaying());
  EXPECT_TRUE(g_live_sink_calls.empty());   // no start-refs pushed yet
}

TEST(MonRingMlogReplay, BeginMlogReplayClearsPending) {
  RingFixture<8> f;
  f.ring.requestMlogReplay();
  ASSERT_TRUE(f.ring.mlogReplayPending());

  f.ring.beginMlogReplay();

  EXPECT_FALSE(f.ring.mlogReplayPending());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
