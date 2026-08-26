#include <gtest/gtest.h>
#include <vector>
#include <helpers/MonRing.h>

namespace {

// A handful of small rings, each with its own backing buffer, sized in whole
// records so eviction math is easy to reason about in tests. init() seeds
// start_sync/start_radio/start_env from a zeroed radio/env at `now` (default
// 1000, matching the capture times most tests use), without consuming a ring
// slot -- so unless a test forces a relatch (setSyncPeriod()) or an actual
// radio/env change, the ring starts out genuinely empty.
template <uint32_t N>
struct RingFixture {
  MonRecord buf[N];
  MonRing ring;
  RingFixture(uint32_t now = 1000) {
    ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), now, RadioRecord{}, EnvRecord{});
    // MON_CAP_ENV is opt-in in production (excluded from the default config),
    // but most of these tests exercise sampleEnv() directly, so enable every
    // kind here; tests that care about masking set their own config.
    ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
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

EnvRecord makeEnv(uint16_t batt_mv = 3700) {
  EnvRecord env; memset(&env, 0, sizeof(env));
  env.batt_mv = batt_mv;
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

TEST(MonRing, InitRejectsNullOrUndersizedBuffer) {
  MonRing ring;
  EXPECT_FALSE(ring.init(nullptr, 1024, 1000, RadioRecord{}, EnvRecord{}));
  uint8_t tiny[4];
  EXPECT_FALSE(ring.init(tiny, sizeof(tiny), 1000, RadioRecord{}, EnvRecord{}));
  EXPECT_FALSE(ring.allocated());
}

TEST(MonRing, AppendRxAssignsSequentialSeq) {
  RingFixture<8> f;  // seeded at now=1000
  uint32_t seq0 = f.ring.appendRx(makeRx(), 1000);
  uint32_t seq1 = f.ring.appendRx(makeRx(), 1001);
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
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), 1000));
  EXPECT_EQ(0u, f.ring.count());

  f.ring.setConfig(MON_CAP_ENABLED | MON_CAP_TX);  // RX masked out
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), 1000));
  EXPECT_EQ(0u, f.ring.count());
}

TEST(MonRing, SerializeForwardWalkFiltersByAfterSeq) {
  RingFixture<8> f;
  f.ring.appendRx(makeRx(1), 1000);  // seq0 = RX
  f.ring.appendRx(makeRx(2), 1001);  // seq1 = RX
  f.ring.appendRx(makeRx(3), 1002);  // seq2 = RX

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
  for (int i = 0; i < 5; i++) f.ring.appendRx(makeRx(i), 1000 + i);

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
  f.ring.appendRx(makeRx(), 1000);
  MonRecord out[8];
  uint32_t returned = 0;
  int bytes = f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), f.ring.nextSeq(), &returned);
  EXPECT_EQ(0, bytes);
  EXPECT_EQ(0u, returned);
}

TEST(MonRing, EvictionReanchorsStartSyncAndOldestSeqAdvances) {
  // Capacity 4. init() at now=1000 seeds start_sync but stores no record. A
  // short sync period forces the very next capture to relatch (store a real
  // SYNC record), which then fills the ring and gets evicted.
  RingFixture<4> f;
  f.ring.setSyncPeriod(1);
  f.ring.appendRx(makeRx(1), 1002);  // period elapsed: seq0=SYNC(1002) seq1=RX
  f.ring.appendRx(makeRx(2), 1002);  // seq2=RX
  f.ring.appendRx(makeRx(3), 1002);  // seq3=RX  (ring now full: 0,1,2,3)
  EXPECT_EQ(0u, f.ring.oldestSeq());

  f.ring.appendRx(makeRx(4), 1002);  // seq4=RX, evicts seq0 (SYNC)
  EXPECT_EQ(1u, f.ring.oldestSeq());
  EXPECT_EQ(5u, f.ring.nextSeq());
  EXPECT_EQ(4u, f.ring.count());
  EXPECT_EQ(1002u, f.ring.startTime());  // reanchored from the evicted SYNC
}

TEST(MonRing, EmitStartRefAlwaysValidFromBootSeedBeforeAnyEviction) {
  // start_sync/start_radio/start_env are always populated once init() has
  // run -- even before anything has actually changed or been evicted -- so
  // emitStartRef() must report a value for every kind immediately.
  RingFixture<8> f;
  MonRecord dest;
  ASSERT_TRUE(f.ring.emitStartRef(MON_SYNC, &dest));
  EXPECT_EQ(MON_SYNC, dest.kind);
  EXPECT_EQ(1000u, dest.sync.timestamp);  // the init() boot seed
  EXPECT_EQ(MONRING_ABI_VERSION, dest.sync.abi_version);
  ASSERT_TRUE(f.ring.emitStartRef(MON_RADIO, &dest));
  EXPECT_EQ(MON_RADIO, dest.kind);
  ASSERT_TRUE(f.ring.emitStartRef(MON_ENV, &dest));
  EXPECT_EQ(MON_ENV, dest.kind);
}

TEST(MonRing, EmitStartRefAllThreeKindsAfterFullWrap) {
  RingFixture<3> f;
  f.ring.noteRadio(makeRadio(), 1000);  // seq0=RADIO (base already latched by init())
  f.ring.sampleEnv(makeEnv(), 1000);    // seq1=ENV
  f.ring.appendTx(makeTx(), 1001);      // seq2=TX (ring full: RADIO,ENV,TX)
  f.ring.appendTx(makeTx(), 1002);      // evicts RADIO(0)
  f.ring.appendTx(makeTx(), 1003);      // evicts ENV(1)

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
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(), 1002);  // period elapsed: seq0=SYNC seq1=RADIO
  f.ring.sampleEnv(makeEnv(), 1002);    // seq2=ENV
  f.ring.appendRx(makeRx(), 1002);      // seq3=RX

  EXPECT_TRUE(injectionPlan(f.ring, f.ring.oldestSeq()).empty());
}

TEST(MonRing, InjectionPlanEvictedSyncOnly) {
  // Ring wraps just enough to evict the real SYNC(0); RADIO(1) and ENV(2)
  // are still resident, so only sync needs to be synthesized ahead of them.
  RingFixture<3> f;
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(), 1002);  // period elapsed: relatch (seq0=SYNC seq1=RADIO)
  f.ring.setSyncPeriod(10000);          // no more relatches for the rest of this test
  f.ring.sampleEnv(makeEnv(), 1002);    // seq2=ENV (ring full)
  f.ring.appendTx(makeTx(), 1003);      // evicts SYNC(0)

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
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(), 1002);  // period elapsed: relatch (seq0=SYNC seq1=RADIO)
  f.ring.setSyncPeriod(10000);          // no more relatches for the rest of this test
  f.ring.sampleEnv(makeEnv(), 1002);    // seq2=ENV
  f.ring.appendTx(makeTx(), 1002);      // seq3=TX (ring full)
  f.ring.appendTx(makeTx(), 1003);      // evicts SYNC(0)
  f.ring.appendTx(makeTx(), 1004);      // evicts RADIO(1)

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
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(), 1002);  // period elapsed: relatch
  f.ring.setSyncPeriod(10000);          // no more relatches for the rest of this test
  f.ring.sampleEnv(makeEnv(), 1002);
  f.ring.appendTx(makeTx(), 1003);
  f.ring.appendTx(makeTx(), 1004);
  f.ring.appendTx(makeTx(), 1005);

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
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(), 1002);  // period elapsed: relatch
  f.ring.setSyncPeriod(10000);          // no more relatches for the rest of this test
  f.ring.sampleEnv(makeEnv(), 1002);
  f.ring.appendRx(makeRx(), 1003);
  f.ring.appendRx(makeRx(), 1004);
  f.ring.appendRx(makeRx(), 1005);

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
  f.ring.setSyncPeriod(1);
  f.ring.noteRadio(makeRadio(915000000), 1002);  // period elapsed: relatch (seq0=SYNC seq1=RADIO(915))
  f.ring.setSyncPeriod(10000);                    // no more relatches for the rest of this test
  f.ring.sampleEnv(makeEnv(), 1002);              // seq2=ENV
  f.ring.noteRadio(makeRadio(868000000), 1003);   // seq3=RADIO(868) (ring full)
  f.ring.appendRx(makeRx(), 1004);  // evicts SYNC(0)
  f.ring.appendRx(makeRx(), 1005);  // evicts RADIO(1) -- original 915 config

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

TEST(MonRing, EnsureSyncRelatchesAfterPeriodElapses) {
  RingFixture<8> f;  // seeded at now=1000
  f.ring.setSyncPeriod(100);
  f.ring.appendRx(makeRx(), 1000);  // within period, base already latched by init(): seq0=RX
  f.ring.appendRx(makeRx(), 1050);  // within period: no new SYNC (seq1=RX)
  EXPECT_EQ(2u, f.ring.nextSeq());

  f.ring.appendRx(makeRx(), 1101);  // period elapsed: relatch (seq2=SYNC seq3=RX)
  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(4u, returned);
  EXPECT_EQ(MON_RX, out[0].kind);
  EXPECT_EQ(MON_RX, out[1].kind);
  EXPECT_EQ(MON_SYNC, out[2].kind);  // relatch inserted before the 3rd RX
  EXPECT_EQ(1101u, out[2].sync.timestamp);
  EXPECT_EQ(MON_RX, out[3].kind);
}

TEST(MonRing, EnsureSyncHandlesUint32EpochRolloverAsAForwardRelatch) {
  // beebo: plain unsigned `now - _base` modular subtraction already
  // computes the correct small elapsed value across the real uint32 epoch
  // rollover (7 Feb 2106) -- the wrapped difference IS the true elapsed
  // time, same as any other wrapping-counter difference. Verifies that
  // directly rather than assuming it.
  RingFixture<8> f(0xFFFFFFFFu - 50);  // seeded 50s before the uint32 rollover
  f.ring.setSyncPeriod(100);
  f.ring.appendRx(makeRx(), 30);  // wrapped clock: 50s before rollover + 30s after = 80s elapsed
  EXPECT_EQ(1u, f.ring.nextSeq());  // period (100) not elapsed yet: no relatch, still just seq0=RX

  f.ring.appendRx(makeRx(), 60);  // 50 + 60 = 110s elapsed since base: relatch due
  MonRecord out[8];
  uint32_t returned = 0;
  f.ring.serialize(reinterpret_cast<uint8_t *>(out), sizeof(out), 0, &returned);
  ASSERT_EQ(3u, returned);
  EXPECT_EQ(MON_RX, out[0].kind);
  EXPECT_EQ(MON_SYNC, out[1].kind);
  EXPECT_EQ(60u, out[1].sync.timestamp);  // relatched to the new (wrapped, small) `now`
  EXPECT_EQ(MON_RX, out[2].kind);
}

TEST(MonRing, NoteRadioAndSampleEnvAreNoOpsWhenUnchanged) {
  RingFixture<8> f;
  f.ring.noteRadio(makeRadio(), 1000);
  uint32_t after_first = f.ring.nextSeq();
  f.ring.noteRadio(makeRadio(), 1001);  // identical config
  EXPECT_EQ(after_first, f.ring.nextSeq());  // no new record stored

  f.ring.noteRadio(makeRadio(915000000, 7, 2500, 5, /*tx_power=*/21), 1002);  // tx_power changed
  EXPECT_GT(f.ring.nextSeq(), after_first);
}

TEST(MonRing, PauseForReadMakesAppendsNoOps) {
  RingFixture<8> f;
  f.ring.appendRx(makeRx(), 1000);
  uint32_t before = f.ring.nextSeq();

  f.ring.pauseForRead();
  EXPECT_EQ(0xFFFFFFFFu, f.ring.appendRx(makeRx(), 1001));
  f.ring.appendTx(makeTx(), 1001);
  EXPECT_EQ(before, f.ring.nextSeq());  // both calls were no-ops

  f.ring.resumeAfterRead();
  f.ring.appendRx(makeRx(), 1002);
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
  f.ring.setSyncPeriod(1);
  f.ring.appendRx(makeRx(), 1002);  // relatch: seq0=SYNC seq1=RX
  f.ring.appendRx(makeRx(), 1002);  // seq2=RX
  f.ring.appendRx(makeRx(), 1002);  // seq3=RX  (ring full)
  f.ring.appendRx(makeRx(), 1002);  // wraps, reanchors start_sync
  ASSERT_EQ(1002u, f.ring.startTime());

  f.ring.clear(2000, makeRadio(), makeEnv());
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
  f.ring.appendRx(makeRx(), 1000);
  f.ring.clear(2000, makeRadio(), makeEnv());
  f.ring.appendRx(makeRx(), 2001);  // well within the fresh sync period

  EXPECT_EQ(0u, f.ring.syncCount());  // no real SYNC record needed -- start_sync covers it
  EXPECT_EQ(2000u, f.ring.startTime());
}

TEST(MonRing, AppendRxWithResolvedDispositionRoundTrips) {
  RingFixture<8> f;
  RxRecord rx = makeRx();
  MonRing::applyDisposition(rx.disp, /*RX_DISP_FORWARDED=*/2);
  uint32_t seq = f.ring.appendRx(rx, 1000);

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
  f.ring.appendRx(makeRx(), 1000);  // seq0=RX
  f.ring.appendTx(makeTx(), 1001);  // seq1=TX (ring full)
  f.ring.appendTx(makeTx(), 1002);  // evicts seq0 (the RX record)
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
  f.ring.appendRx(rx, 1000);  // seq0=RX
  EXPECT_EQ(1u, f.ring.rxCount());

  f.ring.appendTx(makeTx(), 1001);  // seq1=TX (ring full: [RX0, TX1])
  EXPECT_EQ(1u, f.ring.rxCount());

  f.ring.appendTx(makeTx(), 1002);  // evicts seq0 -- the disposed RX itself
  EXPECT_EQ(0u, f.ring.rxCount());
}

TEST(MonRing, TxCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendTx(makeTx(), 1000);  // seq0=TX (TXR_OK)
  EXPECT_EQ(1u, f.ring.txCount());

  f.ring.appendRx(makeRx(), 1001);  // seq1=RX (ring full: [TX0, RX1])
  EXPECT_EQ(1u, f.ring.txCount());

  f.ring.appendRx(makeRx(), 1002);  // evicts seq0 -- the TX itself
  EXPECT_EQ(0u, f.ring.txCount());
}

TEST(MonRing, AppendTuneAssignsSeqAndRoundTripsFields) {
  RingFixture<8> f;
  TuneRecord tune = makeTune(TUNE_TX_DELAY_FACTOR, /*old_value=*/10, /*proposed_value=*/14);
  tune.applied = 0;
  tune.reward_before = 8000;
  f.ring.appendTune(tune, 1000);

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
  f.ring.appendTune(makeTune(), 1000);
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.tuneCount());
}

TEST(MonRing, TuneCapExcludedFromDefaultConfig) {
  // Mirrors MON_CAP_ENV's default-off treatment: MON_TUNE has no
  // controller wired up yet, so it should not be captured unless
  // explicitly opted in via setConfig(), the same way tests must opt ENV in.
  MonRing ring;
  MonRecord buf[8];
  ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), 1000, RadioRecord{}, EnvRecord{});
  EXPECT_EQ(0u, ring.config() & MON_CAP_TUNE);
}

TEST(MonRing, TuneCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendTune(makeTune(), 1000);  // seq0=TUNE
  EXPECT_EQ(1u, f.ring.tuneCount());

  f.ring.appendTx(makeTx(), 1001);  // seq1=TX (ring full: [TUNE0, TX1])
  EXPECT_EQ(1u, f.ring.tuneCount());

  f.ring.appendTx(makeTx(), 1002);  // evicts seq0 -- the TUNE record itself
  EXPECT_EQ(0u, f.ring.tuneCount());
}

TEST(MonRing, AppendEventAssignsSeqAndRoundTripsFaultPayload) {
  RingFixture<8> f;
  f.ring.appendEvent(makeFaultEvent(EVENT_RX_START_TIMEOUT, /*cumulative=*/1), 1000);

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
  f.ring.appendSetting(makeSettingRecord(SETTING_TUNE_ENABLED, 0, 1, EVENT_SOURCE_BINARY), 1000);

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
  f.ring.appendCommand(makeCommandRecord(command_id), 1000);

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
                                        /*pkt_hash=*/0x2dac9975, /*age_ms=*/560), 1000);

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
                                        /*pkt_hash=*/0x11223344, /*age_ms=*/4200), 1000);

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
                                        /*pkt_hash=*/0xAABBCCDD, /*age_ms=*/9001), 1000);

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
                                        /*pkt_hash=*/0x99887766, /*age_ms=*/12000), 1000);

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
  f.ring.appendEvent(event, 1000);

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
    f.ring.appendEvent(event, 1000);

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
                                        /*pkt_hash=*/0x2dac9975, /*age_ms=*/3144), 1000);

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
  f.ring.appendEvent(makeOverflowEvent(EVENT_ACK_SUCCESS, shared_hash, 560), 1000);
  f.ring.appendEvent(makeOverflowEvent(EVENT_ECHO_SUCCESS, shared_hash, 3144), 1002);

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
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), 1000);
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
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), 1000);
  EXPECT_EQ(0u, f.ring.count());
  EXPECT_EQ(0u, f.ring.eventCount());

  // A type NOT excluded from the mask still captures normally.
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_CAD_TIMEOUT, 1), 1000);
  EXPECT_EQ(1u, f.ring.count());
  EXPECT_EQ(1u, f.ring.eventCount());
}

TEST(MonRing, EventCapIncludedInDefaultConfig) {
  // Unlike MON_CAP_ENV/MON_CAP_TUNE (opt-in), EVENT capture is on by
  // default -- fault history matters out of the box, not just for an
  // experimental feature someone has to remember to enable.
  MonRing ring;
  MonRecord buf[8];
  ring.init(reinterpret_cast<uint8_t *>(buf), sizeof(buf), 1000, RadioRecord{}, EnvRecord{});
  EXPECT_NE(0u, ring.config() & MON_CAP_EVENT);
}

TEST(MonRing, EventCountDecrementsOnEviction) {
  RingFixture<2> f;
  f.ring.appendEvent(makeFaultEvent(EVENT_TX_POOL_FULL, 1), 1000);  // seq0=EVENT
  EXPECT_EQ(1u, f.ring.eventCount());

  f.ring.appendTx(makeTx(), 1001);  // seq1=TX (ring full: [EVENT0, TX1])
  EXPECT_EQ(1u, f.ring.eventCount());

  f.ring.appendTx(makeTx(), 1002);  // evicts seq0 -- the EVENT record itself
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
