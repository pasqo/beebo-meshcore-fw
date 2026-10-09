#include <gtest/gtest.h>
#include "helpers/SimpleMeshTables.h"
#include "helpers/MonRing.h"
#include <vector>

using namespace mesh;

// beebo: a fixed real-looking epoch, no sub-second precision (matches the
// default RTCClock::getTime()'s ms=0 fallback) -- just enough to tell
// "correctly scaled epoch-ms" apart from a bogus scale-confused one.
class FakeRTCClock : public RTCClock {
  uint32_t _secs;
public:
  explicit FakeRTCClock(uint32_t secs) : _secs(secs) {}
  uint32_t getCurrentTime() override { return _secs; }
  void setCurrentTime(uint32_t time) override { _secs = time; }
};

// Build a packet that calculatePacketHash() distinguishes by payload content.
// header selects ROUTE_TYPE_FLOOD so isRouteDirect() returns false.
static Packet makeFloodPacket(uint8_t seed) {
    Packet p;
    p.header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.payload[0] = seed;
    p.payload_len = 1;
    p.path_len = 0;
    return p;
}

static Packet makeDirectPacket(uint8_t seed) {
    Packet p;
    p.header = ROUTE_TYPE_DIRECT | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
    p.payload[0] = seed;
    p.payload_len = 1;
    p.path_len = 0;
    return p;
}

// ── wasSeen: pure query ───────────────────────────────────────────────────────

TEST(SimpleMeshTables, WasSeen_ReturnsFalseForUnseen) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    EXPECT_FALSE(t.wasSeen(&p));
}

// wasSeen shouldn't change state
TEST(SimpleMeshTables, WasSeen_IsPureQuery_DoesNotInsert) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    EXPECT_FALSE(t.wasSeen(&p));
    EXPECT_FALSE(t.wasSeen(&p));
}

// ── markSeen + wasSeen ───────────────────────────────────────────────────────

TEST(SimpleMeshTables, MarkSeen_MakesWasSeenReturnTrue) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    t.markSeen(&p);
    EXPECT_TRUE(t.wasSeen(&p));
}

TEST(SimpleMeshTables, MarkSeen_DoesNotAffectOtherPackets) {
    SimpleMeshTables t;
    Packet p1 = makeFloodPacket(0x01);
    Packet p2 = makeFloodPacket(0x02);
    t.markSeen(&p1);
    EXPECT_FALSE(t.wasSeen(&p2));
}

// Canonical pattern used at every onRecvPacket call site:
//   if (!wasSeen(pkt)) { markSeen(pkt); process(pkt); }
TEST(SimpleMeshTables, QueryThenMark_WorksCorrectly) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    EXPECT_FALSE(t.wasSeen(&p));
    t.markSeen(&p);
    EXPECT_TRUE(t.wasSeen(&p));
}

// ── dup stats ────────────────────────────────────────────────────────────────

TEST(SimpleMeshTables, WasSeen_IncrementsFloodDupStat) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    t.markSeen(&p);
    t.wasSeen(&p);
    EXPECT_EQ(1u, t.getNumFloodDups());
    EXPECT_EQ(0u, t.getNumDirectDups());
}

TEST(SimpleMeshTables, WasSeen_IncrementsDirectDupStat) {
    SimpleMeshTables t;
    Packet p = makeDirectPacket(0x01);
    t.markSeen(&p);
    t.wasSeen(&p);
    EXPECT_EQ(0u, t.getNumFloodDups());
    EXPECT_EQ(1u, t.getNumDirectDups());
}

// ── clear ────────────────────────────────────────────────────────────────────

TEST(SimpleMeshTables, Clear_RemovesSeenPacket) {
    SimpleMeshTables t;
    Packet p = makeFloodPacket(0x01);
    t.markSeen(&p);
    ASSERT_TRUE(t.wasSeen(&p));
    t.clear(&p);
    EXPECT_FALSE(t.wasSeen(&p));
}

// ── MonRing event emission (regression, 2026-09-18) ─────────────────────────
//
// beebo: _emitEchoEvent()/_emitEchoOverflowEvent()/markSeen()'s dedup-full
// path used to hand MonRing::appendEvent() the RTCClock's raw epoch
// SECONDS (_rtc->getCurrentTime()) where an already-resolved epoch-MS
// instant (_rtc->nowMillis()) was required -- misread as if it were a
// millis()-domain elapsed-time delta, this landed the resulting record (and
// the MON_SYNC relatch it forces) roughly `epoch_secs` MILLISECONDS in the
// future, ~21 days ahead on real hardware (gatto-mr, 2026-09-18). Verifies
// the dedup-table-full path (the same appendEvent() call, easiest of the
// three to trigger deterministically -- a 160-slot ring wraps back onto its
// own first slot on the 161st insert, still well within the default 10s
// dedup window since no real time passes in a fast test) resolves the
// governing SYNC to the fake clock's actual epoch second, not a wildly
// displaced one.
TEST(SimpleMeshTables, DedupTableFullEvent_TimestampMatchesRtcEpochNotScaleConfused) {
    SimpleMeshTables t;
    MonRecord buf[8];
    MonRing ring;
    ASSERT_TRUE(ring.init(reinterpret_cast<uint8_t*>(buf), sizeof(buf), 0, 0, RadioRecord{}, EnvRecord{}));
    ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
    FakeRTCClock clock(1758000000u);   // a real-looking epoch second
    t.setMonRing(&ring, &clock);

    for (int i = 0; i < MAX_PACKET_HASHES + 1; i++) {
        Packet p = makeFloodPacket((uint8_t)i);
        t.markSeen(&p);
    }

    MonRecord out[8];
    uint32_t returned = 0;
    ring.serialize(reinterpret_cast<uint8_t*>(out), sizeof(out), 0, &returned);
    ASSERT_GE(returned, 2u);
    MonRecord* sync_rec = nullptr;
    MonRecord* event_rec = nullptr;
    for (uint32_t i = 0; i < returned; i++) {
        if (out[i].kind == MON_SYNC) sync_rec = &out[i];
        if (out[i].kind == MON_EVENT && out[i].event.event_type == EVENT_RX_DEDUP_TABLE_FULL) {
            event_rec = &out[i];
        }
    }
    ASSERT_NE(nullptr, sync_rec);
    ASSERT_NE(nullptr, event_rec);
    EXPECT_EQ(1758000000u, sync_rec->sync.timestamp);
}

// ── echo retry (flood forwards) ──────────────────────────────────────────────

namespace {
struct RetryCall { uint8_t raw[MAX_TRANS_UNIT]; uint8_t len; uint8_t attempt; };
struct RetryProbe {
    int calls = 0;
    bool accept = true;
    RetryCall last;
};
bool retryHook(void* ctx, const uint8_t* raw, uint8_t len, uint8_t attempt) {
    RetryProbe* p = (RetryProbe*)ctx;
    p->calls++;
    p->last.len = len;
    p->last.attempt = attempt;
    memcpy(p->last.raw, raw, len);
    return p->accept;
}

const uint32_t AIRTIME_MS = 100;
// ECHO_TIMEOUT_BASE_MILLIS + airtime * ECHO_PERHOP_FACTOR + ECHO_PERHOP_EXTRA_MILLIS = 1350
const uint32_t PAST_WINDOW_MS = 2000;

struct RetryFixture : public ::testing::Test {
    SimpleMeshTables t;
    RetryProbe probe;
    uint8_t store[MAX_ECHO_HASHES * MAX_TRANS_UNIT];
    void SetUp() override {
        g_mock_millis = 1000;
        t.setRetryStore(store);
        t.setRetryHook(retryHook, &probe);
    }
    // what Mesh::routeRecvPacket() does for a forwarded flood
    void forward(Packet& p) {
        p._retryable = 1;                      // Mesh::routeRecvPacket() marks a forward retryable
        t.markSeen(&p);
        t.markSelfTx(&p, AIRTIME_MS);
    }
    void pastWindow() {
        g_mock_millis += PAST_WINDOW_MS;
        t.checkEchoTimeouts();
    }
    // the queued retry just went on the air: its echo window starts now
    void retrySent(Packet& p) {
        Packet r = p;
        r._tx_attempt = 1;                     // the packet the dispatcher just sent is the retry
        t.noteTx(&r, AIRTIME_MS, true);
    }
};
}

TEST_F(RetryFixture, RetryNoZero_TimeoutResolvesWithoutRetry) {
    Packet p = makeFloodPacket(0x10);
    forward(p);
    pastWindow();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());
}

TEST_F(RetryFixture, RetryNoOne_TimeoutFiresRetryWithSameBytes) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x11);
    forward(p);
    uint8_t expect[MAX_TRANS_UNIT];
    uint8_t expect_len = p.writeTo(expect);
    pastWindow();
    ASSERT_EQ(1, probe.calls);
    EXPECT_EQ(1, probe.last.attempt);
    ASSERT_EQ(expect_len, probe.last.len);
    EXPECT_EQ(0, memcmp(expect, probe.last.raw, expect_len));
    EXPECT_EQ(0u, t.getEchoTimeoutCount());   // not resolved yet
    EXPECT_EQ(1u, t.getEchoAttemptCount());   // a retry is the same confirmable packet
}

TEST_F(RetryFixture, RetryEchoed_CountsSuccessNotTimeout) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x12);
    forward(p);
    pastWindow();
    ASSERT_EQ(1, probe.calls);
    retrySent(p);
    g_mock_millis += 100;
    EXPECT_TRUE(t.wasSeen(&p));               // neighbor's echo of the retry
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoSuccessCount());
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());
}

TEST_F(RetryFixture, RetryExhausted_CountsTimeoutOnce) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x13);
    forward(p);
    pastWindow();
    retrySent(p);
    pastWindow();
    EXPECT_EQ(1, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
    EXPECT_EQ(0u, t.getEchoSuccessCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());
    pastWindow();                             // resolved: nothing more happens
    EXPECT_EQ(1, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, RetryNoTwo_RetriesTwiceWithIncreasingAttempt) {
    t.setRetryNo(2);
    Packet p = makeFloodPacket(0x14);
    forward(p);
    pastWindow();
    EXPECT_EQ(1, probe.last.attempt);
    retrySent(p);
    pastWindow();
    EXPECT_EQ(2, probe.last.attempt);
    retrySent(p);
    pastWindow();
    EXPECT_EQ(2, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());
}

TEST_F(RetryFixture, RetryNo_ClampedToThree) {
    t.setRetryNo(9);
    EXPECT_EQ(3, t.getRetryNo());
    t.setRetryNo(0);
    EXPECT_EQ(0, t.getRetryNo());
}

TEST_F(RetryFixture, EchoBeforeTimeout_NoRetry) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x15);
    forward(p);
    g_mock_millis += 200;
    EXPECT_TRUE(t.wasSeen(&p));
    pastWindow();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(1u, t.getEchoSuccessCount());
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, HookRefuses_ResolvesAsTimeout) {
    t.setRetryNo(1);
    probe.accept = false;                     // TX queue full
    Packet p = makeFloodPacket(0x16);
    forward(p);
    pastWindow();
    EXPECT_EQ(1, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());   // the refused retry never went out
    pastWindow();
    EXPECT_EQ(1, probe.calls);
}

TEST_F(RetryFixture, NoStore_NeverRetries) {
    SimpleMeshTables bare;                    // PSRAM allocation failed or not compiled in
    bare.setRetryHook(retryHook, &probe);
    bare.setRetryNo(1);
    Packet p = makeFloodPacket(0x17);
    bare.markSeen(&p);
    p._retryable = 1;
    bare.markSelfTx(&p, AIRTIME_MS);
    g_mock_millis += PAST_WINDOW_MS;
    bare.checkEchoTimeouts();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(1u, bare.getEchoTimeoutCount());
}

TEST_F(RetryFixture, DirectForward_NeverRetried) {
    t.setRetryNo(1);
    Packet p = makeDirectPacket(0x18);
    t.markSeen(&p);
    p._retryable = 1;
    t.markSelfTx(&p, AIRTIME_MS);
    pastWindow();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(0u, t.getEchoAttemptCount());
    EXPECT_EQ(1u, t.getSelfTxDirectCount());
}

// the echo events say which transmission they judge: data[9] = 0 for the first
// send, n for the nth retry
struct EventFixture : public RetryFixture {
    MonRecord buf[16];
    MonRing ring;
    FakeRTCClock clock{1758000000u};
    void SetUp() override {
        RetryFixture::SetUp();
        ASSERT_TRUE(ring.init(reinterpret_cast<uint8_t*>(buf), sizeof(buf), 0, 0, RadioRecord{}, EnvRecord{}));
        ring.setConfig(MON_CAP_ALL | MON_CAP_ENABLED);
        t.setMonRing(&ring, &clock);
        t.setRetryNo(2);
    }
    // (event_type, attempt) of every echo verdict event, oldest first
    std::vector<std::pair<int, int>> verdicts() {
        MonRecord out[16];
        uint32_t returned = 0;
        ring.serialize(reinterpret_cast<uint8_t*>(out), sizeof(out), 0, &returned);
        std::vector<std::pair<int, int>> v;
        for (uint32_t i = 0; i < returned; i++) {
            if (out[i].kind != MON_EVENT) continue;
            int type = out[i].event.event_type;
            if (type == EVENT_ECHO_SUCCESS || type == EVENT_ECHO_TIMEOUT) v.push_back({type, out[i].event.data[9]});
        }
        return v;
    }
};

// data[10:12] = the echo window (ms, u16 LE) the slot ran with
TEST_F(EventFixture, VerdictEventsCarryTheWindow) {
    auto windows = [&]() {
        MonRecord out[16];
        uint32_t returned = 0;
        ring.serialize(reinterpret_cast<uint8_t*>(out), sizeof(out), 0, &returned);
        std::vector<int> w;
        for (uint32_t i = 0; i < returned; i++) {
            if (out[i].kind != MON_EVENT) continue;
            int type = out[i].event.event_type;
            if (type == EVENT_ECHO_SUCCESS || type == EVENT_ECHO_TIMEOUT) {
                uint16_t v;
                memcpy(&v, &out[i].event.data[10], 2);
                w.push_back(v);
            }
        }
        return w;
    };
    Packet heard = makeFloodPacket(0x34);
    forward(heard);
    EXPECT_TRUE(t.wasSeen(&heard));            // success: 500 + 100 * 6 + 250
    t.setEchoFactor(9.0f);
    Packet lost = makeFloodPacket(0x35);
    t.setRetryNo(0);
    forward(lost);
    pastWindow();                              // timeout at the new factor: 500 + 900 + 250
    EXPECT_EQ((std::vector<int>{1350, 1650}), windows());
}

TEST_F(EventFixture, ARetryKeepsTheWindowOfItsFirstSend) {
    Packet p = makeFloodPacket(0x36);
    forward(p);
    pastWindow();
    t.setEchoFactor(15.0f);                    // changed while the retry waits
    retrySent(p);
    pastWindow();
    retrySent(p);
    pastWindow();
    MonRecord out[16];
    uint32_t returned = 0;
    ring.serialize(reinterpret_cast<uint8_t*>(out), sizeof(out), 0, &returned);
    for (uint32_t i = 0; i < returned; i++) {
        if (out[i].kind == MON_EVENT && out[i].event.event_type == EVENT_ECHO_TIMEOUT) {
            uint16_t v;
            memcpy(&v, &out[i].event.data[10], 2);
            EXPECT_EQ(1350, v);
        }
    }
}

TEST_F(EventFixture, FirstSendEchoed_EventAttemptZero) {
    Packet p = makeFloodPacket(0x30);
    forward(p);
    EXPECT_TRUE(t.wasSeen(&p));
    ASSERT_EQ(1u, verdicts().size());
    EXPECT_EQ(std::make_pair((int)EVENT_ECHO_SUCCESS, 0), verdicts()[0]);
}

TEST_F(EventFixture, RetryEchoed_EventAttemptOne_NoTimeoutEventForTheFirstSend) {
    Packet p = makeFloodPacket(0x31);
    forward(p);
    pastWindow();                              // first send unanswered -> retry 1 queued
    retrySent(p);
    EXPECT_TRUE(verdicts().empty());           // a retry in flight is not a verdict
    EXPECT_TRUE(t.wasSeen(&p));
    ASSERT_EQ(1u, verdicts().size());
    EXPECT_EQ(std::make_pair((int)EVENT_ECHO_SUCCESS, 1), verdicts()[0]);
}

TEST_F(EventFixture, RetriesExhausted_TimeoutEventCarriesLastAttempt) {
    Packet p = makeFloodPacket(0x32);
    forward(p);
    pastWindow();
    retrySent(p);
    pastWindow();
    retrySent(p);
    pastWindow();                              // retry 2 unanswered: final verdict
    ASSERT_EQ(1u, verdicts().size());
    EXPECT_EQ(std::make_pair((int)EVENT_ECHO_TIMEOUT, 2), verdicts()[0]);
}

TEST_F(EventFixture, NoRetry_TimeoutEventAttemptZero) {
    t.setRetryNo(0);
    Packet p = makeFloodPacket(0x33);
    forward(p);
    pastWindow();
    ASSERT_EQ(1u, verdicts().size());
    EXPECT_EQ(std::make_pair((int)EVENT_ECHO_TIMEOUT, 0), verdicts()[0]);
}

// the retry's echo window starts when it is on the air, not when it is queued
TEST_F(RetryFixture, QueuedRetry_HasNoWindowUntilItIsSent) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x40);
    forward(p);
    pastWindow();                             // retry handed to the queue
    ASSERT_EQ(1, probe.calls);
    g_mock_millis += 60000;                    // stuck behind other traffic for a minute
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());   // no window running yet
    retrySent(p);
    g_mock_millis += 1000;
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());   // inside the window that just started
    g_mock_millis += PAST_WINDOW_MS;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());   // and it ends one window after the send
}

TEST_F(RetryFixture, EchoOfTheFirstSendWhileTheRetryWaits_CountsSuccessForAttemptZero) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x41);
    forward(p);
    pastWindow();                             // retry queued, not sent
    g_mock_millis += 5000;
    EXPECT_TRUE(t.wasSeen(&p));               // a late echo of the first send
    EXPECT_EQ(1u, t.getEchoSuccessCount());
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, NoteTx_IgnoresPacketsThatAreNotWaitingForTheirRetry) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x42);
    forward(p);
    t.noteTx(&p, AIRTIME_MS, true);           // the first send going out
    g_mock_millis += 1000;
    Packet other = makeFloodPacket(0x43);
    other._tx_attempt = 1;
    t.noteTx(&other, AIRTIME_MS, true);       // a retry this node never tracked
    pastWindow();                             // the original window still ends on time
    EXPECT_EQ(1, probe.calls);
}

// ── echo slot at transmission (setEchoAtTx) ─────────────────────────────────

struct AtTxFixture : public RetryFixture {
    void SetUp() override {
        RetryFixture::SetUp();
        t.setEchoAtTx(true);
    }
    // what the dispatcher reports once the first send is on the air
    void sentOnAir(Packet& p) { t.noteTx(&p, AIRTIME_MS, true); }
};

TEST_F(AtTxFixture, ScheduledForward_HasNoSlotUntilItIsSent) {
    Packet p = makeFloodPacket(0x50);
    forward(p);                               // scheduled, still queued
    EXPECT_EQ(0u, t.getEchoAttemptCount());
    g_mock_millis += 3000;
    EXPECT_TRUE(t.wasSeen(&p));               // another repeater's forward, before ours went out
    EXPECT_EQ(0u, t.getEchoSuccessCount());   // not an echo of our transmission
    pastWindow();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());   // and nothing to time out
}

TEST_F(AtTxFixture, SentForward_StartsItsWindowAtTheSend) {
    Packet p = makeFloodPacket(0x51);
    forward(p);
    g_mock_millis += 60000;                   // waited a minute in the queue
    sentOnAir(p);
    EXPECT_EQ(1u, t.getEchoAttemptCount());
    g_mock_millis += 100;
    EXPECT_TRUE(t.wasSeen(&p));               // echo of the transmission
    EXPECT_EQ(1u, t.getEchoSuccessCount());
}

TEST_F(AtTxFixture, SentForward_UnansweredTimesOutOneWindowAfterTheSend) {
    Packet p = makeFloodPacket(0x52);
    forward(p);
    g_mock_millis += 60000;
    sentOnAir(p);
    g_mock_millis += 500;
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    pastWindow();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

// the window has the upstream one-hop baseline in both modes: 500 + airtime * 6 + 250 ms
// (100 ms airtime here); echo_at_tx only changes when it starts
TEST_F(AtTxFixture, WindowHasTheSameBaselineAsFromScheduling) {
    Packet p = makeFloodPacket(0x58);
    forward(p);
    sentOnAir(p);
    g_mock_millis += 1300;                     // 500 + 100 * 6 + 250 = 1350
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    g_mock_millis += 100;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

// the factor is a setting (setEchoFactor): the window is airtime * factor + 250 ms from the send,
// 500 ms more from scheduling. 100 ms airtime here.
TEST_F(AtTxFixture, WindowFollowsTheEchoFactor) {
    t.setEchoFactor(9.0f);
    Packet p = makeFloodPacket(0x5A);
    forward(p);
    sentOnAir(p);
    g_mock_millis += 1600;                     // 500 + 100 * 9 + 250 = 1650
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    g_mock_millis += 100;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, ScheduleTimeWindowFollowsTheEchoFactorToo) {
    t.setEchoFactor(4.0f);
    Packet p = makeFloodPacket(0x5B);
    forward(p);
    g_mock_millis += 1100;                     // 500 + 100 * 4 + 250 = 1150
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    g_mock_millis += 100;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(AtTxFixture, EchoFactorDefaultsToSixAndKeepsWhatItWasSet) {
    EXPECT_FLOAT_EQ(6.0f, t.getEchoFactor());
    t.setEchoFactor(11.5f);
    EXPECT_FLOAT_EQ(11.5f, t.getEchoFactor());
}

TEST_F(AtTxFixture, ALiveSlotKeepsTheWindowItWasStartedWith) {
    Packet p = makeFloodPacket(0x5C);
    forward(p);
    sentOnAir(p);                              // started at factor 6: 1350 ms
    t.setEchoFactor(20.0f);
    g_mock_millis += 1400;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, ScheduleTimeWindowKeepsTheBase) {
    Packet p = makeFloodPacket(0x59);
    forward(p);
    g_mock_millis += 1300;                     // 500 + 100 * 6 + 250 = 1350
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    g_mock_millis += 100;
    t.checkEchoTimeouts();
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(AtTxFixture, SendThatFailedToStart_NeverGetsASlot) {
    Packet p = makeFloodPacket(0x53);
    forward(p);
    t.noteTx(&p, AIRTIME_MS, false);
    pastWindow();
    EXPECT_EQ(0u, t.getEchoAttemptCount());
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
}

TEST_F(AtTxFixture, DirectPacket_NeverGetsASlot) {
    Packet p = makeDirectPacket(0x54);
    t.markSeen(&p);
    t.markSelfTx(&p, AIRTIME_MS);
    sentOnAir(p);
    EXPECT_EQ(0u, t.getEchoAttemptCount());
    EXPECT_EQ(1u, t.getSelfTxDirectCount());
}

TEST_F(AtTxFixture, OriginatedFlood_IsTrackedButNeverRetried) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x55);         // not marked retryable: we originated it
    t.markSeen(&p);
    t.markSelfTx(&p, AIRTIME_MS);
    sentOnAir(p);
    EXPECT_EQ(1u, t.getEchoAttemptCount());
    pastWindow();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(AtTxFixture, Forward_RetriedWithTheWindowStartingAtEachSend) {
    t.setRetryNo(1);
    Packet p = makeFloodPacket(0x56);
    forward(p);
    sentOnAir(p);
    pastWindow();                             // first send unanswered -> retry queued
    ASSERT_EQ(1, probe.calls);
    g_mock_millis += 30000;                   // retry waits in the queue
    t.checkEchoTimeouts();
    EXPECT_EQ(0u, t.getEchoTimeoutCount());
    retrySent(p);
    g_mock_millis += 100;
    EXPECT_TRUE(t.wasSeen(&p));
    EXPECT_EQ(1u, t.getEchoSuccessCount());
    EXPECT_EQ(1u, t.getEchoAttemptCount());   // one packet, however often retried
}

TEST_F(RetryFixture, ScheduleTimeSlot_IsTheDefault) {
    Packet p = makeFloodPacket(0x57);
    forward(p);                               // slot exists from scheduling
    EXPECT_EQ(1u, t.getEchoAttemptCount());
    t.noteTx(&p, AIRTIME_MS, true);           // the send changes nothing in this mode
    EXPECT_EQ(1u, t.getEchoAttemptCount());
}

TEST_F(RetryFixture, SelfOriginatedFlood_NeverRetried) {
    t.setRetryNo(1);                          // markSelfTx() alone: an advert or message we originated
    Packet p = makeFloodPacket(0x1B);
    t.markSeen(&p);
    t.markSelfTx(&p, AIRTIME_MS);
    pastWindow();
    EXPECT_EQ(0, probe.calls);
    EXPECT_EQ(1u, t.getEchoTimeoutCount());
}

TEST_F(RetryFixture, RetryNoChange_AffectsOnlyNewForwards) {
    t.setRetryNo(1);
    Packet p1 = makeFloodPacket(0x19);
    forward(p1);
    t.setRetryNo(0);                          // slot keeps the value it had when the packet was forwarded
    Packet p2 = makeFloodPacket(0x1A);
    forward(p2);
    pastWindow();
    EXPECT_EQ(1, probe.calls);                // only p1 retried
    EXPECT_EQ(1u, t.getEchoTimeoutCount());   // p2 resolved
}

TEST_F(RetryFixture, RetrySlotReusedByRingWrap_DoesNotRetryStaleBody) {
    t.setRetryNo(1);
    Packet first = makeFloodPacket(0x20);
    forward(first);
    for (int i = 1; i <= MAX_ECHO_HASHES; i++) {   // wraps onto first's slot
        Packet p = makeFloodPacket(0x20 + i);
        forward(p);
    }
    EXPECT_EQ(1u, t.getEchoOverflowCount());
    pastWindow();
    // every live slot retries its own body; the overwritten one never does
    EXPECT_EQ(MAX_ECHO_HASHES, probe.calls);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
