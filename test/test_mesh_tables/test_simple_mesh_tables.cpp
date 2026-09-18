#include <gtest/gtest.h>
#include "helpers/SimpleMeshTables.h"
#include "helpers/MonRing.h"

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

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
