#include <gtest/gtest.h>
#include <string>
#include "Dispatcher.h"
#include "helpers/StaticPoolPacketManager.h"

using namespace mesh;

// beebo: per-packet TX coding rate. Dispatcher::checkSend() sets the packet's
// own CR on the radio before startSendRaw() (only when nonzero), and times the
// send out against the airtime at that CR.

namespace {
struct FakeClock : public MillisecondClock {
    unsigned long now = 1000;
    unsigned long getMillis() override { return now; }
};

// airtime = 100 ms per CR step (cr 5 -> 500 ms), so a CR change is visible
struct FakeRadio : public Radio {
    std::string log;
    bool send_complete = false;
    int recvRaw(uint8_t*, int) override { return 0; }
    uint32_t getEstAirtimeFor(int) override { return 500; }
    uint32_t getEstAirtimeFor(int, uint8_t cr) override { return 100u * cr; }
    float packetScore(float, int) override { return 0; }
    void setTxCr(uint8_t cr) override { log += "cr" + std::to_string(cr) + ","; }
    bool startSendRaw(const uint8_t*, int) override { log += "start,"; return true; }
    bool isSendComplete() override { return send_complete; }
    void onSendFinished() override { log += "finish,"; }
    bool isInRecvMode() const override { return true; }
};

struct FakePool : public PacketManager {
    Packet pool;
    Packet* queued = nullptr;
    Packet* allocNew() override { return &pool; }
    void free(Packet*) override { queued = nullptr; }
    bool queueOutbound(Packet* p, uint8_t, uint32_t) override { queued = p; return true; }
    Packet* getNextOutbound(uint32_t) override { Packet* p = queued; queued = nullptr; return p; }
    int getOutboundCount(uint32_t) const override { return queued ? 1 : 0; }
    int getOutboundTotal() const override { return queued ? 1 : 0; }
    int getFreeCount() const override { return 1; }
    Packet* getOutboundByIdx(int) override { return nullptr; }
    Packet* removeOutboundByIdx(int) override { return nullptr; }
    bool queueInbound(Packet*, uint32_t) override { return false; }
    Packet* getNextInbound(uint32_t) override { return nullptr; }
    uint32_t getTxQueueFullCount() const override { return 0; }
    uint32_t getRxQueueFullCount() const override { return 0; }
};

struct TestDispatcher : public Dispatcher {
    TestDispatcher(Radio& r, MillisecondClock& c, PacketManager& m) : Dispatcher(r, c, m) {}
    DispatcherAction onRecvPacket(Packet*) override { return ACTION_RELEASE; }
    float getAirtimeBudgetFactor() const override { return 0.0f; }
};

struct Fixture : public ::testing::Test {
    FakeClock clock;
    FakeRadio radio;
    FakePool pool;
    TestDispatcher d{radio, clock, pool};
    void SetUp() override { d.begin(); }
    void send(uint8_t cr) {
        Packet* p = d.obtainNewPacket();
        p->header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
        p->payload_len = 1;
        p->payload[0] = 0x01;
        p->path_len = 0;
        p->_tx_cr = cr;
        d.sendPacket(p, 0);
        clock.now += 10;
        d.loop();
    }
};
}

TEST_F(Fixture, PacketCrZero_UsesConfiguredCr_NoRadioCrCall) {
    send(0);
    EXPECT_EQ("start,", radio.log);
}

TEST_F(Fixture, PacketCr_SetBeforeStartSend) {
    send(8);
    EXPECT_EQ("cr8,start,", radio.log);
}

TEST_F(Fixture, SendTimeout_UsesAirtimeAtPacketCr) {
    send(8);                                  // 800 ms * 3/2 = 1200 ms
    radio.log.clear();
    clock.now += 1000;                        // would have timed out at CR 5 (750 ms)
    d.loop();
    EXPECT_EQ("", radio.log);
    clock.now += 300;
    d.loop();
    EXPECT_EQ("finish,", radio.log.substr(0, 7));
}

TEST_F(Fixture, ObtainNewPacket_ClearsStaleTxCr) {
    pool.pool._tx_cr = 8;                     // pool reuses a packet that was sent at CR 8
    Packet* p = d.obtainNewPacket();
    EXPECT_EQ(0, p->_tx_cr);
}

// beebo: PacketQueue::get() starts its best-priority search at 0xFF and keeps
// only a strictly lower value, so a packet queued at priority 255 is never
// selected: it would sit in the queue forever, holding a pool slot. (A retry
// is queued at its hop count, far below this; the limit is pinned here.)
TEST(PacketQueuePriority, Priority255IsNeverSelected) {
    PacketQueue q(4);
    Packet p;
    ASSERT_TRUE(q.add(&p, 255, 0));
    EXPECT_EQ(nullptr, q.get(1000));
    EXPECT_EQ(1, q.count());                  // still stuck in the queue
}

TEST(PacketQueuePriority, Priority254IsStillSelectedAfterTheOthers) {
    PacketQueue q(4);
    Packet low, forward;
    ASSERT_TRUE(q.add(&low, 254, 0));
    ASSERT_TRUE(q.add(&forward, 3, 0));
    EXPECT_EQ(&forward, q.get(1000));
    EXPECT_EQ(&low, q.get(1000));
    EXPECT_EQ(0, q.count());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
