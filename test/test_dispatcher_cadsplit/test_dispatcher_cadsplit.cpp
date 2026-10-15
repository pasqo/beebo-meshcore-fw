#include <gtest/gtest.h>
#include "Dispatcher.h"
#include "helpers/StaticPoolPacketManager.h"

using namespace mesh;

// beebo: tx_wait_cad is split by the check behind each busy verdict (frame in
// progress, RSSI, hardware CAD), and the backoff a verdict sets is charged to
// that same check.

namespace {
struct FakeClock : public MillisecondClock {
    unsigned long now = 1000;
    unsigned long getMillis() override { return now; }
};

struct FakeRadio : public Radio {
    bool busy = false;
    uint8_t reason = BUSY_NONE;
    bool sent = false;
    int recvRaw(uint8_t*, int) override { return 0; }
    uint32_t getEstAirtimeFor(int) override { return 500; }
    float packetScore(float, int) override { return 0; }
    bool startSendRaw(const uint8_t*, int) override { sent = true; return true; }
    bool isSendComplete() override { return false; }
    void onSendFinished() override {}
    bool isInRecvMode() const override { return true; }
    bool isReceiving() override { return busy; }
    uint8_t getBusyReason() const override { return reason; }
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
    void SetUp() override {
        d.begin();
        Packet* p = d.obtainNewPacket();
        p->header = ROUTE_TYPE_FLOOD | (PAYLOAD_TYPE_ACK << PH_TYPE_SHIFT);
        p->payload_len = 1;
        p->payload[0] = 0x01;
        p->path_len = 0;
        d.sendPacket(p, 0);
    }
    void step(unsigned long ms) {
        clock.now += ms;
        d.loop();
    }
};
}

TEST_F(Fixture, NotReceiving_NoVerdictNoCadWait) {
    step(10);
    EXPECT_EQ(0u, d.getTxCadVerdicts());
    EXPECT_EQ(0u, d.getTxWaitCadMs());
    EXPECT_TRUE(radio.sent);
}

TEST_F(Fixture, FrameVerdict_ChargesPassAndBackoffToFrame) {
    radio.busy = true;
    radio.reason = BUSY_FRAME;
    step(10);                 // verdict, sets a 200 ms backoff
    step(100);                // still inside the backoff
    EXPECT_EQ(1u, d.getTxCadVerdicts());
    EXPECT_EQ(110u, d.getTxWaitCadMs());
    EXPECT_EQ(110u, d.getTxWaitCadFrameMs());
    EXPECT_EQ(0u, d.getTxWaitCadRssiMs());
    EXPECT_EQ(0u, d.getTxWaitCadHwMs());
}

TEST_F(Fixture, RssiVerdict_ChargesRssi) {
    radio.busy = true;
    radio.reason = BUSY_RSSI;
    step(10);
    step(100);
    EXPECT_EQ(110u, d.getTxWaitCadRssiMs());
    EXPECT_EQ(0u, d.getTxWaitCadFrameMs());
    EXPECT_EQ(0u, d.getTxWaitCadHwMs());
}

TEST_F(Fixture, HwVerdict_ChargesHw) {
    radio.busy = true;
    radio.reason = BUSY_HW;
    step(10);
    step(100);
    EXPECT_EQ(110u, d.getTxWaitCadHwMs());
    EXPECT_EQ(0u, d.getTxWaitCadFrameMs());
    EXPECT_EQ(0u, d.getTxWaitCadRssiMs());
}

TEST_F(Fixture, EachVerdictCharges_ItsOwnSourceAndTheSplitSumsToTotal) {
    radio.busy = true;
    radio.reason = BUSY_FRAME;
    step(10);                 // frame verdict
    radio.reason = BUSY_RSSI;
    step(250);                // backoff over: the next verdict is RSSI
    step(100);
    EXPECT_EQ(2u, d.getTxCadVerdicts());
    EXPECT_EQ(d.getTxWaitCadMs(),
              d.getTxWaitCadFrameMs() + d.getTxWaitCadRssiMs() + d.getTxWaitCadHwMs());
    EXPECT_GT(d.getTxWaitCadFrameMs(), 0u);
    EXPECT_GT(d.getTxWaitCadRssiMs(), 0u);
}

TEST_F(Fixture, AfterTx_BackoffIsNotChargedToTheLastVerdict) {
    radio.busy = true;
    radio.reason = BUSY_FRAME;
    step(10);
    radio.busy = false;
    step(250);                // backoff over, channel clear: sends
    ASSERT_TRUE(radio.sent);
    uint32_t frame = d.getTxWaitCadFrameMs();
    step(50);                 // nothing queued: no wait of any kind
    EXPECT_EQ(frame, d.getTxWaitCadFrameMs());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
