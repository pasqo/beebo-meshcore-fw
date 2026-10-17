#include <gtest/gtest.h>
#include "Dispatcher.h"
#include "helpers/StaticPoolPacketManager.h"

using namespace mesh;

// beebo: a radio that stays out of RX mode is reported once when it passes
// 8 s and again every 8 s after that, not once per loop() call (about
// 1.4 kHz): one stuck radio filled the whole 65,536-record MonRing.

namespace {
struct FakeClock : public MillisecondClock {
    unsigned long now = 1000;
    unsigned long getMillis() override { return now; }
};

struct FakeRadio : public Radio {
    bool in_rx = true;
    int recvRaw(uint8_t*, int) override { return 0; }
    uint32_t getEstAirtimeFor(int) override { return 500; }
    float packetScore(float, int) override { return 0; }
    bool startSendRaw(const uint8_t*, int) override { return true; }
    bool isSendComplete() override { return false; }
    void onSendFinished() override {}
    bool isInRecvMode() const override { return in_rx; }
};

struct FakePool : public PacketManager {
    Packet pool;
    Packet* allocNew() override { return &pool; }
    void free(Packet*) override {}
    bool queueOutbound(Packet*, uint8_t, uint32_t) override { return true; }
    Packet* getNextOutbound(uint32_t) override { return nullptr; }
    int getOutboundCount(uint32_t) const override { return 0; }
    int getOutboundTotal() const override { return 0; }
    int getFreeCount() const override { return 1; }
    Packet* getOutboundByIdx(int) override { return nullptr; }
    Packet* removeOutboundByIdx(int) override { return nullptr; }
    bool queueInbound(Packet*, uint32_t) override { return false; }
    Packet* getNextInbound(uint32_t) override { return nullptr; }
    uint32_t getTxQueueFullCount() const override { return 0; }
    uint32_t getRxQueueFullCount() const override { return 0; }
};

struct TestDispatcher : public Dispatcher {
    int rx_start_events = 0;
    uint16_t errFlags() const { return _err_flags; }
    TestDispatcher(Radio& r, MillisecondClock& c, PacketManager& m) : Dispatcher(r, c, m) {}
    DispatcherAction onRecvPacket(Packet*) override { return ACTION_RELEASE; }
    float getAirtimeBudgetFactor() const override { return 0.0f; }
    void logFaultEvent(uint16_t bit) override {
        if (bit == ERR_EVENT_STARTRX_TIMEOUT) rx_start_events++;
    }
};

struct Fixture : public ::testing::Test {
    FakeClock clock;
    FakeRadio radio;
    FakePool pool;
    TestDispatcher d{radio, clock, pool};
    void SetUp() override { d.begin(); }
    // loop() every millisecond for ms milliseconds, as the main loop runs
    void run(unsigned long ms) {
        for (unsigned long t = 0; t < ms; t++) { clock.now += 1; d.loop(); }
    }
};
}

TEST_F(Fixture, InRx_NoEvent) {
    run(30000);
    EXPECT_EQ(0, d.rx_start_events);
}

TEST_F(Fixture, OutOfRxUnder8s_NoEvent) {
    radio.in_rx = false;
    run(7000);
    EXPECT_EQ(0, d.rx_start_events);
}

TEST_F(Fixture, OutOfRx_EventOnceAt8sThenEvery8s) {
    radio.in_rx = false;
    run(8500);
    EXPECT_EQ(1, d.rx_start_events);
    run(8000);   // 16.5 s
    EXPECT_EQ(2, d.rx_start_events);
    run(8000);   // 24.5 s
    EXPECT_EQ(3, d.rx_start_events);
}

TEST_F(Fixture, FaultFlagStaysSetWhileStuck) {
    radio.in_rx = false;
    run(8500);
    EXPECT_NE(0, d.errFlags() & ERR_EVENT_STARTRX_TIMEOUT);
}

TEST_F(Fixture, BackInRxThenStuckAgain_ReportsAgainAfter8s) {
    radio.in_rx = false;
    run(8500);
    EXPECT_EQ(1, d.rx_start_events);
    radio.in_rx = true;
    run(1000);
    radio.in_rx = false;
    run(7000);
    EXPECT_EQ(1, d.rx_start_events);   // a new episode: not yet 8 s
    run(1500);
    EXPECT_EQ(2, d.rx_start_events);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
