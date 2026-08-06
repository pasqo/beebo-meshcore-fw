#pragma once

#include <MeshCore.h>
#include <Identity.h>
#include <Packet.h>
#include <Utils.h>
#include <string.h>

namespace mesh {

/**
 * \brief  Abstraction of local/volatile clock with Millisecond granularity.
*/
class MillisecondClock {
public:
  virtual unsigned long getMillis() = 0;
};

/**
 * \brief  Abstraction of this device's packet radio.
*/
class Radio {
public:
  virtual void begin() { }

  /**
   * \brief  polls for incoming raw packet.
   * \param  bytes  destination to store incoming raw packet.
   * \param  sz   maximum packet size allowed.
   * \returns 0 if no incoming data, otherwise length of complete packet received.
  */
  virtual int recvRaw(uint8_t* bytes, int sz) = 0;

  /**
   * \returns  estimated transmit air-time needed for packet of 'len_bytes', in milliseconds.
  */
  virtual uint32_t getEstAirtimeFor(int len_bytes) = 0;

  virtual float packetScore(float snr, int packet_len) = 0;

  /**
   * \brief  starts the raw packet send. (no wait)
   * \param  bytes   the raw packet data
   * \param  len  the length in bytes
   * \returns true if successfully started
  */
  virtual bool startSendRaw(const uint8_t* bytes, int len) = 0;

  /**
   * \returns true if the previous 'startSendRaw()' completed successfully.
  */
  virtual bool isSendComplete() = 0;

  /**
   * \brief  a hook for doing any necessary clean up after transmit.
  */
  virtual void onSendFinished() = 0;

  /**
   * \brief  do any processing needed on each loop cycle
   */
  virtual void loop() { }

  virtual int getNoiseFloor() const { return 0; }

  virtual void triggerNoiseFloorCalibrate(int threshold) { }

  virtual void resetAGC() { }

  virtual bool isInRecvMode() const = 0;

  /**
   * \returns  true if the radio is currently mid-receive of a packet.
  */
  virtual bool isReceiving() { return false; }

  virtual float getLastRSSI() const { return 0; }
  virtual float getLastSNR() const { return 0; }
};

/**
 * \brief  An abstraction for managing instances of Packets (eg. in a static pool),
 *        and for managing the outbound packet queue.
*/
class PacketManager {
public:
  virtual Packet* allocNew() = 0;
  virtual void free(Packet* packet) = 0;

  // Returns false if the packet was dropped (queue full) instead of enqueued
  // -- the caller already freed it in that case, see StaticPoolPacketManager.
  virtual bool queueOutbound(Packet* packet, uint8_t priority, uint32_t scheduled_for) = 0;
  virtual Packet* getNextOutbound(uint32_t now) = 0;    // by priority
  virtual int getOutboundCount(uint32_t now) const = 0;
  virtual int getOutboundTotal() const = 0;
  virtual int getFreeCount() const = 0;
  virtual Packet* getOutboundByIdx(int i) = 0;
  virtual Packet* removeOutboundByIdx(int i) = 0;
  virtual bool queueInbound(Packet* packet, uint32_t scheduled_for) = 0;
  virtual Packet* getNextInbound(uint32_t now) = 0;

  // beebo: lifetime counts of queueOutbound()/queueInbound() silently
  // dropping an already-allocated packet because send_queue/rx_queue was
  // full (see StaticPoolPacketManager) -- a packet can reach here already
  // logged as RX_DISP_FORWARDED (the decision to relay is logged before the
  // queue is actually touched), so a rising getTxQueueFullCount() means that
  // disposition is, for some packets, optimistic rather than what actually
  // happened on the air.
  virtual uint32_t getTxQueueFullCount() const = 0;
  virtual uint32_t getRxQueueFullCount() const = 0;
};

typedef uint32_t  DispatcherAction;

#define ACTION_RELEASE           (0)
#define ACTION_MANUAL_HOLD       (1)
#define ACTION_RETRANSMIT(pri)   (((uint32_t)1 + (pri))<<24)
#define ACTION_RETRANSMIT_DELAYED(pri, _delay)  ((((uint32_t)1 + (pri))<<24) | (_delay))

#define ERR_EVENT_FULL              (1 << 0)
#define ERR_EVENT_CAD_TIMEOUT       (1 << 1)
#define ERR_EVENT_STARTRX_TIMEOUT   (1 << 2)

// beebo: reason a received frame was accepted or rejected along the RX chain,
// passed to logRxDisposition() at each decision site so the monitor ring can
// record *why* a frame was dropped. The ring maps each value onto one of three
// independent axes (endpoint / routing / distill), so multiple calls on the same
// frame accumulate rather than overwrite (see MonRing.h applyDisposition).
enum RxDisposition : uint8_t {
  RX_DISP_NONE         = 0,   // captured only, not yet resolved
  RX_DISP_ACCEPTED     = 1,   // decrypted / delivered to this node
  RX_DISP_FORWARDED    = 2,   // retransmit scheduled (repeater role)
  RX_DISP_DUP          = 3,   // already seen (hasSeen)
  RX_DISP_NOT_FOR_US   = 4,   // dest-hash mismatch, or decrypt/MAC failed
  RX_DISP_WRONG_HOP    = 5,   // direct route, this node is not the next hop
  RX_DISP_NO_FORWARD   = 6,   // forwarding disabled (client_repeat off)
  RX_DISP_FILTERED     = 7,   // filterRecvFloodPacket() rejected
  RX_DISP_INCOMPLETE   = 8,   // payload too short for its type
  RX_DISP_FORGED_SIG   = 9,   // advertisement signature verify failed
  RX_DISP_SELF         = 10,  // our own advert echoed back
  RX_DISP_UNKNOWN_TYPE = 11,  // unknown payload type
  RX_DISP_PARSE_ERR    = 12,  // bad version / path-mode / corrupt / oversize
  RX_DISP_POOL_FULL    = 13,  // no free packet buffer to receive into
  RX_DISP_DISTILL_BAD  = 14,  // ring couldn't distil the frame (distill rider bit)
  RX_DISP_PATH_FULL    = 15,  // wanted to relay but the path-hash buffer is full
  RX_DISP_MISC_DROP    = 16,  // dropped with no more specific reason (catch-all)
  RX_DISP_QUEUE_FULL   = 17,  // dropped after being scheduled RX_DISP_FORWARDED
};

/**
 * \brief  The low-level task that manages detecting incoming Packets, and the queueing
 *      and scheduling of outbound Packets.
*/
class Dispatcher {
  Packet* outbound;  // current outbound packet
  unsigned long outbound_expiry, outbound_start, total_air_time, rx_air_time;
  unsigned long next_tx_time;
  unsigned long cad_busy_start;
  uint32_t cad_busy_events;  // beebo: count of CAD-busy episodes (0->nonzero transitions of cad_busy_start), for the tuning optimizer's collision-proxy indicator
  unsigned long radio_nonrx_start;
  unsigned long next_floor_calib_time, next_agc_reset_time;
  bool  prev_isrecv_mode;
  uint32_t n_sent_flood, n_sent_direct;
  uint32_t n_recv_flood, n_recv_direct;
  unsigned long tx_budget_ms;
  unsigned long last_budget_update;
  unsigned long duty_cycle_window_ms;

  void processRecvPacket(Packet* pkt);
  void updateTxBudget();

protected:
  PacketManager* _mgr;
  Radio* _radio;
  MillisecondClock* _ms;
  uint16_t _err_flags;

  Dispatcher(Radio& radio, MillisecondClock& ms, PacketManager& mgr)
    : _radio(&radio), _ms(&ms), _mgr(&mgr)
  {
    outbound = NULL;
    total_air_time = rx_air_time = 0;
    next_tx_time = ms.getMillis();
    cad_busy_start = 0;
    cad_busy_events = 0;
    next_floor_calib_time = next_agc_reset_time = 0;
    _err_flags = 0;
    radio_nonrx_start = 0;
    prev_isrecv_mode = true;
    tx_budget_ms = 0;
    last_budget_update = 0;
    duty_cycle_window_ms = 3600000;
  }

  virtual DispatcherAction onRecvPacket(Packet* pkt) = 0;

  virtual void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) { }   // custom hook

  // beebo: stage a just-captured packet's distilled monitor-ring fields (call
  // right after logRxRaw ran), accumulate its accept/reject reason(s) at each
  // RX decision site, then commit one finished ring record once the packet's
  // disposition is fully resolved (called from processRecvPacket(), right
  // after onRecvPacket() returns). All three default to no-ops so upstream
  // builds are unaffected; only the RX_DISPOSITION build enables the wiring.
  virtual void onPacketCaptured(Packet* pkt) { }
  virtual void logRxDisposition(const Packet* pkt, uint8_t reason) { }
  virtual void onPacketDisposed(Packet* pkt) { }

  // beebo: immediate-emission hooks (called exactly at the point of failure,
  // every occurrence -- not diffed/polled) so a subclass can log a
  // discrete, causally-precise MonRing event without this base class
  // knowing anything about MonRing itself. All default to no-ops.
  //   logFaultEvent(bit): one of the ERR_EVENT_* bits (above) just fired --
  //     called every time, unlike _err_flags itself (sticky, set-once).
  //   logTxQueueFull(is_relay): a queueOutbound() call failed. is_relay is
  //     true from processRecvPacket() (a received packet's forward attempt
  //     -- already has a richer per-record trace, see RX_DISP_QUEUE_FULL),
  //     false from sendPacket() (a self-originated send/ACK reply, which has
  //     no other trace at all).
  //   logRxQueueFull(): a queueInbound() call failed (checkRecv()'s delayed-
  //     flood scheduling queue) -- fires before onRecvPacket() ever runs, so
  //     no disposition exists yet for this packet to attach information to.
  virtual void logFaultEvent(uint16_t bit) { }
  virtual void logTxQueueFull(bool is_relay) { }
  virtual void logRxQueueFull() { }

  virtual void logRx(Packet* packet, int len, float score) { }   // hooks for custom logging
  virtual void logTx(Packet* packet, int len) { }
  virtual void logTxFail(Packet* packet, int len) { }
  virtual const char* getLogDateTime() { return ""; }

  virtual float getAirtimeBudgetFactor() const;
  virtual int calcRxDelay(float score, uint32_t air_time) const;
  virtual uint32_t getCADFailRetryDelay() const;
  virtual uint32_t getCADFailMaxDuration() const;
  virtual int getInterferenceThreshold() const { return 0; }    // disabled by default
  virtual int getAGCResetInterval() const { return 0; }    // disabled by default
  virtual unsigned long getDutyCycleWindowMs() const { return 3600000; }

  // beebo: begin() starts the 8-second "stuck out of RX mode" watchdog
  // (radio_nonrx_start, checked in loop()) the moment the radio itself is
  // initialised -- fine for a subclass whose own begin() does nothing slow
  // afterward, but a subclass that does real work (extra flash reads, etc.)
  // between calling Dispatcher::begin() and actually reaching its own
  // loop() eats into that same 8-second budget before the radio has even
  // had a chance to be polled once. Letting such a subclass re-stamp the
  // clock once its own begin() work is fully done -- instead of trimming
  // that work to fit an unrelated radio-health budget -- keeps the two
  // concerns independent. No-op effect on any subclass that never calls it.
  void resetRxTimeoutClock() {
    radio_nonrx_start = _ms->getMillis();
    prev_isrecv_mode = _radio->isInRecvMode();
  }

public:
  void begin();
  void loop();

  Packet* obtainNewPacket();
  void releasePacket(Packet* packet);
  void sendPacket(Packet* packet, uint8_t priority, uint32_t delay_millis=0);

  unsigned long getTotalAirTime() const { return total_air_time; }
  unsigned long getReceiveAirTime() const {return rx_air_time; }
  uint32_t getCADBusyEventCount() const { return cad_busy_events; }
  unsigned long getRemainingTxBudget() const { return tx_budget_ms; }
  bool isTransmitting() const { return outbound != NULL; }  // beebo: for radioIsIdle()
  uint32_t getNumSentFlood() const { return n_sent_flood; }
  uint32_t getNumSentDirect() const { return n_sent_direct; }
  uint32_t getNumRecvFlood() const { return n_recv_flood; }
  uint32_t getNumRecvDirect() const { return n_recv_direct; }
  void resetStats() {
    n_sent_flood = n_sent_direct = n_recv_flood = n_recv_direct = 0;
    _err_flags = 0;
  }

  // helper methods
  bool millisHasNowPassed(unsigned long timestamp) const;
  unsigned long futureMillis(int millis_from_now) const;

  bool tryParsePacket(Packet* pkt, const uint8_t* raw, int len);

private:
  void checkRecv();
  void checkSend();
};

}
