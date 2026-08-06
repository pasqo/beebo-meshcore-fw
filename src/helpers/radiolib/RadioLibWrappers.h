#pragma once

#include <Mesh.h>
#include <RadioLib.h>

class RadioLibWrapper : public mesh::Radio {
protected:
  PhysicalLayer* _radio;
  mesh::MainBoard* _board;
  uint32_t n_recv, n_sent, n_recv_errors;
  int16_t _noise_floor, _threshold;
  uint16_t _num_floor_samples;
  int32_t _floor_sample_sum;
  uint8_t _preamble_sf;

  void idle();
  void startRecv();
  float packetScoreInt(float snr, int sf, int packet_len);
  virtual bool isReceivingPacket() =0;
  virtual void doResetAGC();

  bool _cw_active = false;         // beebo: bench CW carrier engaged
  unsigned long _cw_start = 0;     // beebo: millis() when the carrier was keyed
  uint32_t _cw_max_ms = 0;         // beebo: auto-stop after this long (0 = no limit)
  bool _pa_optimize = true;        // beebo: RadioLib paOptTable optimization (SX1262 only)

public:
  RadioLibWrapper(PhysicalLayer& radio, mesh::MainBoard& board) : _radio(&radio), _board(&board), _preamble_sf(0) { n_recv = n_sent = 0; }

  void begin() override;
  virtual void powerOff() { _radio->sleep(); }
  // beebo: live counterpart to powerOff() -- wakes the chip back to
  // standby and resumes normal reception, same standby()+startRecv()
  // sequence stopCW() already uses to recover from its own special radio
  // state, plus resetAGC()'s noise-floor-sampling reset (stale after a
  // sleep period). No RadioLib exotica: SX126x auto-wakes on any SPI
  // transaction, and standby() is itself that transaction.
  void wake();
  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override;
  bool isChannelActive();

  bool isReceiving() override { 
    if (isReceivingPacket()) return true;

    return isChannelActive();
  }

  virtual void setParams(float freq, float bw, uint8_t sf, uint8_t cr) = 0;
  uint32_t getRngSeed();
  virtual void setTxPower(int8_t dbm);

  // beebo: select RadioLib's per-level PA optimization (paOptTable). true =
  // efficiency-optimized but non-monotonic at the low end; false = fixed PA
  // config (paDutyCycle=4,hpMax=7,paVal=dbm) for a monotonic sweep. SX1262 only.
  void setTxPowerOptimize(bool en) { _pa_optimize = en; }
  bool getTxPowerOptimize() const { return _pa_optimize; }

  virtual float getCurrentRSSI() =0;
  virtual uint8_t getSpreadingFactor() const { return LORA_SF; }
  static uint16_t preambleLengthForSF(uint8_t sf) { return sf <= 8 ? 32 : 16; }
  void updatePreamble(uint8_t sf) { _preamble_sf = sf; _radio->setPreambleLength(preambleLengthForSF(sf)); }

  int getNoiseFloor() const override { return _noise_floor; }
  void triggerNoiseFloorCalibrate(int threshold) override;
  void resetAGC() override;

  void loop() override;

  uint32_t getPacketsRecv() const { return n_recv; }
  uint32_t getPacketsRecvErrors() const { return n_recv_errors; }
  uint32_t getPacketsSent() const { return n_sent; }
  void resetStats() { n_recv = n_sent = n_recv_errors = 0; }

  virtual float getLastRSSI() const override;
  virtual float getLastSNR() const override;

  float packetScore(float snr, int packet_len) override { return packetScoreInt(snr, 10, packet_len); }  // assume sf=10

  virtual void setRxBoostedGainMode(bool) { }
  virtual bool getRxBoostedGainMode() const { return false; }

  // beebo: bench-test continuous-wave (unmodulated carrier) for spectrum/power
  // measurement. Keys the radio + external FEM at the configured freq/power;
  // the normal recv/send paths are suspended while active. max_ms bounds how
  // long the PA stays keyed (0 = until stopCW()) as a safety against an
  // abandoned session leaving the carrier on.
  void startCW(uint32_t max_ms = 0);
  void stopCW();
  bool isCWActive() const { return _cw_active; }
};

/**
 * \brief  an RNG impl using the noise from the LoRa radio as entropy.
 *         NOTE: this is VERY SLOW!  Use only for things like creating new LocalIdentity
*/
class RadioNoiseListener : public mesh::RNG {
  PhysicalLayer* _radio;
public:
  RadioNoiseListener(PhysicalLayer& radio): _radio(&radio) { }

  void random(uint8_t* dest, size_t sz) override {
    for (int i = 0; i < sz; i++) {
      dest[i] = _radio->randomByte() ^ (::random(0, 256) & 0xFF);
    }
  }
};
