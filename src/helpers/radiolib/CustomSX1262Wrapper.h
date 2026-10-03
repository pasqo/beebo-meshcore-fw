#pragma once

#include "CustomSX1262.h"
#include "RadioLibWrappers.h"
#include "SX126xReset.h"

#ifndef USE_SX1262
#define USE_SX1262
#endif

class CustomSX1262Wrapper : public RadioLibWrapper {
public:
  CustomSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomSX1262 *)_radio)->setFrequency(freq);
    ((CustomSX1262 *)_radio)->setSpreadingFactor(sf);
    ((CustomSX1262 *)_radio)->setBandwidth(bw);
    ((CustomSX1262 *)_radio)->setCodingRate(cr);
    _cfg_cr = cr;
    _tx_cr_active = false;
    updatePreamble(sf);
    PacketMillis pm = calcMaxPacketMillis(sf, bw, cr, preambleLengthForSF(sf));
    ((CustomSX1262 *)_radio)->setPreambleMillis(pm.preambleMillis);
    ((CustomSX1262 *)_radio)->setMaxPayloadMillis(pm.payloadMillis);
  }

  // beebo: per-packet TX coding rate (mesh::Radio::setTxCr()). The receiver
  // reads the CR from the explicit LoRa header, so a frame at another CR
  // decodes without any receiver change. One SetModulationParams command; the
  // configured CR is restored when the send ends or fails to start.
  void setTxCr(uint8_t cr) override {
    if (cr < 5 || cr > 8 || cr == _cfg_cr) return;
    if (((CustomSX1262 *)_radio)->setCodingRate(cr) == RADIOLIB_ERR_NONE) _tx_cr_active = true;
  }
  bool startSendRaw(const uint8_t* bytes, int len) override {
    bool ok = RadioLibWrapper::startSendRaw(bytes, len);
    if (!ok) restoreCr();
    return ok;
  }
  void onSendFinished() override {
    RadioLibWrapper::onSendFinished();
    restoreCr();
  }
  using RadioLibWrapper::getEstAirtimeFor;
  uint32_t getEstAirtimeFor(int len_bytes, uint8_t cr) override {
    if (cr < 5 || cr > 8 || cr == _cfg_cr) return RadioLibWrapper::getEstAirtimeFor(len_bytes);
    CustomSX1262* r = (CustomSX1262 *)_radio;
    DataRate_t dr = {};
    PacketConfig_t pc = {};
    dr.lora.codingRate = cr;
    dr.lora.spreadingFactor = r->spreadingFactor;
    dr.lora.bandwidth = r->bandwidthKhz;
    pc.lora.preambleLength = r->preambleLengthLoRa;
    pc.lora.crcEnabled = (bool)r->crcTypeLoRa;
    pc.lora.implicitHeader = r->headerType == RADIOLIB_SX126X_LORA_HEADER_IMPLICIT;
    pc.lora.ldrOptimize = (bool)r->ldrOptimize;
    return r->calculateTimeOnAir(RADIOLIB_MODEM_LORA, dr, pc, len_bytes) / 1000;
  }

  // beebo: honor the optimize flag so a bench sweep can compare RadioLib's
  // efficiency table (true) against a monotonic fixed-PA mapping (false).
  void setTxPower(int8_t dbm) override {
    ((CustomSX1262 *)_radio)->setOutputPower(dbm, _pa_optimize);
  }

  bool isReceivingPacket() override { 
    return ((CustomSX1262 *)_radio)->isReceiving();
  }
#ifdef BEEBO_RSSI_ANT_REF
  float getCurrentRSSI() override {
    return ((CustomSX1262 *)_radio)->getRSSI(false) + _rssi_offset;
  }
  float getLastRSSI() const override { return ((CustomSX1262 *)_radio)->getRSSI() + _rssi_offset; }
#else
  float getCurrentRSSI() override {
    return ((CustomSX1262 *)_radio)->getRSSI(false);
  }
  float getLastRSSI() const override { return ((CustomSX1262 *)_radio)->getRSSI(); }
#endif
  float getLastSNR() const override { return ((CustomSX1262 *)_radio)->getSNR(); }

  float packetScore(float snr, int packet_len) override {
    int sf = ((CustomSX1262 *)_radio)->spreadingFactor;
    return packetScoreInt(snr, sf, packet_len);
  }
  uint8_t getSpreadingFactor() const override { return ((CustomSX1262 *)_radio)->spreadingFactor; }
  virtual void powerOff() override {
    ((CustomSX1262 *)_radio)->sleep(false);
  }

  bool setRxBoostedGainMode(bool en) override {
    return ((CustomSX1262 *)_radio)->setRxBoostedGainMode(en) == RADIOLIB_ERR_NONE;
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomSX1262 *)_radio)->getRxBoostedGainMode();
  }

  void doResetAGC() override { sx126xResetAGC((SX126x *)_radio, getRxBoostedGainMode()); }

private:
  uint8_t _cfg_cr = 5;          // beebo: the CR setParams() last configured
  bool _tx_cr_active = false;   // beebo: radio is at a per-packet CR, not _cfg_cr
  void restoreCr() {
    if (!_tx_cr_active) return;
    ((CustomSX1262 *)_radio)->setCodingRate(_cfg_cr);
    _tx_cr_active = false;
  }
};
