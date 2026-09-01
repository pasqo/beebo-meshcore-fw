#include "Beebo.h"

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>
#include <SHA256.h>  // beebo: rxlog-compatible packet hash for the monitor ring
#include <helpers/TransportLog.h>
#include <helpers/DebugLog.h>
#include <helpers/ProfileLog.h>
#include <helpers/BattTrend.h>
#include "BeeboProtocol.h"
#include "AdminSelfCommand.h"
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <esp_mac.h>  // beebo: esp_efuse_mac_get_default() for BEEBO_CMD_GET_BOARD_ID
#include <esp_coexist.h>  // beebo: esp_coex_preference_set() -- see applyTransportConfig()'s BLE-teardown branch
#include <esp_bt.h>  // beebo: esp_bt_controller_get_status() -- see applyTransportConfig()'s TLOG_BT_CONTROLLER_STATUS log call
#if defined(ESP32)
#include <WiFi.h>
#endif

// beebo: monitor ring sizing. This board has only ~2 MB PSRAM total (NOT the
// 16 MB flash). Size as a fraction of the PSRAM actually free when the ring is
// allocated (after transports are up), so it can never exceed the total and
// always leaves a comfortable amount free for WiFi/BLE/LWIP/general use.
#define MONRING_FREE_FRACTION 2               // take 1/N of free PSRAM (leave the rest)
#define MONRING_MAX_BYTES     (1024u * 1024)  // hard cap (~32k records)
#define MONRING_MIN_BYTES     (64u * 1024)    // below this, don't bother (~2k records)
#define MONRING_ALIGN_BYTES   4096            // round allocation down to a page-size multiple
#define SLOWSTAT_REFRESH_MS   60000u          // beebo: cadence to refresh cached FS usage + MCU temp
#define TUNE_TICK_INTERVAL_MS 300000u         // beebo: dynamic-tuning optimizer re-tune cadence (5 min)

#ifndef TCP_PORT
  #define TCP_PORT 5000
#endif
// beebo: a live ble<->tcp switch queues its OK reply on whichever transport
// carried the command, but that transport's send_queue is only drained by a
// *later* checkRecvFrame() call -- if we tear it down in the same loop()
// iteration that queued the reply, the reply (and the caller's confirmation)
// is lost and they just see a hard disconnect. Teardown is deferred until
// that transport's own isConnected() goes false (see
// checkTransportsAndBoard() below), not a fixed delay.

// beebo: battery charge-trend state machine (see loop()'s resample block);
// BATT_STATE_*/BATT_SAMPLE_PERIOD_DEFAULT_SECS/classifyBattTrend() are shared with
// simple_repeater via helpers/BattTrend.h so the two firmwares can't drift.

void Beebo::writeOKFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_OK;
  _serial->writeFrame(buf, 1);
}
void Beebo::writeErrFrame(uint8_t err_code) {
  uint8_t buf[2];
  buf[0] = RESP_CODE_ERR;
  buf[1] = err_code;
  _serial->writeFrame(buf, 2);
}

void Beebo::writeDisabledFrame() {
  uint8_t buf[1];
  buf[0] = RESP_CODE_DISABLED;
  _serial->writeFrame(buf, 1);
}

void Beebo::writeContactRespFrame(uint8_t code, const ContactInfo &contact) {
  int i = 0;
  out_frame[i++] = code;
  memcpy(&out_frame[i], contact.id.pub_key, PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  out_frame[i++] = contact.type;
  out_frame[i++] = contact.flags;
  out_frame[i++] = contact.out_path_len;
  memcpy(&out_frame[i], contact.out_path, MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  StrHelper::strzcpy((char *)&out_frame[i], contact.name, 32);
  i += 32;
  memcpy(&out_frame[i], &contact.last_advert_timestamp, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lat, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.gps_lon, 4);
  i += 4;
  memcpy(&out_frame[i], &contact.lastmod, 4);
  i += 4;
  _serial->writeFrame(out_frame, i);
}

void Beebo::updateContactFromFrame(ContactInfo &contact, uint32_t& last_mod, const uint8_t *frame, int len) {
  int i = 0;
  uint8_t code = frame[i++]; // eg. CMD_ADD_UPDATE_CONTACT
  memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE);
  i += PUB_KEY_SIZE;
  contact.type = frame[i++];
  contact.flags = frame[i++];
  contact.out_path_len = frame[i++];
  memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE);
  i += MAX_PATH_SIZE;
  memcpy(contact.name, &frame[i], 32);
  i += 32;
  memcpy(&contact.last_advert_timestamp, &frame[i], 4);
  i += 4;
  if (len >= i + 8) { // optional fields
    memcpy(&contact.gps_lat, &frame[i], 4);
    i += 4;
    memcpy(&contact.gps_lon, &frame[i], 4);
    i += 4;
    if (len >= i + 4) {
      memcpy(&last_mod, &frame[i], 4);
    }
  }
}

bool Beebo::Frame::isChannelMsg() const {
  return buf[0] == RESP_CODE_CHANNEL_MSG_RECV || buf[0] == RESP_CODE_CHANNEL_MSG_RECV_V3 ||
         buf[0] == RESP_CODE_CHANNEL_DATA_RECV;
}

void Beebo::addToOfflineQueue(const uint8_t frame[], int len) {
  if (offline_queue_len >= OFFLINE_QUEUE_SIZE) {
    MESH_DEBUG_PRINTLN("WARN: offline_queue is full!");
    int pos = 0;
    while (pos < offline_queue_len) {
      if (offline_queue[pos].isChannelMsg()) {
        for (int i = pos; i < offline_queue_len - 1; i++) { // delete oldest channel msg from queue
          offline_queue[i] = offline_queue[i + 1];
        }
        MESH_DEBUG_PRINTLN("INFO: removed oldest channel message from queue.");
        offline_queue[offline_queue_len - 1].len = len;
        memcpy(offline_queue[offline_queue_len - 1].buf, frame, len);
        return;
      }
      pos++;
    }
    MESH_DEBUG_PRINTLN("INFO: no channel messages to remove from queue.");
  } else {
    offline_queue[offline_queue_len].len = len;
    memcpy(offline_queue[offline_queue_len].buf, frame, len);
    offline_queue_len++;
  }
}

int Beebo::getFromOfflineQueue(uint8_t frame[]) {
  if (offline_queue_len > 0) {         // check offline queue
    size_t len = offline_queue[0].len; // take from top of queue
    memcpy(frame, offline_queue[0].buf, len);

    offline_queue_len--;
    for (int i = 0; i < offline_queue_len; i++) { // delete top item from queue
      offline_queue[i] = offline_queue[i + 1];
    }
    return len;
  }
  return 0; // queue is empty
}

// beebo: airtime_factor is one of the 14 SharedPrefs fields (see
// NodePrefs.h) -- one physical storage slot in _role_state->prefs regardless of
// role, so no role branch is needed (companion and repeater share this field
// cleanup).
float Beebo::getAirtimeBudgetFactor() const {
  return _role_state->prefs.airtime_factor;
}

int Beebo::getInterferenceThreshold() const {
  return 0; // disabled for now, until currentRSSI() problem is resolved
}

// beebo: Dispatcher's default
// (0, disabled) was never overridden here at all. Repeater role uses the
// RAM-cached _role_state->prefs.agc_reset_interval (see loadRepeaterFwdPrefs()); companion
// keeps the prior implicit default.
int Beebo::getAGCResetInterval() const {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) return ((int)_role_state->prefs.agc_reset_interval) * 4000;  // milliseconds
#endif
  return 0;
}

// beebo: rx_delay_base is one of the 14 SharedPrefs fields (see
// NodePrefs.h) -- no role branch needed since both roles share the field.
int Beebo::calcRxDelay(float score, uint32_t air_time) const {
  float rx_delay_base = _role_state->prefs.rx_delay_base;
  if (rx_delay_base <= 0.0f) return 0;
  return (int)((pow(rx_delay_base, 0.85f - score) - 1.0) * air_time);
}

// beebo: these two hardcoded
// companion_radio's own constants (0.5f/0.2f) unconditionally, so
// repeater.txdelay/repeater.direct.txdelay round-tripped through the CLI
// but had zero effect on real retransmit timing. Repeater role now uses the
// RAM-cached _role_state->prefs.tx_delay_factor/com_prefs.direct_tx_delay_factor (see
// loadRepeaterFwdPrefs()); companion keeps its prior hardcoded values
// unchanged.
uint32_t Beebo::getRetransmitDelay(const mesh::Packet *packet) {
#if BEEBO_ENABLE_REPEATER_ROLE
  float factor = isRepeater() ? _role_state->prefs.tx_delay_factor : 0.5f;
#else
  float factor = 0.5f;
#endif
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * factor);
  return getRNG()->nextInt(0, 5*t + 1);
}
uint32_t Beebo::getDirectRetransmitDelay(const mesh::Packet *packet) {
#if BEEBO_ENABLE_REPEATER_ROLE
  float factor = isRepeater() ? _role_state->prefs.direct_tx_delay_factor : 0.2f;
#else
  float factor = 0.2f;
#endif
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * factor);
  return getRNG()->nextInt(0, 5*t + 1);
}

// beebo: multi_acks is one of the 14 SharedPrefs fields (see NodePrefs.h)
// -- no role branch needed since both roles share the field.
uint8_t Beebo::getExtraAckTransmitCount() const {
  return _role_state->prefs.multi_acks;
}

void Beebo::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
  _last_radio_active_ms = _ms->getMillis();  // beebo: feeds isIdle()'s settle-margin gate
  // beebo: distill the raw frame into the fields rxlog lets a companion recover
  // — never the message body — so a read reproduces rxlog's view offline. No
  // Packet exists yet, so this only stages into _rx_stage; onPacketCaptured()
  // copies it onto the Packet, and the finished record is committed (once,
  // after disposition is fully known) by onPacketDisposed(). All frame walking
  // is bounds-checked against len so a corrupt frame can't over-read.
  //
  // The route/hop-count/sender-identity walk below always runs (len>0 is the
  // only gate) — it feeds two independent consumers: MonRing's RxRecord
  // capture (rxlog, still gated on monring.enabled()/allocated() below) and,
  // new, the direct-neighbour table (putNeighbour(), unconditional). A
  // neighbour sighting shouldn't disappear just because rxlog capture happens
  // to be off.
  bool want_monring = monring.enabled() && monring.allocated() && len > 0;
  const uint8_t *idsrc = nullptr;
  uint8_t idsrc_len = 0;
  uint8_t idsrc_payload_type = 0xFF;
  uint8_t hops = 0;
  if (len > 0) {
    RxRecord &rec = _rx_stage;
    memset(&rec, 0, sizeof(rec));
    rec.snr = (int8_t)(snr * 4);
    rec.rssi = (int8_t)rssi;
    rec.header = raw[0];
    // RX-gain state now lives in the RADIO record (monring.noteRadio), not here.

    // Walk header -> [transport codes] -> path_byte -> path -> payload. If any
    // bounds check below fails we still keep the record but flag it INVALID: we
    // couldn't distill route/nbr/hash so the node can't use it, and we don't claim
    // to know why (corruption, foreign LoRa traffic, or truncation).
    bool parsed = false;
    uint8_t route = raw[0] & 0x03;  // PH_ROUTE_MASK
    int off = 1;
    if (route == ROUTE_TYPE_TRANSPORT_FLOOD || route == ROUTE_TYPE_TRANSPORT_DIRECT)
      off += 4;  // transport codes present
    if (off < len) {
      uint8_t path_byte = raw[off++];
      uint8_t hash_size = ((path_byte >> 6) & 0x03) + 1;
      hops = path_byte & 0x3F;
      int path_bytes = (int)hops * hash_size;
      if (off + path_bytes <= len) {
        parsed = true;
        rec.flags |= (hops << RXREC_FLAG_HOPS_SHIFT) & RXREC_FLAG_HOPS_MASK;
        if (hops > 0) {
          // Immediate neighbour = last hop's hash (last hash_size path bytes).
          uint8_t nlen = hash_size < 3 ? hash_size : 3;
          memcpy(rec.nbr, raw + off + path_bytes - nlen, nlen);
          rec.flags |= (nlen & RXREC_FLAG_NBRLEN_MASK);
          // beebo: whoever transmitted the last hop to us is a genuine direct
          // RF neighbour, regardless of how many hops the packet has
          // travelled overall or what it's carrying -- this is the dominant
          // real-world neighbour sighting (any relayed flood/direct traffic),
          // unlike the hops==0 payload-embedded-identity case below. Uses the
          // full hash_size (1-4 bytes), not the 3-byte cap the MonRing
          // rec.nbr display field above is limited to.
          putNeighbour(raw + off + path_bytes - hash_size, hash_size, 0,
                       (int8_t)(snr * 4), 0xFF, NULL, 0, 0);
        }

        off += path_bytes;
        int payload_len = len - off;
        if (hops == 0) {
          // No path hash to identify the sender by — heard directly, before
          // any relay appended one. Fall back to an identity carried in the
          // payload itself, same heuristic as the host-side rxlog analysis
          // (_relay_hash): an advert's own public key, the src hash a direct
          // request/response/text-message/path prefixes itself with, the
          // ephemeral pubkey an anon request prefixes itself with, or (for a
          // NODE_DISCOVER_RESP control reply) the responder's own pubkey --
          // same identity putNeighbour() reads out of this exact reply in
          // onControlDataRecv().
          uint8_t payload_type = (raw[0] & 0x3C) >> 2;  // PH_TYPE_MASK/SHIFT
          uint8_t nlen = 0;
          if (payload_type == PAYLOAD_TYPE_ADVERT && payload_len > 0) {
            idsrc = raw + off;                    // advert: originator = itself
            nlen = payload_len < 3 ? payload_len : 3;
          } else if ((payload_type == PAYLOAD_TYPE_REQ ||
                      payload_type == PAYLOAD_TYPE_RESPONSE ||
                      payload_type == PAYLOAD_TYPE_TXT_MSG ||
                      payload_type == PAYLOAD_TYPE_PATH) && payload_len > 1) {
            idsrc = raw + off + 1;                // direct datagram/path: src hash
            nlen = 1;
          } else if (payload_type == PAYLOAD_TYPE_ANON_REQ && payload_len > 1) {
            idsrc = raw + off + 1;                // anon request: sender's ephemeral pubkey
            nlen = (payload_len - 1) < 3 ? (payload_len - 1) : 3;
          } else if (payload_type == PAYLOAD_TYPE_CONTROL
                     && (route == ROUTE_TYPE_DIRECT || route == ROUTE_TYPE_TRANSPORT_DIRECT)
                     && payload_len > 6 && (raw[off] & 0xF0) == 0x90 /* NODE_DISCOVER_RESP */) {
            idsrc = raw + off + 6;                // discover response: responder's pubkey
            nlen = (payload_len - 6) < 3 ? (payload_len - 6) : 3;
          }
          idsrc_len = nlen;
          idsrc_payload_type = payload_type;
          if (idsrc) {
            rec.flags |= (nlen & RXREC_FLAG_NBRLEN_MASK);
            if (want_monring) memcpy(rec.nbr, idsrc, nlen);
          }
        }

        if (want_monring) {
          // pkt_hash = SHA256(payload)[0:4] LE, matching rxlog (payload only).
          SHA256 sha;
          sha.update(raw + off, len - off);
          uint8_t digest[32];
          sha.finalize(digest, sizeof(digest));
          rec.pkt_hash = (uint32_t)digest[0] | ((uint32_t)digest[1] << 8) |
                         ((uint32_t)digest[2] << 16) | ((uint32_t)digest[3] << 24);
        }
      }
    }
    if (want_monring) {
      // Seed the disposition: set the distill rider bit if we couldn't distill the
      // frame; the endpoint/routing axes stay NONE for the RX chain to fill in
      // later via logRxDisposition().
      if (!parsed) rec.disp |= RXREC_DISTILL;
      // beebo: close out any stale radio/env epoch before this capture
      monring.noteRadio(buildRadioRecord(), (uint32_t)getRTCClock()->getCurrentTime());
      monring.sampleEnv(buildEnvRecord(), (uint32_t)getRTCClock()->getCurrentTime());
    }
  }
  _rx_staged = want_monring;

  // beebo: any zero-hop RX naming a sender is a direct-neighbour sighting,
  // not just an advert. ADVERT and the NODE_DISCOVER_RESP CONTROL case are
  // excluded here: onAdvertRecv()/onControlDataRecv() already call
  // putNeighbour() for those, once the packet is fully parsed, with a better
  // (full-pubkey, for adverts) identity than this raw-frame heuristic gets --
  // adding them here would just be a redundant short-prefix write immediately
  // superseded by the real one. Every other zero-hop-identified type
  // (REQ/RESPONSE/TXT_MSG/PATH src hash, ANON_REQ ephemeral key) never gets
  // more than a 1-3 byte prefix, so this is their only source. putNeighbour()
  // itself handles matching an existing slot (refreshing heard_timestamp/SNR)
  // vs. creating a new one, and a later full-pubkey advert always upgrades a
  // short prefix recorded here in place.
  if (idsrc && hops == 0 && idsrc_payload_type != PAYLOAD_TYPE_ADVERT &&
      idsrc_payload_type != PAYLOAD_TYPE_CONTROL) {
    putNeighbour(idsrc, idsrc_len, 0, (int8_t)(snr * 4), 0xFF, NULL, 0, 0);
  }

  if (_serial->isConnected() && len + 3 <= MAX_FRAME_SIZE) {
    int i = 0;
    out_frame[i++] = PUSH_CODE_LOG_RX_DATA;
    out_frame[i++] = (int8_t)(snr * 4);
    out_frame[i++] = (int8_t)(rssi);
    memcpy(&out_frame[i], raw, len);
    i += len;

    _serial->writeFrame(out_frame, i);
  }
}

void Beebo::onPacketCaptured(mesh::Packet* pkt) {
  // beebo: transfer logRxRaw's staged fields onto the just-allocated packet, so
  // they survive untouched until disposition is fully known (onPacketDisposed),
  // even across a queued flood packet's delay wait during which other frames
  // get captured into _rx_stage.
#ifdef RX_DISPOSITION
  pkt->_rx_logged = _rx_staged;
  if (_rx_staged) {
    pkt->_rx_hash = _rx_stage.pkt_hash;
    pkt->_rx_time = (uint32_t)getRTCClock()->getCurrentTime();
    pkt->_rx_rssi = _rx_stage.rssi;
    pkt->_rx_flags = _rx_stage.flags;
    memcpy(pkt->_rx_nbr, _rx_stage.nbr, sizeof(pkt->_rx_nbr));
    pkt->_rx_disp = _rx_stage.disp;
  }
#endif
}

void Beebo::logRxDisposition(const mesh::Packet* pkt, uint8_t reason) {
  // beebo: RX_DISP_POOL_FULL/RX_DISP_PARSE_ERR fire before a finished Packet
  // exists to build a ring record from (no allocation, or freed immediately),
  // so they're tracked as a lifetime counter plus a dedicated EVENT_RX_
  // POOL_FULL/EVENT_RX_PARSE_ERROR trail (bumpRxDropCount()) instead --
  // separate event types, not a shared one with a reason byte, since these
  // are two different causes (radio/allocation capacity vs. a malformed
  // frame). Every other reason (including RX_DISP_QUEUE_FULL) accumulates
  // onto the packet's own disp byte; onPacketDisposed() commits it once.
#ifdef RX_DISPOSITION
  if (pkt == NULL || reason == mesh::RX_DISP_PARSE_ERR) {
    uint32_t cum = monring.bumpRxDropCount(reason);
    EventRecord rec{};
    rec.event_type = (reason == mesh::RX_DISP_PARSE_ERR) ? EVENT_RX_PARSE_ERROR : EVENT_RX_POOL_FULL;
    memcpy(&rec.data[0], &cum, 4);
    monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
  } else {
    MonRing::applyDisposition(const_cast<mesh::Packet*>(pkt)->_rx_disp, reason);
  }
#endif
}

void Beebo::onPacketDisposed(mesh::Packet* pkt) {
  // beebo: disposition is now fully resolved (called right after onRecvPacket()
  // returns) — commit the one finished record for this packet, if logRxRaw
  // actually staged capture data for it.
#ifdef RX_DISPOSITION
  if (pkt->_rx_logged) {
    RxRecord rec{};
    rec.pkt_hash = pkt->_rx_hash;
    rec.snr = pkt->_snr;
    rec.rssi = pkt->_rx_rssi;
    rec.header = pkt->header;
    rec.flags = pkt->_rx_flags;
    rec.disp = pkt->_rx_disp;
    memcpy(rec.nbr, pkt->_rx_nbr, sizeof(rec.nbr));
    monring.appendRx(rec, pkt->_rx_time);
  }
#endif
}

static void fillTxRecordCommon(TxRecord& rec, mesh::Packet* pkt, int len, mesh::Radio* radio) {
  // beebo: shared TX record fill for logTx/logTxFail — pkt_hash uses the same
  // payload-only SHA256 scheme as the RX capture so records correlate across
  // the rx/tx path (see Beebo::logRxRaw). Airtime is estimated rather than
  // measured: it's a closed-form function of sf/bw/cr/len, so an estimate is
  // as accurate as a wall-clock measurement and needs no extra timing plumbing.
  memset(&rec, 0, sizeof(rec));
  rec.header = pkt->header;
  rec.airtime_ms = (uint16_t)radio->getEstAirtimeFor(len);

  SHA256 sha;
  sha.update(pkt->payload, pkt->payload_len);
  uint8_t digest[32];
  sha.finalize(digest, sizeof(digest));
  rec.pkt_hash = (uint32_t)digest[0] | ((uint32_t)digest[1] << 8) |
                 ((uint32_t)digest[2] << 16) | ((uint32_t)digest[3] << 24);

  if (pkt->isRouteDirect() && pkt->getPathHashCount() > 0) {
    uint8_t hash_size = pkt->getPathHashSize();
    uint8_t nlen = hash_size < 3 ? hash_size : 3;
    memcpy(rec.dst, pkt->path, nlen);  // next hop = first hash in the path
  }
}

void Beebo::logTx(mesh::Packet* pkt, int len) {
  _last_radio_active_ms = _ms->getMillis();  // beebo: feeds isIdle()'s settle-margin gate
  if (monring.enabled() && monring.allocated() && len > 0) {
    TxRecord rec;
    fillTxRecordCommon(rec, pkt, len, _radio);
    rec.result = TXR_OK;
    // beebo: close out any stale radio/env epoch before this capture
    monring.noteRadio(buildRadioRecord(), (uint32_t)getRTCClock()->getCurrentTime());
    monring.sampleEnv(buildEnvRecord(), (uint32_t)getRTCClock()->getCurrentTime());
    monring.appendTx(rec, (uint32_t)getRTCClock()->getCurrentTime());
  }
}

void Beebo::logTxFail(mesh::Packet* pkt, int len) {
  _last_radio_active_ms = _ms->getMillis();  // beebo: feeds isIdle()'s settle-margin gate
  if (monring.enabled() && monring.allocated() && len > 0) {
    TxRecord rec;
    fillTxRecordCommon(rec, pkt, len, _radio);
    rec.result = TXR_TIMEOUT;
    // beebo: close out any stale radio/env epoch before this capture
    monring.noteRadio(buildRadioRecord(), (uint32_t)getRTCClock()->getCurrentTime());
    monring.sampleEnv(buildEnvRecord(), (uint32_t)getRTCClock()->getCurrentTime());
    monring.appendTx(rec, (uint32_t)getRTCClock()->getCurrentTime());
  }
}

// beebo: see MonRing.h's
// EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT comment for the data[] layout. SUCCESS/
// TIMEOUT only -- see emitAckOverflowEvent() for the resource-exhaustion case.
void Beebo::emitAckResultEvent(uint8_t verdict, uint32_t pkt_hash, uint32_t age_ms) {
  if (!monring.enabled() || !monring.allocated()) return;
  EventRecord rec{};
  rec.event_type = (verdict == TXCONFIRM_SUCCESS) ? EVENT_ACK_SUCCESS : EVENT_ACK_TIMEOUT;
  memcpy(&rec.data[1], &pkt_hash, 4);
  memcpy(&rec.data[5], &age_ms, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

// beebo: DoS/QoS audit -- resource-exhaustion fault, see MonRing.h's
// EVENT_ACK_OVERFLOW comment. Same pkt_hash/age_ms shape as
// emitAckResultEvent() but no verdict byte (the event type itself says what
// happened).
void Beebo::emitAckOverflowEvent(uint32_t pkt_hash, uint32_t age_ms) {
  if (!monring.enabled() || !monring.allocated()) return;
  EventRecord rec{};
  rec.event_type = EVENT_ACK_OVERFLOW;
  memcpy(&rec.data[1], &pkt_hash, 4);
  memcpy(&rec.data[5], &age_ms, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

// beebo: record/refresh a direct neighbour (evict least-recently-heard on
// overflow). Matches an existing slot by prefix over the shorter of the two
// lengths, searching every slot for the MOST SPECIFIC match rather than
// stopping at the first one found (see the specificity comment in the loop
// below). The SNR-drift check (NEIGHBOUR_SNR_DRIFT) only applies when both
// sightings carry the SAME prefix length -- that's the genuinely ambiguous
// case (two candidates of comparable specificity, nothing else to tell them
// apart). Whenever the two lengths DIFFER -- a short src-hash refresh
// against a neighbour whose full pubkey is already known, or a fresh full
// advert arriving to upgrade an already-known short-prefix sighting -- the
// prefix match alone is trusted regardless of SNR drift, in either
// direction: requiring fresh SNR agreement on every such match made
// ordinary SNR variance (well beyond 6dB packet-to-packet in practice)
// silently spawn a second, disconnected slot instead of ever merging into
// the real one. A longer prefix always upgrades the stored one in place;
// name==NULL (a discover hit, not an advert) leaves any existing name/type/
// location/advert_timestamp untouched rather than blanking them.
void Beebo::putNeighbour(const uint8_t* pubkey, uint8_t pubkey_len, uint32_t advert_timestamp,
                          int8_t snr, uint8_t type, const char* name, int32_t lat, int32_t lon) {
  NeighbourInfo* slot = &neighbours[0];
  NeighbourInfo* oldest_slot = &neighbours[0];
  uint32_t oldest = 0xFFFFFFFF;
  bool matched = false;
  int best_specificity = -1;
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    NeighbourInfo& nb = neighbours[i];
    if (nb.heard_timestamp != 0) {
      uint8_t shared = pubkey_len < nb.pubkey_len ? pubkey_len : nb.pubkey_len;
      // beebo: trust the prefix match alone whenever the two sightings carry
      // DIFFERENT prefix lengths -- one side strictly contains more identity
      // information than the other (e.g. a fresh full-pubkey advert arriving
      // to upgrade an already-known short-prefix sighting, or vice versa),
      // so there's nothing genuinely ambiguous to resolve via SNR. Only
      // require SNR-drift agreement when both sides carry the SAME prefix
      // length -- the one case where two truly distinct nodes could
      // plausibly collide on it with nothing else to tell them apart.
      bool trust_established = nb.pubkey_len != pubkey_len;
      int drift = (int)snr - (int)nb.snr;
      if (drift < 0) drift = -drift;
      if (memcmp(pubkey, nb.pubkey, shared) == 0 &&
          (trust_established || drift <= NEIGHBOUR_SNR_DRIFT)) {
        // beebo: keep searching all slots and take the MOST SPECIFIC match
        // (longest stored prefix), not just the first one iteration
        // happens to reach -- a short-prefix slot that matches itself
        // (shared == both lengths, no trust_established) would otherwise
        // shadow a later, more specific slot the same sighting also
        // matches (e.g. a full-pubkey neighbour already known from an
        // advert), permanently splitting the two into separate entries.
        // Ties keep the first (lowest-index) candidate.
        if (!matched || nb.pubkey_len > best_specificity) {
          slot = &nb;
          matched = true;
          best_specificity = nb.pubkey_len;
        }
      }
    }
    if (nb.heard_timestamp < oldest) {  // else target the oldest slot
      oldest = nb.heard_timestamp;
      oldest_slot = &nb;
    }
  }
  if (!matched) slot = oldest_slot;
  bool is_new = !matched;
  if (pubkey_len >= slot->pubkey_len || is_new) {
    memset(slot->pubkey, 0, sizeof(slot->pubkey));
    memcpy(slot->pubkey, pubkey, pubkey_len);
    slot->pubkey_len = pubkey_len;
  }
  slot->heard_timestamp = getRTCClock()->getCurrentTime();
  slot->snr = snr;
  if (name != NULL) {
    slot->advert_timestamp = advert_timestamp;
    slot->type = type;
    slot->lat = lat;
    slot->lon = lon;
    StrHelper::strzcpy(slot->name, name, sizeof(slot->name));
  } else if (is_new) {
    slot->advert_timestamp = 0;
    slot->type = 0xFF;
    slot->lat = slot->lon = 0;
    slot->name[0] = 0;
  }
}

// beebo: companion-side counterpart to the repeater ACL refresh in
// onPeerDataRecv (BeeboRepeater.cpp isn't involved -- that call sits inline
// in this file's own onPeerDataRecv override, below) -- both funnel into the
// same putNeighbour(). getPathHashCount()==0 gates on zero-hop the same way
// onAdvertRecv() does; a multi-hop message from a known contact says nothing
// about who our own direct neighbours are.
void Beebo::refreshNeighbourFromContact(const ContactInfo& from, mesh::Packet* pkt) {
  if (pkt->getPathHashCount() != 0) return;
  putNeighbour(from.id.pub_key, PUB_KEY_SIZE, from.last_advert_timestamp,
               (int8_t)(pkt->getSNR() * 4), from.type, from.name, from.gps_lat, from.gps_lon);
}

// beebo: shared by BEEBO_CMD_SET_NEIGHBOR_REMOVE's own binary handler (below,
// this file) and CommonCLICallbacks::removeNeighbor() (the CommonCLI
// fallback's 'neighbor.remove <hex>', BeeboRepeater.cpp) -- one
// implementation, so they can't drift apart. Not role-gated: the direct-
// neighbour table itself isn't repeater-only (putNeighbour() above runs for
// any received advert regardless of role).
//
// Unlike putNeighbour()'s prefix-over-shared-length match, this requires the
// stored slot's length to match the request length EXACTLY, not just share
// a prefix over the shorter of the two. `beebo neighbors remove`/`forget`
// only accept a specific row's sequence number (never a bare hash/name), so
// each request here always carries that one row's own exact stored key --
// but if that row happens to be a still-unmerged partial-key duplicate (see
// putNeighbour()'s specificity comment), a shared-length match would let its
// short key also match a longer, unrelated (or "good") slot that merely
// happens to share the prefix -- e.g. deleting a 3-byte row could otherwise
// take out the real full-pubkey neighbour behind it too. Exact-length
// matching means a request can only ever hit a slot of the identical
// specificity it was resolved from.
bool Beebo::removeNeighborByPrefix(const uint8_t* pubkey, int key_len) {
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    NeighbourInfo& nb = neighbours[i];
    if (nb.heard_timestamp == 0) continue;
    if (nb.pubkey_len == key_len && key_len > 0 && memcmp(pubkey, nb.pubkey, key_len) == 0) {
      nb.pubkey_len = 0;
      nb.heard_timestamp = 0;
      return true;
    }
  }
  return false;
}

// beebo: every advert flows through here. We still chain to the base (contact
// discovery / auto-add are unchanged), then additionally capture zero-hop
// adverts into the direct-neighbour table — these are our immediate neighbours,
// regardless of whether they're a saved contact.
void Beebo::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp,
                          const uint8_t* app_data, size_t app_data_len) {
  BaseChatMesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len);

  if (packet->getPathHashCount() != 0) return;   // not heard directly => not a neighbour
  AdvertDataParser parser(app_data, app_data_len);
  if (!(parser.isValid() && parser.hasName())) return;
  int32_t lat = parser.hasLatLon() ? parser.getIntLat() : 0;
  int32_t lon = parser.hasLatLon() ? parser.getIntLon() : 0;
  putNeighbour(id.pub_key, PUB_KEY_SIZE, timestamp, (int8_t)(packet->getSNR() * 4),
               parser.getType(), parser.getName(), lat, lon);
}

static int sort_by_recent(const void *a, const void *b) {
  return ((AdvertPath *) b)->recv_timestamp - ((AdvertPath *) a)->recv_timestamp;
}

int Beebo::getRecentlyHeard(AdvertPath dest[], int max_num) {
  if (max_num > ADVERT_PATH_TABLE_SIZE) max_num = ADVERT_PATH_TABLE_SIZE;
  qsort(advert_paths, ADVERT_PATH_TABLE_SIZE, sizeof(advert_paths[0]), sort_by_recent);

  for (int i = 0; i < max_num; i++) {
    dest[i] = advert_paths[i];
  }
  return max_num;
}

bool Beebo::filterRecvFloodPacket(mesh::Packet* packet) {
  // beebo: stash the packet's region match for allowPacketForward() to
  // consult right after (repeater role only; see that function). Reuses
  // companion's own existing region_map instance rather than a second
  // copy. Run unconditionally regardless of role -- companion role just
  // never reads the
  // result, and this keeps recv_pkt_region live across a hot role switch
  // instead of stale from whenever it was last set.
#if BEEBO_ENABLE_REPEATER_ROLE
  if (packet->getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD) {
    recv_pkt_region = region_map.findMatch(packet, REGION_DENY_FLOOD);
  } else if (packet->getRouteType() == ROUTE_TYPE_FLOOD) {
    if (region_map.getWildcard().flags & REGION_DENY_FLOOD) {
      recv_pkt_region = NULL;
    } else {
      recv_pkt_region = &region_map.getWildcard();
    }
  } else {
    recv_pkt_region = NULL;
  }
#else
  // beebo: STATIC_ROLE_BUILDS -- no region_map without repeater support;
  // allowPacketForward() never reads recv_pkt_region when !isRepeater().
  recv_pkt_region = NULL;
#endif
  // do normal processing
  return false;
}

// beebo: same loop-detection maximums as simple_repeater's own --
// indexed by path-hash size (1/2/3 bytes).
static uint8_t max_loop_minimal[] =  { 0, /* 1-byte */  4, /* 2-byte */  2, /* 3-byte */  1 };
static uint8_t max_loop_moderate[] = { 0, /* 1-byte */  2, /* 2-byte */  1, /* 3-byte */  1 };
static uint8_t max_loop_strict[] =   { 0, /* 1-byte */  1, /* 2-byte */  1, /* 3-byte */  1 };

bool Beebo::isLooped(const mesh::Packet* packet, const uint8_t max_counters[]) {
  uint8_t hash_size = packet->getPathHashSize();
  uint8_t hash_count = packet->getPathHashCount();
  uint8_t n = 0;
  const uint8_t* path = packet->path;
  while (hash_count > 0) {      // count how many times this node is already in the path
    if (self_id.isHashMatch(path, hash_size)) n++;
    hash_count--;
    path += hash_size;
  }
  return n >= max_counters[hash_size];
}

bool Beebo::allowPacketForward(const mesh::Packet* packet) {
  // beebo: role-branched -- companion keeps its pre-existing behaviour
  // unchanged (client_repeat, a companion-only "also relay while acting as
  // someone's client" toggle); repeater role reads
  // the RAM-cached /com_prefs fields loadRepeaterFwdPrefs() populates
  // instead of _role_state->prefs.* directly (this function runs on every
  // received flood packet, so it can't afford readComPrefsField()'s
  // per-call file open -- see the _fwd_* field comments in Beebo.h).
  //
  // Must not fall back to `return _role_state->prefs.client_repeat != 0;`
  // unconditionally -- that's a companion-only pref that defaults off, so a
  // freshly-role-switched repeater would silently forward nothing at all
  // regardless of repeater.repeat/node.role, with none of simple_repeater's
  // hop-limit/loop-detection/region-scoping logic applied either.
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    if (_role_state->prefs.disable_fwd) return false;
    if (packet->isRouteFlood()) {
      if (packet->getPathHashCount() >= _role_state->prefs.flood_max
          || (packet->getRouteType() == ROUTE_TYPE_FLOOD && packet->getPathHashCount() >= _role_state->prefs.flood_max_unscoped)
          || (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT && packet->getPathHashCount() >= _role_state->prefs.flood_max_advert)) {
        ++_max_hop_no_fwd_count;
        logForwardDenyEvent(EVENT_MAX_HOP_NO_FWD, packet);
        return false;
      }
    }
    if (packet->isRouteFlood() && recv_pkt_region == NULL) {
      MESH_DEBUG_PRINTLN("allowPacketForward: unknown transport code, or wildcard not allowed for FLOOD packet");
      ++_region_no_fwd_count;
      logForwardDenyEvent(EVENT_REGION_NO_FWD, packet);
      return false;
    }
    if (packet->isRouteFlood() && _role_state->prefs.loop_detect != LOOP_DETECT_OFF) {
      const uint8_t* maximums;
      if (_role_state->prefs.loop_detect == LOOP_DETECT_MINIMAL) {
        maximums = max_loop_minimal;
      } else if (_role_state->prefs.loop_detect == LOOP_DETECT_MODERATE) {
        maximums = max_loop_moderate;
      } else {
        maximums = max_loop_strict;
      }
      if (isLooped(packet, maximums)) {
        MESH_DEBUG_PRINTLN("allowPacketForward: FLOOD packet loop detected!");
        ++_loop_no_fwd_count;
        logForwardDenyEvent(EVENT_LOOP_NO_FWD, packet);
        return false;
      }
    }
    return true;
  }
#endif
  return _role_state->prefs.client_repeat != 0;
}

// beebo: shared append helper for allowPacketForward()'s three MON_EVENT
// types (see EVENT_MAX_HOP_NO_FWD/EVENT_REGION_NO_FWD/EVENT_LOOP_NO_FWD,
// MonRing.h) -- just the declining packet's own hash (Packet::
// calculateMonRingHash(), same convention RxRecord/TxRecord/
// EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT already use), so a downloaded trace
// can tell which packet each decline was for. The lifetime cumulative
// count is the caller's own member field (_max_hop_no_fwd_count/etc.,
// surfaced via the GET_MONRING header instead) -- not duplicated here.
void Beebo::logForwardDenyEvent(uint8_t event_type, const mesh::Packet* packet) {
  EventRecord rec{};
  rec.event_type = event_type;
  uint32_t pkt_hash = packet->calculateMonRingHash();
  memcpy(&rec.data[1], &pkt_hash, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

void Beebo::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis) {
  // beebo: path_hash_mode is one of the 14 SharedPrefs fields (see
  // NodePrefs.h) -- no role branch needed since both roles share the field.
  uint8_t path_hash_mode = _role_state->prefs.path_hash_mode;
  if (scope.isNull()) {
    sendFlood(pkt, delay_millis, path_hash_mode + 1);
  } else {
    uint16_t codes[2];
    codes[0] = scope.calcTransportCode(pkt);
    codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
    sendFlood(pkt, codes, delay_millis, path_hash_mode + 1);
  }
}

void Beebo::onControlDataRecv(mesh::Packet *packet) {
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }

  // beebo: a NODE_DISCOVER_RESP heard directly is as good a neighbour sighting
  // as an advert, just with (at most) a partial pubkey and no name/location.
  // Wire layout after the control-type byte: [SNR_in:1][tag:4][pubkey:1..32].
  if (packet->getPathHashCount() == 0 && packet->payload_len >= 6
      && (packet->payload[0] & 0xF0) == 0x90 /* ControlType::NODE_DISCOVER_RESP */) {
    uint8_t pubkey_len = packet->payload_len - 6;
    if (pubkey_len > 0 && pubkey_len <= PUB_KEY_SIZE) {
      putNeighbour(&packet->payload[6], pubkey_len, 0, (int8_t)(_radio->getLastSNR() * 4),
                   0xFF, NULL, 0, 0);
    }
  }

  // beebo: "node discover" protocol, generalized from simple_repeater's
  // own onControlDataRecv: neither answering nor sending is role-gated
  // here -- a
  // companion node answers truthfully as ADV_TYPE_CHAT, a repeater as
  // ADV_TYPE_REPEATER, whichever the filter asked for (simple_repeater's
  // original only ever answered as a repeater, since that's the only role
  // it has). REQ-sending (sendNodeDiscoverReq/"discover.neighbors"/
  // BEEBO_CMD_NODE_DISCOVER) is not role-gated either: any node can
  // legitimately ask "who's near me". RESP-matching here is naturally
  // gated by pending_discover_tag, which only sendNodeDiscoverReq() ever
  // sets.
  // Simplified from the original: skips the optional "since" field
  // (discovery_mod_timestamp incremental-query gate) -- multi_role has no
  // such NodePrefs field, and this is already a minor/QoL feature per the
  // review's own ranking, not worth a new persisted field for. Always
  // answers a matching-filter request instead.
  uint8_t ctl_type = packet->payload[0] & 0xF0;
  uint8_t own_type = isRepeater() ? ADV_TYPE_REPEATER : ADV_TYPE_CHAT;
#if BEEBO_ENABLE_REPEATER_ROLE
  bool fwd_ok = !_role_state->prefs.disable_fwd;
#else
  // beebo: STATIC_ROLE_BUILDS -- no _role_state->prefs/disable_fwd concept without
  // repeater support; a companion-only build always answers.
  bool fwd_ok = true;
#endif
  bool discover_req_ok = ctl_type == CTL_TYPE_NODE_DISCOVER_REQ
      && packet->payload_len >= 6 && fwd_ok && discover_limiter.allow(getRTCClock()->getCurrentTime());
  if (discover_req_ok) {
    int j = 1;
    uint8_t filter = packet->payload[j++];
    uint32_t tag;
    memcpy(&tag, &packet->payload[j], 4); j += 4;

    if ((filter & (1 << own_type)) != 0) {
      bool prefix_only = packet->payload[0] & 1;
      uint8_t data[6 + PUB_KEY_SIZE];
      data[0] = CTL_TYPE_NODE_DISCOVER_RESP | own_type;   // low 4-bits for node type
      data[1] = packet->_snr;   // let sender know the inbound SNR ( x 4)
      memcpy(&data[2], &tag, 4);     // include tag from request, for client to match to
      memcpy(&data[6], self_id.pub_key, PUB_KEY_SIZE);
      auto resp = createControlData(data, prefix_only ? 6 + 8 : 6 + PUB_KEY_SIZE);
      if (resp) {
        sendZeroHop(resp, getRetransmitDelay(resp)*4);  // apply random delay (widened x4), as multiple nodes can respond to this
      }
    }
  } else if (ctl_type == CTL_TYPE_NODE_DISCOVER_RESP && packet->payload_len >= 6
             && pending_discover_tag != 0 && !millisHasNowPassed(pending_discover_until)) {
    uint8_t node_type = packet->payload[0] & 0x0F;
    int pubkey_len = packet->payload_len - 6;
    if (pubkey_len > 0 && pubkey_len <= PUB_KEY_SIZE) {
      uint32_t tag;
      memcpy(&tag, &packet->payload[2], 4);
      if (tag == pending_discover_tag) {
        putNeighbour(&packet->payload[6], (uint8_t)pubkey_len, 0, packet->_snr, node_type, NULL, 0, 0);
      }
    }
  }

  int i = 0;
  out_frame[i++] = PUSH_CODE_CONTROL_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = packet->path_len;
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onControlDataRecv(), data received while app offline");
  }
}

void Beebo::onRawDataRecv(mesh::Packet *packet) {
  if (packet->payload_len + 4 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), payload_len too long: %d", packet->payload_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_RAW_DATA;
  out_frame[i++] = (int8_t)(_radio->getLastSNR() * 4);
  out_frame[i++] = (int8_t)(_radio->getLastRSSI());
  out_frame[i++] = 0xFF; // reserved (possibly path_len in future)
  memcpy(&out_frame[i], packet->payload, packet->payload_len);
  i += packet->payload_len;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onRawDataRecv(), data received while app offline");
  }
}

void Beebo::onPokeReply(mesh::Packet *packet, uint32_t tag, int16_t rssi, int16_t snr, int16_t noise_floor) {
  // beebo: append our own (local) telemetry for the reply packet's reception,
  // alongside the far end's telemetry it reported for the original poke --
  // gives both ends of the link in a single round trip.
  int16_t local_rssi;
#ifdef MSG_INCLUDE_RSSI
  local_rssi = (int16_t) packet->getRSSI();
#else
  local_rssi = INT16_MIN;
#endif
  int16_t local_snr = (int16_t) (packet->getSNR() * 4);
  int16_t local_noise_floor = (int16_t) _radio->getNoiseFloor();

  int i = 0;
  out_frame[i++] = PUSH_CODE_POKE_REPLY;
  memcpy(&out_frame[i], &tag, 4); i += 4;
  memcpy(&out_frame[i], &rssi, 2); i += 2;
  memcpy(&out_frame[i], &snr, 2); i += 2;
  memcpy(&out_frame[i], &noise_floor, 2); i += 2;
  memcpy(&out_frame[i], &local_rssi, 2); i += 2;
  memcpy(&out_frame[i], &local_snr, 2); i += 2;
  memcpy(&out_frame[i], &local_noise_floor, 2); i += 2;

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onPokeReply(), data received while app offline");
  }
}

void Beebo::onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                         const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) {
  uint8_t path_sz = flags & 0x03;  // NEW v1.11+
  if (12 + path_len + (path_len >> path_sz) + 1 > sizeof(out_frame)) {
    MESH_DEBUG_PRINTLN("onTraceRecv(), path_len is too long: %d", (uint32_t)path_len);
    return;
  }
  int i = 0;
  out_frame[i++] = PUSH_CODE_TRACE_DATA;
  out_frame[i++] = 0; // reserved
  out_frame[i++] = path_len;
  out_frame[i++] = flags;
  memcpy(&out_frame[i], &tag, 4);
  i += 4;
  memcpy(&out_frame[i], &auth_code, 4);
  i += 4;
  memcpy(&out_frame[i], path_hashes, path_len);
  i += path_len;

  memcpy(&out_frame[i], path_snrs, path_len >> path_sz);
  i += path_len >> path_sz;
  out_frame[i++] = (int8_t)(packet->getSNR() * 4); // extra/final SNR (to this node)

  if (_serial->isConnected()) {
    _serial->writeFrame(out_frame, i);
  } else {
    MESH_DEBUG_PRINTLN("onTraceRecv(), data received while app offline");
  }
}

Beebo::Beebo(mesh::Radio &radio, mesh::RNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables, DataStore& store)
    : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(16), tables),
      _serial(NULL), telemetry(MAX_PACKET_PAYLOAD - 4), _store(&store) {
  _iter_started = false;
  _pending_disconnect = false;
  offline_queue_len = 0;
  app_target_ver = 0;
  _app_max_tx = MAX_FRAME_SIZE;
  _app_stream = false;
  if (_monread.active) monring.resumeAfterRead();  // abandoned mid-stream: don't leave capture paused
  _monread.active = false;
  _statread.active = false;
  _neighread.active = false;
  _pathread.active = false;
  clearPendingReqs();
  next_ack_idx = 0;
  ack_overflow_count = 0;
  sign_data = NULL;
  ota_handle = 0;
  ota_partition = NULL;
  dirty_contacts_expiry = 0;
  memset(advert_paths, 0, sizeof(advert_paths));
  memset(neighbours, 0, sizeof(neighbours));   // beebo: direct-neighbour table
  memset(send_scope.key, 0, sizeof(send_scope.key));
  send_unscoped = false;

  // defaults
  memset(&_role_state->prefs, 0, sizeof(_role_state->prefs));
  _role_state->prefs.airtime_factor = 1.0;
  strcpy(_role_state->prefs.node_name, "NONAME");
  _role_state->prefs.freq = LORA_FREQ;
  _role_state->prefs.sf = LORA_SF;
  _role_state->prefs.bw = LORA_BW;
  _role_state->prefs.cr = LORA_CR;
  _role_state->prefs.tx_power_dbm = LORA_TX_POWER;
  _role_state->prefs.gps_enabled = 0;       // GPS disabled by default
  _role_state->prefs.gps_interval = 0;      // No automatic GPS updates by default
  //com_prefs.rx_delay_base = 10.0f;  enable once new algo fixed
#if defined(USE_SX1262) || defined(USE_SX1268)
#ifdef SX126X_RX_BOOSTED_GAIN
  _role_state->prefs.rx_boosted_gain = SX126X_RX_BOOSTED_GAIN;
#else
  _role_state->prefs.rx_boosted_gain = 1; // enabled by default
#endif
#endif
  _role_state->prefs.radio_fem_rxgain = 0; // LNA disabled by default
  _role_state->prefs.ble_enabled = 1;      // BLE transport on by default
  _role_state->prefs.tcp_enabled = 0;      // WiFi/TCP transport off by default (no creds initially)
  _role_state->prefs.usb_enabled = 1;      // USB companion transport on by default (lets a fresh node be configured over USB before BLE/WiFi are set up)
  // beebo: ENV and TUNE are opt-in (ENV is comparatively high-volume; TUNE is
  // the still-experimental dynamic-tuning optimizer's own record kind, off
  // until repeater.routing.tune.enabled is turned on) -- everything else,
  // including EVENT, captures by default. Kept in sync with MonRing.h's own
  // in-class _config default (test/test_monring's own EXPECT_EQ on this
  // exact bit combination is what caught this previously disagreeing with
  // that header).
  _role_state->prefs.monring_config = (MON_CAP_ALL & ~MON_CAP_ENV & ~MON_CAP_TUNE) | MON_CAP_ENABLED;
  // beebo: the memset above wipes out BeeboBasePrefs.h's in-class default
  // (0xFFFFFFFFu, capture every event type) same as it wipes monring_config's
  // -- re-assert it explicitly here too, or a device whose persisted file
  // predates this field (DataStore.cpp's own tail-hazard-gated read for it)
  // boots with a live mask of 0, silently dropping every MON_EVENT record
  // forever despite the live lifetime counters (ack_success_count etc.)
  // still incrementing normally -- confirmed live on `bunch` 2026-08-15,
  // tracked in BUGS.md.
  _role_state->prefs.monring_event_mask = 0xFFFFFFFFu;
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: repeater-role _role_state->prefs (CommonCLI's real
  // NodePrefs, /com_prefs) defaults. These ctor values are only the fallback
  // for a device with no /com_prefs at all (never ran simple_repeater) --
  // cli.loadPrefs() (loadRoleState()'s repeater branch, Beebo.cpp)
  // overwrites every one of them from the real file if it exists, so a
  // device that already ran simple_repeater (e.g. reflashed from
  // heltec_v4_repeater to multi_role) keeps whatever it already had
  // configured there, not a silent reset. Every value here matches
  // simple_repeater's own MyMesh::begin() defaults exactly, for the fields
  // it seeds the same way (password: ADMIN_PASSWORD; flood_max/
  // flood_max_unscoped/flood_max_advert/tx_delay_factor/
  // direct_tx_delay_factor: same literals as its own struct used to carry
  // as in-class defaults before ComPrefs lost those when the shared
  // CommonCLI.h/.cpp extraction touched only beebo-only fields, not these
  // real upstream ones -- see COMMONCLI_TEXT_DISPATCH.md's persistence
  // section). Every other ComPrefs field not listed here is fine at zero --
  // Beebo is a global object, so every member is zero-initialized before
  // this constructor body even runs.
  StrHelper::strncpy(_role_state->prefs.password, ADMIN_PASSWORD, sizeof(_role_state->prefs.password));
  _role_state->prefs.flood_max = 64;
  _role_state->prefs.flood_max_unscoped = 64;
  _role_state->prefs.flood_max_advert = 8;
  _role_state->prefs.tx_delay_factor = 0.5f;
  _role_state->prefs.direct_tx_delay_factor = 0.3f;
  _role_state->prefs.airtime_factor = 1.0f;   // repeater's own independent copy
#endif
}

// beebo: loadRepeaterAuthPrefs()/loadRepeaterFwdPrefs()/readComPrefsField()/
// writeComPrefsField()/updateAdvertTimer()/updateFloodAdvertTimer() moved to
// BeeboRepeater.cpp in the event-loop-review file split -- see Beebo.h's
// declarations for the full rationale.

// beebo: the per-role state store -- loads/migrates/generates role's
// identity into its own resident role_state_store[role].identity slot.
// Does NOT touch self_id -- begin() mirrors self_id explicitly once the
// live role is known at boot (a runtime switch never repoints self_id --
// it always reboots instead, see requestNodeRoleSwitch()). Splitting the
// mirror out lets this run for a role that ISN'T currently live (eager
// per-role boot loading, loadRoleState() below) without clobbering self_id
// mid-load.
//
// beebo: migrateLegacyIdentity() must only be offered to the role the node
// was already operating as before the split (DataStore.h's own doc comment
// on migrateLegacyIdentity: "the other role must always start from a fresh
// identity"), because it doesn't delete "_main" after copying it, so it
// stays available to every future caller. Since both roles are loaded
// every boot regardless of which is live, an explicit gate here is
// required or both roles' first load would silently migrate the same
// "_main" and end up with IDENTICAL keys instead of one migrated + one
// fresh. _board.role (the resolved default/live role) is already settled
// by the time begin() calls loadRoleState() for either role.
// beebo: shared by loadIdentityForRole()'s missing-file fallback and
// CMD_GENERATE_IDENTITY (BEEBO_CMD_GENERATE_IDENTITY) -- the latter makes
// this independently callable against a role whose identity file may
// already exist, instead of only ever firing here on a missing one.
mesh::LocalIdentity Beebo::generateFreshIdentity() {
  mesh::LocalIdentity id = radio_new_identity(); // create new random identity
  int count = 0;
  while (count < 10 && (id.pub_key[0] == 0x00 || id.pub_key[0] == 0xFF)) { // reserved id hashes
    id = radio_new_identity();
    count++;
  }
  return id;
}

void Beebo::loadIdentityForRole(uint8_t role) {
  mesh::LocalIdentity& id = role_state_store[role].identity;
  if (_store->loadRoleIdentity(role, id)) return;
  if (role == _board.role && _store->migrateLegacyIdentity(role, id)) return;
  id = generateFreshIdentity();
  _store->saveRoleIdentity(role, id);
}

// beebo: the per-role state store -- loads a compiled-in role's full
// persisted state (identity, prefs, and for repeater: ACL/RegionMap) into
// its own resident role_state_store[] slot, independent of which role is
// currently live. Called once per compiled-in role from begin() (eager
// per-role boot load) -- replaces ensureRepeaterStateLoaded()'s lazy
// load-on-first-repeater-entry model and the ~20 defensive call sites that
// used to guard against touching not-yet-loaded repeater state. Folds in
// ensureRepeaterStateLoaded()'s former body (BeeboRepeater.cpp) verbatim
// for the repeater branch, operating on `slot`/`role` explicitly instead
// of the implicit "whatever _role_state/self_id currently are" the old
// lazy version relied on (see acl.load()'s identity-keying fix below --
// that's the one behavior-relevant change from the old body, everything
// else is the same load sequence).
void Beebo::loadRoleState(uint8_t role) {
  BeeboRoleState& slot = role_state_store[role];
  loadIdentityForRole(role);

  if (role == NODE_ROLE_COMPANION) {
    bool existed = _store->loadBeeboCompanionPrefs(slot.prefs, _board, sensors.node_lat, sensors.node_lon);
    if (!existed) {
      // beebo: true first boot under companion role -- seed from stock
      // companion_radio's /new_prefs (read-only, never written back) if
      // this node ever ran stock firmware, else the same
      // ADVERT_NAME/pubkey-hex/DEFAULT_FLOOD_SCOPE_NAME defaults as before.
      FILESYSTEM* fs = _store->getPrimaryFS();
      bool had_legacy = fs->exists("/new_prefs") || fs->exists("/node_prefs");
      if (had_legacy) {
        _store->loadLegacyNodePrefs(slot.prefs, sensors.node_lat, sensors.node_lon);
      } else {
#ifdef ADVERT_NAME
        strcpy(slot.prefs.node_name, ADVERT_NAME);
#else
        // use hex of first 4 bytes of identity public key as default node name
        char pub_key_hex[10];
        mesh::Utils::toHex(pub_key_hex, slot.identity.pub_key, 4);
        strcpy(slot.prefs.node_name, pub_key_hex);
#endif
#ifdef DEFAULT_FLOOD_SCOPE_NAME
        strcpy(slot.prefs.default_scope_name, DEFAULT_FLOOD_SCOPE_NAME);
        {
          TransportKeyStore temp;
          TransportKey key;
          temp.getAutoKeyFor(0, "#" DEFAULT_FLOOD_SCOPE_NAME, key);
          memcpy(slot.prefs.default_scope_key, key.key, sizeof(key.key));
        }
#endif
      }
      // beebo: write the seed/defaults out now, directly (not via
      // savePrefs()/flushDirtyPrefs(), which target whichever role
      // _role_state currently points at -- this slot may not be live yet).
      _store->saveBeeboCompanionPrefs(slot.prefs, _board, sensors.node_lat, sensors.node_lon);
    }
  }
#if BEEBO_ENABLE_REPEATER_ROLE
  else if (role == NODE_ROLE_REPEATER) {
    // beebo: load the repeater-role admin login
    // table. Separate file (/s_contacts, ClientACL.cpp), no interaction
    // with /new_prefs's OTA-continuity story. Keyed on this role's own
    // identity explicitly (not self_id, which may currently hold a
    // different role's identity during eager boot loading) -- acl/
    // key_store/region_map (Beebo.h) are already permanently aliased to
    // this same slot, so this writes straight into the resident object.
    acl.load(_store->getPrimaryFS(), slot.identity);
    // beebo: SETTINGS_ISOLATION -- own region file, /beebo_regions, instead
    // of /regions2 (shared with stock simple_repeater). One-time read-only
    // seed from /regions2 if /beebo_regions doesn't exist yet; /regions2
    // is never written to.
    {
      FILESYSTEM* fs = _store->getPrimaryFS();
      if (fs->exists("/beebo_regions")) {
        region_map.load(fs, "/beebo_regions");
      } else if (fs->exists("/regions2")) {
        region_map.load(fs, "/regions2");
        region_map.save(fs, "/beebo_regions");
      }
    }
    // beebo: restores stock simple_repeater's own default-scope seeding --
    // auto-creates DEFAULT_FLOOD_SCOPE_NAME only if no default region
    // exists yet at all; an operator who already set one keeps it untouched.
    if (region_map.getDefaultRegion() == NULL) {
#ifdef DEFAULT_FLOOD_SCOPE_NAME
      RegionEntry* r = region_map.findByName(DEFAULT_FLOOD_SCOPE_NAME);
      if (r == NULL) {
        r = region_map.putRegion(DEFAULT_FLOOD_SCOPE_NAME, 0);  // auto-create the default scope region
        if (r) { r->flags = 0; }   // Allow-flood
      }
      if (r) {
        region_map.setDefaultRegion(r);
        region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      }
#endif
    }
    // beebo: SETTINGS_ISOLATION -- ComPrefs now folds into /beebo_repeater
    // (raw blob). Only a true first boot (no /beebo_repeater yet) reads
    // /com_prefs at all, via CommonCLI's own (untouched) loadPrefs() --
    // read-only, /com_prefs is never written back to. cli's own prefs
    // pointer must target this slot for that call to land correctly (it
    // may not be the live role yet), restored to the live role's slot
    // afterward by begin()'s own repoint once every role has loaded.
    bool beebo_repeater_existed = _store->loadBeeboRepeaterPrefs(slot.prefs, _board, static_cast<ComPrefs*>(&slot.prefs), sizeof(ComPrefs));
    if (!beebo_repeater_existed) {
      FILESYSTEM* fs = _store->getPrimaryFS();
      if (fs->exists("/com_prefs")) {
        cli.setPrefs(&slot.prefs);
        cli.loadPrefs(fs);
      }
      // beebo: rx_delay_base/airtime_factor need no explicit seed here --
      // they're SharedPrefs fields (NodePrefs.h), one physical storage slot
      // in this role's own prefs, seeded by loadBeeboRepeaterPrefs()/
      // loadBeeboCompanionPrefs() from whichever role loaded first if this
      // is genuinely the second role ever activated on this device.
      _store->saveBeeboRepeaterPrefs(slot.prefs, _board, static_cast<ComPrefs*>(&slot.prefs), sizeof(ComPrefs));
    }
    // beebo: 0 is a real, literal "disabled" value, not "unset" -- this
    // clamp only catches genuinely out-of-range data (an older beebo build's
    // shadow-offset scheme, or file corruption), never a deliberate 0.
    if (slot.prefs.dedup_window_ms > DEDUP_WINDOW_MAX_MS) {
      slot.prefs.dedup_window_ms = DEDUP_LIVE_WINDOW_MS_DEFAULT;
    }
  }
#endif
}

void Beebo::begin() {
  Mesh::begin();

  // beebo: BeeboBoardPrefs's own file (role/board_password/board_name,
  // and as of BOARD_BATTERY_PREFS.md the seven battery/ADC fields).
  // Before this file existed (or before it carried the battery/ADC
  // fields), these were persisted as a tail fold-in of /beebo_companion/
  // /beebo_repeater -- still read there by loadBeeboCompanionPrefs's/
  // loadBeeboRepeaterPrefs's own tail-hazard guards, purely as a one-time
  // migration seed now. board_existed false means this is that migration
  // case (or a genuinely fresh device); see the saveBeeboBoardPrefs() call
  // below (after both roles' own files have loaded) for why the persist
  // has to wait until then rather than happening right here.
  bool board_existed = _store->loadBeeboBoardPrefs(_board);

  // beebo: PER_ROLE_IDENTITY -- true first boot under this role (neither
  // role has its own identity file yet). Detect a role carried over from
  // pre-multi_role firmware via its role-exclusive files, so reflashing
  // beebo on top of an existing simple_repeater/companion_radio node keeps
  // operating as that role instead of silently defaulting to companion --
  // /s_contacts is repeater-only (ClientACL), /contacts3 and /channels2
  // are companion-only (companion_radio's DataStore). A genuinely fresh
  // device has none of these and falls through to sanitizeNodeRole()'s
  // own companion default below.
  {
    mesh::LocalIdentity tmp;
    if (!_store->loadRoleIdentity(NODE_ROLE_COMPANION, tmp) &&
        !_store->loadRoleIdentity(NODE_ROLE_REPEATER, tmp)) {
      FILESYSTEM* fs = _store->getPrimaryFS();
      if (fs->exists("/s_contacts")) {
        _board.role = NODE_ROLE_REPEATER;
      } else if (fs->exists("/contacts3") || fs->exists("/channels2")) {
        _board.role = NODE_ROLE_COMPANION;
      }
    }
  }
  // sync cache -- loadBeeboCompanionPrefs()/loadBeeboBoardPrefs() (or the
  // detection above) write role directly
  sanitizeNodeRole();

  // beebo: the per-role state store -- eager per-role boot load. Every
  // compiled-in role gets its own resident role_state_store[] slot loaded
  // now (identity, prefs, and for repeater ACL/RegionMap -- see
  // loadRoleState()'s own comment), regardless of which is actually live.
  // This is what fixes the boot-time clobbering bug that the old
  // single-shared-slot0 design had (loadBeeboCompanionPrefs() and
  // ensureRepeaterStateLoaded() used to both write into the exact same
  // _role_state->prefs object, so a companion-configured boot on a
  // dual-role-tested device silently got its BeeboBasePrefs fields
  // overwritten by repeater's persisted copies) -- each role now has its
  // own physically separate slot, so there's nothing left to clobber.
#if BEEBO_ENABLE_COMPANION_ROLE
  loadRoleState(NODE_ROLE_COMPANION);
#endif
#if BEEBO_ENABLE_REPEATER_ROLE
  loadRoleState(NODE_ROLE_REPEATER);
#endif

  // beebo: BeeboBoardPrefs unification -- board_existed false means
  // /beebo_board either didn't exist yet (pre-refactor device, or
  // genuinely fresh) or predated the battery/ADC fields
  // (BOARD_BATTERY_PREFS.md) -- either way, persist whatever role
  // detection/sanitizeNodeRole() above settled on, plus whatever
  // loadRoleState() above just migration-seeded into `_board` from
  // /beebo_companion's/beebo_repeater's own legacy tail (board_password/
  // board_name/adc_multiplier/adc_resolution_bits/batt_present/
  // batt_sample_period_secs/batt_sample_window_secs/batt_charged_mv/
  // idle_margin_ms), into the new file now. Deliberately placed here, not
  // right after loadBeeboBoardPrefs() above -- that seed only exists once
  // loadRoleState() has actually read it in, so saving any earlier would
  // persist `_board`'s plain compiled-in defaults instead of the real
  // migrated values on exactly the one-time upgrade path this exists to
  // handle correctly.
  if (!board_existed) {
    _store->saveBeeboBoardPrefs(_board);
  } else {
    // beebo: loadRoleState(NODE_ROLE_COMPANION) above just called
    // loadBeeboCompanionPrefs(), which unconditionally re-reads _board.role/
    // board_password/board_name/the battery-ADC fields from /beebo_companion's
    // own legacy tail (see that function's own comment: "a genuine migration
    // seed when /beebo_board predates these fields, an inert echo of the
    // already-authoritative /beebo_board value otherwise") -- that claim only
    // holds if /beebo_companion's tail is actually kept in sync with
    // /beebo_board on every write, which it isn't: requestNodeRoleSwitch()
    // persists a role change to /beebo_board immediately, but never to
    // /beebo_companion's own legacy tail copy, so that tail can carry an
    // arbitrarily old role byte from whenever it was last flushed. board_existed true means
    // /beebo_board is the real authoritative file here, so undo that clobber
    // by re-reading it now -- the same fix reloadPrefs() already gets for
    // free by ordering its own loadBeeboBoardPrefs() call after
    // loadBeeboCompanionPrefs(). isRepeater()/isCompanion() read _board.role
    // live, so no cache to re-sync -- just re-normalize against
    // isNodeRoleBuiltIn() in case the reloaded role isn't compiled in.
    _store->loadBeeboBoardPrefs(_board);
    sanitizeNodeRole();
  }

  // beebo: point _role_state (and mirror self_id/cli's own prefs pointer)
  // at the live role's now-resident slot. Every generic call site below
  // this point (clampRadioPrefs(), applyRadioPrefs(), beginCompanion()/
  // beginRepeater(), ...) reads/writes _role_state->prefs transparently,
  // same as before -- it's just guaranteed to be the correct role's slot
  // now instead of always slot 0.
  _role_state = &role_state_store[_board.role];
  self_id = _role_state->identity;
#if BEEBO_ENABLE_REPEATER_ROLE
  cli.setPrefs(&_role_state->prefs);
#endif

  // beebo: begin() is split
  // by role (BeeboCompanion.cpp/BeeboRepeater.cpp) so a companion boot
  // does exactly what companion_radio's own begin() does, no more.
  // beebo: isCompanion()/isRepeater() are mutually exclusive (see
  // requestNodeRoleSwitch()), so exactly one of these runs. Kept as two plain ifs
  // (not else-if) rather than folded into one call, so line coverage shows
  // each role's boot path was actually exercised.
#if BEEBO_ENABLE_COMPANION_ROLE
  if (isCompanion()) beginCompanion();
#endif
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) beginRepeater();
#endif

  // sanitise bad pref values
  clampRadioPrefs();

  // beebo: seed the trend state once at boot from the just-loaded pref,
  // instead of always defaulting to INIT and relying on the first
  // updateBattTrend() call to notice batt_present==NO and correct it.
  resetBattTrendRef(_batt_state, _cached_batt_mv, _board.batt_present);

  // beebo: anchor the battery sample schedule to boot time instead of
  // leaving _next_batt_trigger/_next_batt_deadline at their zero-init value
  // -- t=0 is the start of the first period, so the soft (idle-gated) window
  // is open from the very start, not just its final window_secs as in
  // steady-state scheduling -- there's no prior sample to wait out, so give
  // every loop() tick from boot a chance to land on an idle read. The hard
  // deadline still lands at period_secs + window_secs, same as steady state.
  {
    uint32_t period_secs = _board.batt_sample_period_secs;
    uint32_t window_secs = _board.batt_sample_window_secs;
    _next_batt_trigger = futureMillis(0);
    _next_batt_deadline = futureMillis((period_secs + window_secs) * 1000UL);
  }

  _role_state->prefs.gps_enabled = constrain(_role_state->prefs.gps_enabled, 0, 1);  // Ensure boolean 0 or 1
  _role_state->prefs.gps_interval = constrain(_role_state->prefs.gps_interval, 0, 86400);  // Max 24 hours
  _role_state->prefs.ble_enabled = constrain(_role_state->prefs.ble_enabled, 0, 1);
  _role_state->prefs.tcp_enabled = constrain(_role_state->prefs.tcp_enabled, 0, 1);
  _role_state->prefs.usb_enabled = constrain(_role_state->prefs.usb_enabled, 0, 1);
  // Safety invariant: never leave the node with no transport at all. Turning
  // usb off buys nothing power-wise once both radios are off, so use it as
  // the fallback instead of forcing a radio back on -- an explicit "set usb
  // off" later is still honoured, this only fires here at load.
  if (!_role_state->prefs.ble_enabled && !(_role_state->prefs.tcp_enabled && _role_state->prefs.wifi_ssid[0] != '\0')) {
    _role_state->prefs.usb_enabled = 1;
  }

  if (_role_state->prefs.ble_pin == 0) {
    _active_ble_pin = BLE_PIN_CODE; // otherwise static pin
  } else {
    _active_ble_pin = _role_state->prefs.ble_pin;
  }

  // beebo: role separation -- contact/channel/message state is companion-
  // only (upstream simple_repeater never has a contacts table at all, it
  // uses ClientACL instead); a repeater-role node must not load or persist
  // it, matching handleCmdFrame()'s CMD_SEND_TXT_MSG/CMD_ADD_UPDATE_CONTACT/
  // etc. role guard just above.
#if BEEBO_ENABLE_COMPANION_ROLE
  if (isCompanion()) {
    resetContacts();
    _store->loadContacts(this);
    bootstrapRTCfromContacts();
    addChannel("Public", PUBLIC_GROUP_PSK); // pre-configure Andy's public channel
    _store->loadChannels(this);
  }
#endif

  applyRadioPrefs();
  MESH_DEBUG_PRINTLN("RX Boosted Gain Mode: %s",
                     radio_driver.getRxBoostedGainMode() ? "Enabled" : "Disabled");

  beginTransports();
  resetRxTimeoutClock();
}

// beebo: transports are brought up per the persisted enable flags (NodePrefs):
//   ble_enabled (default 1), tcp_enabled (default 0), usb_enabled (default 1).
// Invariant (enforced just above in begin(), right after loadPrefs()): if ble
// and tcp would both end up off, usb_enabled is forced back on in memory --
// so at least one transport is always up, regardless of what was actually
// persisted (including a fresh/zeroed struct from an unformatted SPIFFS, or a
// short-read pre-usb_enabled file).
// beebo: see this method's own declaration comment in Beebo.h for why it's
// one shared body instead of a copy inlined at each WiFi.onEvent() site.
void Beebo::_onWifiStaEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    _sta_disc_reason = info.wifi_sta_disconnected.reason;
    WIFI_DEBUG_PRINTLN("WiFi disconnected. Flagging for reconnect...");
    _wifi_needs_reconnect = true;
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    _sta_got_ip = true;
    WIFI_DEBUG_PRINTLN("WiFi connected successfully!");
    _wifi_needs_reconnect = false;
    // Re-bind the listening socket: every reassociation (the STA's very
    // first one after boot, and every later one -- auto-reconnect after a
    // beacon timeout, a manual loopTransports() reconnect, a live creds
    // change) leaves the previous WiFiServer orphaned -- see
    // SerialWifiInterface::rebind()'s own comment. Deferred to loopTransports()
    // on the main task -- see _wifi_needs_rebind's own comment in Beebo.h for
    // why this can't call wifi_interface.rebind() directly from here.
    _wifi_needs_rebind = true;
  }
}

// beebo: see this method's own declaration comment in Beebo.h. Var ids are
// defined once, alongside TLOG_XPORT_VAR_CHANGED itself, in TransportLog.h
// -- keep the two in sync if a field is added or removed here.
void Beebo::_checkTransportStateChanges() {
  auto check = [this](int id, int val) {
    if (_last_xport_var[id] != val) {
      int32_t detail = id | ((_last_xport_var[id] & 0xFF) << 8) | ((val & 0xFF) << 16);
      transport_log.log(TLOG_XPORT_VAR_CHANGED, detail);
      _last_xport_var[id] = (int16_t)val;
    }
  };

  check(TLOG_XPORT_VAR_WIFI_IFACE_ENABLED,    wifi_interface.isEnabled());
  check(TLOG_XPORT_VAR_WIFI_IFACE_CONNECTED,  wifi_interface.isConnected());
  check(TLOG_XPORT_VAR_WIFI_LISTENING,        wifi_interface.isListening());
  check(TLOG_XPORT_VAR_BLE_IFACE_ENABLED,     ble_interface.isEnabled());
  check(TLOG_XPORT_VAR_BLE_IFACE_CONNECTED,   ble_interface.isConnected());
  check(TLOG_XPORT_VAR_USB_IFACE_ENABLED,     usb_interface.isEnabled());
  check(TLOG_XPORT_VAR_USB_IFACE_CONNECTED,   usb_interface.isConnected());
  check(TLOG_XPORT_VAR_MULTI_ENABLED,         serial_interface.isEnabled());
  check(TLOG_XPORT_VAR_MULTI_CONNECTED,       serial_interface.isConnected());
  check(TLOG_XPORT_VAR_WIFI_STARTED,          _wifi_started);
  check(TLOG_XPORT_VAR_WIFI_UP,               _wifi_up);
  check(TLOG_XPORT_VAR_WIFI_NEEDS_RECONNECT,  _wifi_needs_reconnect);
  check(TLOG_XPORT_VAR_TRANSPORT_SWITCH_PENDING, _transport_switch_pending);
  check(TLOG_XPORT_VAR_BLE_ADDED,             _ble_added);
  check(TLOG_XPORT_VAR_BLE_UP,                _ble_up);
  check(TLOG_XPORT_VAR_USB_ADDED,             _usb_added);
  check(TLOG_XPORT_VAR_USB_UP,                _usb_up);
  check(TLOG_XPORT_VAR_WL_STATUS,             (int)WiFi.status());
  check(TLOG_XPORT_VAR_ACTIVE,                (int)serial_interface.activeTransportType());
  check(TLOG_XPORT_VAR_WIFI_CREDS_RECONNECT_PENDING, _wifi_creds_reconnect_pending);
}

void Beebo::beginTransports() {
  bool ble_on = _role_state->prefs.ble_enabled != 0;
  bool tcp_on = _role_state->prefs.tcp_enabled != 0 && _role_state->prefs.wifi_ssid[0] != '\0';
  bool usb_on = _role_state->prefs.usb_enabled != 0;

  if (ble_on) {
    ble_interface.begin(BLE_NAME_PREFIX, _role_state->prefs.node_name, getBLEPin());
    serial_interface.addInterface(&ble_interface, nullptr, nullptr, true, TLOG_XPORT_BLE);
    _ble_added = true;
    _ble_up = true;
  }

  if (usb_on) {
    usb_interface.begin(Serial);
    serial_interface.addInterface(&usb_interface, []() -> bool { return (bool)Serial; }, nullptr,
                                  false, TLOG_XPORT_USB);  // non-exclusive: USB coexists with BLE/WiFi
    _usb_added = true;
    _usb_up = true;
  }

  if (tcp_on) {
    board.setInhibitSleep(true);   // prevent sleep when WiFi is active
    WiFi.setAutoReconnect(true);

    WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info){ _onWifiStaEvent(event, info); });

    WiFi.begin(_role_state->prefs.wifi_ssid, _role_state->prefs.wifi_pwd);
    // BLE and TCP are mutually exclusive, so no BT + WiFi coexistence -- power
    // save can always be off so the station stays awake (eliminates the
    // multi-second TCP latency/loss).
    WiFi.setSleep(false);
    wifi_interface.begin(TCP_PORT);
    serial_interface.addInterface(&wifi_interface, nullptr, nullptr, true, TLOG_XPORT_TCP);
    _wifi_started = true;
    _wifi_up = true;
  }

  startInterface(serial_interface);
  debug_log.attach(&usb_interface, RESP_CODE_BEEBO, BEEBO_RESP_DEBUG_LOG, BEEBO_RESP_DEBUG_TLOG);

  for (int i = 0; i < TLOG_XPORT_VAR_COUNT; i++) _last_xport_var[i] = -1;
  _checkTransportStateChanges();   // logs every var's boot value as "changed from -1"
}

// beebo: clamp the radio-affecting prefs to sane ranges — shared by begin()
// and reloadPrefs() so the two load paths can't drift apart.
void Beebo::clampRadioPrefs() {
  _role_state->prefs.rx_delay_base = constrain(_role_state->prefs.rx_delay_base, 0, 20.0f);
  _role_state->prefs.airtime_factor = constrain(_role_state->prefs.airtime_factor, 0, 9.0f);
  _role_state->prefs.freq = constrain(_role_state->prefs.freq, 150.0f, 2500.0f);
  _role_state->prefs.bw = constrain(_role_state->prefs.bw, 7.8f, 500.0f);
  _role_state->prefs.sf = constrain(_role_state->prefs.sf, 5, 12);
  _role_state->prefs.cr = constrain(_role_state->prefs.cr, 5, 8);
  _role_state->prefs.tx_power_dbm = constrain(_role_state->prefs.tx_power_dbm, -9, MAX_LORA_TX_POWER);
  _role_state->prefs.radio_fem_rxgain = constrain(_role_state->prefs.radio_fem_rxgain, 0, 1);
  _board.adc_multiplier = constrain(_board.adc_multiplier, 0.0f, 10.0f);
  if (_board.batt_sample_period_secs == 0) {
    _board.batt_sample_period_secs = BATT_SAMPLE_PERIOD_DEFAULT_SECS;  // beebo: default period
  }
  if (_board.batt_sample_window_secs == 0) {
    _board.batt_sample_window_secs = BATT_SAMPLE_WINDOW_DEFAULT_SECS;  // beebo: default window
  }
  _board.batt_present = constrain(_board.batt_present, (uint8_t)BATT_PRESENT_UNKNOWN, (uint8_t)BATT_PRESENT_YES);
  if (_board.adc_resolution_bits != 10 && _board.adc_resolution_bits != 12) {
    _board.adc_resolution_bits = 12;  // beebo: default resolution
  }
  // beebo: dedup_window_ms has NO "0 = default" resolution, unlike every
  // field above -- 0 is a real, literal value here (disables live-eviction
  // counting entirely, see SimpleMeshTables.h's setDedupWindowMs()
  // comment), so a device that's never explicitly configured it (0 from
  // NodePrefs' zero-fill) simply boots with it disabled. Explicit range
  // validation lives in the SET_DEDUP_WINDOW command handler instead.
}

// beebo: DoS/QoS audit follow-up -- SimpleMeshTables caches its dedup window
// rather than reading _role_state->prefs/isRepeater() live like rx_delay_base/
// airtime_factor's own call sites do, so it needs an explicit push whenever
// the role-appropriate value could have changed: boot, a prefs reload,
// either dedup-window SET handler, and every node.role hot switch.
void Beebo::pushActiveDedupWindow() {
  uint32_t ms = _role_state->prefs.dedup_window_ms;
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) ms = _role_state->prefs.dedup_window_ms;
#endif
  ((SimpleMeshTables*)getTables())->setDedupWindowMs(ms);
}

// beebo: push the radio-affecting prefs into the radio driver and board FEM —
// the one place a new radio pref must be added to take effect on (re)load.
void Beebo::applyRadioPrefs() {
  radio_driver.setParams(_role_state->prefs.freq, _role_state->prefs.bw, _role_state->prefs.sf, _role_state->prefs.cr);
  radio_driver.setTxPower(_role_state->prefs.tx_power_dbm);
  radio_driver.setRxBoostedGainMode(_role_state->prefs.rx_boosted_gain);
  board.setLoRaFemLnaEnabled(_role_state->prefs.radio_fem_rxgain);
  board.setAdcMultiplier(_board.adc_multiplier);
  board.setAdcResolution(_board.adc_resolution_bits);
}

// beebo: allocate the monitor ring. Called last in setup(), after the
// transports are up, so it sizes from the PSRAM that's genuinely spare (this
// build routes WiFi/LWIP through PSRAM). Takes a fraction of free PSRAM, capped,
// so it never exceeds the ~2 MB total and always leaves the rest free. Still a
// one-shot setup-time allocation — never freed, never in the loop.
void Beebo::initMonRing() {
  size_t free_ps = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t want = free_ps / MONRING_FREE_FRACTION;   // leave the rest of PSRAM free
  if (want > MONRING_MAX_BYTES) want = MONRING_MAX_BYTES;
  want &= ~(MONRING_ALIGN_BYTES - 1);   // round down to a page-aligned size
  uint8_t *ring = (want >= MONRING_MIN_BYTES)
                  ? (uint8_t *)heap_caps_malloc(want, MALLOC_CAP_SPIRAM) : NULL;
  // beebo: not radioIsIdle()-verified at this point in boot -- reset the
  // trend anchor/state instead of seeding it directly, so updateBattTrend()
  // won't classify against it until a confirmed-idle sample replaces it.
  resetBattTrendRef(_batt_state, _cached_batt_mv, _board.batt_present);
  if (ring != NULL && monring.init(ring, want, (uint32_t)getRTCClock()->getCurrentTime(),
                                    buildRadioRecord(), buildEnvRecord())) {
    monring.setConfig(_role_state->prefs.monring_config);  // apply persisted enable + per-kind capture mask
    monring.setEventTypeMask(_role_state->prefs.monring_event_mask);  // apply persisted per-event-type capture mask
    MESH_DEBUG_PRINTLN("MonRing: %u records (%u KB PSRAM), %u KB PSRAM free after",
                       monring.capacity(), (unsigned)(want / 1024),
                       (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
  } else {
    if (ring != NULL) heap_caps_free(ring);
    MESH_DEBUG_PRINTLN("MonRing: PSRAM unavailable (%u KB free), capture disabled",
                       (unsigned)(free_ps / 1024));
  }
  // beebo: wire the flood-echo side's
  // EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT emission to this now-(maybe-)allocated ring.
  // Unconditional even if allocation failed above: monring's own
  // enabled()/allocated() checks already make appendEvent() a safe no-op,
  // and this only ever needs to be set once at boot.
  ((SimpleMeshTables*)getTables())->setMonRing(&monring, getRTCClock());
  pushActiveDedupWindow();
  // beebo: warm the slow-stat caches so the first `status` returns instantly
  // (getStorageUsedKb() is a live block-scan; getMCUTemperature() averages four
  // ~76 ms sensor reads); loop() refreshes them thereafter. Storage total is the
  // partition size — constant, so read once here.
  _fs_total_kb = _store->getStorageTotalKb();
  _fs_used_kb = _store->getStorageUsedKb();
  _mcu_temp_scaled = (int16_t)(board.getMCUTemperature() * 10);
  _next_slowstat_refresh = futureMillis(SLOWSTAT_REFRESH_MS);

  tune_controller.begin();
  _next_tune_tick = futureMillis(TUNE_TICK_INTERVAL_MS);
}

// beebo: snapshot the current radio config into a RadioRecord. Shared by the
// monring.noteRadio() capture sites (diffed against the ring's running state)
// and the monring.init()/monring.clear() call sites (which seed that running
// state directly), so all build the record identically and can't drift apart.
// beebo: true once the radio has been quiet for IDLE_MARGIN ms -- general-
// purpose idle probe (not RX/TX-specific), used to keep the periodic Vbat
// sample from landing mid RX/TX (or too soon after, while an IR-drop sag is
// still recovering) and skewing the trend classifier. Also worth capturing
// into the Vbat history trace itself so we can chart Vbat against idle/busy
// state and tune IDLE_MARGIN from real recovery data instead of guessing --
// starting point of 100ms, to be retuned once real recovery traces are in.
// Always IDLE_MARGIN_MS (BattTrend.h) -- not runtime-overridable.
bool Beebo::radioIsIdle() const {
  return !_radio->isReceiving() && !isTransmitting()
         && millisHasNowPassed(_last_radio_active_ms + IDLE_MARGIN_MS);
}

RadioRecord Beebo::buildRadioRecord() {
  RadioRecord radio{};
  radio.freq = (uint32_t)(_role_state->prefs.freq * 1000000.0f + 0.5f);
  radio.sf = _role_state->prefs.sf;
  radio.bw = (uint16_t)(_role_state->prefs.bw * 10.0f + 0.5f);
  radio.cr = _role_state->prefs.cr;
  radio.tx_power = (int8_t)_role_state->prefs.tx_power_dbm;
  if (_role_state->prefs.rx_boosted_gain)  radio.flags |= RADIO_FLAG_RXBOOST;
  if (_role_state->prefs.radio_fem_rxgain) radio.flags |= RADIO_FLAG_FEMRXGAIN;
  return radio;
}

// beebo: snapshot the current env sample into an EnvRecord. noise_floor,
// free_heap, temp_c, pool_free, tx_queue and err_flags are all cheap member/
// cached reads (no radio I/O, no blocking delay) so they're sampled fresh
// here; batt_mv reuses the slow-timer cache since the ADC read itself blocks
// for ~10-12ms (see _cached_batt_mv). err_flags is truncated from
// Dispatcher's uint16_t to the wire struct's uint8_t; only the low 3 bits
// (ERR_EVENT_FULL/CAD_TIMEOUT/STARTRX_TIMEOUT) are ever set, so no data is
// lost. Shared by the monring.sampleEnv() capture sites and the
// monring.init()/monring.clear() call sites, same reason as buildRadioRecord().
EnvRecord Beebo::buildEnvRecord() {
  EnvRecord env{};
  env.noise_floor = (int8_t)_radio->getNoiseFloor();
  env.batt_mv = _cached_batt_mv;
  env.temp_c = (int8_t)(_mcu_temp_scaled / 10);
  env.free_heap = (uint16_t)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
  env.pool_free = (uint8_t)_mgr->getFreeCount();
  env.tx_queue = (uint8_t)_mgr->getOutboundCount((uint32_t)getRTCClock()->getCurrentTime());
  env.err_flags = (uint8_t)_err_flags;
  env.cad_busy_events = (uint8_t)getCADBusyEventCount();
  return env;
}

// beebo: writes one TuneController::Decision -- reuses the exact same
// tlvSet*/direct-NodePrefs path each param's own individual GET/SET_*
// command already uses (see BeeboRepeater.cpp's tlvSet* functions and this
// file's CMD_SET_TUNING_PARAMS handler), rather than a third
// implementation, so a live-applied tuning change can't drift from what a
// human explicitly setting that same value would produce. TUNE_RX_DELAY_BASE/
// TUNE_AIRTIME_FACTOR now write the repeater's own independent copy
// via tlvSetRxDelayBase/
// tlvSetAirtimeFactor instead of the shared companion _role_state->prefs fields
// -- the tuning optimizer only ever runs on a repeater (see loop()'s
// `isRepeater() && _tune_enabled` gate), so this is never reached for a
// companion. TUNE_INTERFERENCE_THRESHOLD never reaches here
// (TuneController::isAppliable() excludes it from should_apply).
void Beebo::applyTuneDecision(uint8_t param_id, int16_t value) {
  switch (param_id) {
    case TUNE_RX_DELAY_BASE: {
      float vf = value / 100.0f;
      uint32_t bits; memcpy(&bits, &vf, 4);
      tlvSetRxDelayBase(this, NODE_ROLE_REPEATER, bits);
      flushDirtyPrefs();
      break;
    }
    case TUNE_TX_DELAY_FACTOR: {
      float vf = value / 100.0f;
      uint32_t bits; memcpy(&bits, &vf, 4);
      tlvSetTxDelayFactor(this, NODE_ROLE_REPEATER, bits);
      flushDirtyPrefs();
      break;
    }
    case TUNE_DIRECT_TX_DELAY_FACTOR: {
      float vf = value / 100.0f;
      uint32_t bits; memcpy(&bits, &vf, 4);
      tlvSetDirectTxDelayFactor(this, NODE_ROLE_REPEATER, bits);
      flushDirtyPrefs();
      break;
    }
    case TUNE_AGC_RESET_INTERVAL:
      tlvSetAgcResetInterval(this, NODE_ROLE_REPEATER, ((uint32_t)value) * 4);
      flushDirtyPrefs();
      break;
    case TUNE_AIRTIME_FACTOR: {
      float vf = value / 100.0f;
      uint32_t bits; memcpy(&bits, &vf, 4);
      tlvSetAirtimeFactor(this, NODE_ROLE_REPEATER, bits);
      flushDirtyPrefs();
      break;
    }
    default:
      break;   // TUNE_INTERFERENCE_THRESHOLD (dead knob) or unknown -- no-op
  }
}

// beebo: Dispatcher.h hook override -- fires immediately, every single
// occurrence of an ERR_EVENT_* condition (not just the first; _err_flags
// itself is still sticky/one-shot for `beebo report`'s "Active Faults"
// check, but the MonRing event trail now records every recurrence with its
// own real timestamp). One dedicated event type per bit (EVENT_TX_POOL_FULL/
// EVENT_TX_CAD_TIMEOUT/EVENT_RX_START_TIMEOUT, MonRing.h) -- these are three
// genuinely different causes, not one "fault" with a reason bitmask.
void Beebo::logFaultEvent(uint16_t bit) {
#ifdef RX_DISPOSITION
  if (!monring.enabled() || !monring.allocated()) return;
  uint8_t event_type;
  uint32_t *cumulative;
  switch (bit) {
    case ERR_EVENT_FULL:            event_type = EVENT_TX_POOL_FULL;    cumulative = &_tx_pool_full_count;    break;
    case ERR_EVENT_CAD_TIMEOUT:     event_type = EVENT_TX_CAD_TIMEOUT;  cumulative = &_cad_timeout_count;     break;
    case ERR_EVENT_STARTRX_TIMEOUT: event_type = EVENT_RX_START_TIMEOUT; cumulative = &_rx_start_timeout_count; break;
    default: return;  // unknown bit -- nothing to log
  }
  (*cumulative)++;
  EventRecord rec{};
  rec.event_type = event_type;
  memcpy(&rec.data[0], cumulative, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
#endif
}

// beebo: Dispatcher.h hook override -- fires immediately at the point
// queueOutbound() fails. is_relay=true (a received packet's own forward
// attempt) is a no-op here: that case already has a richer, already-
// correlated trace on the packet's own MON_RX record (RXREC_QUEUE_FULL, see
// MonRing.h's EVENT_TX_QUEUE_FULL comment for why a second, poorer
// event here would be redundant). is_relay=false (self-originated send/ACK
// reply) has no other trace at all, so gets its own event.
void Beebo::logTxQueueFull(bool is_relay) {
#ifdef RX_DISPOSITION
  if (is_relay || !monring.enabled() || !monring.allocated()) return;
  uint32_t cum = _mgr->getTxQueueFullCount();
  EventRecord rec{};
  rec.event_type = EVENT_TX_QUEUE_FULL;
  memcpy(&rec.data[0], &cum, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
#endif
}

// beebo: Dispatcher.h hook override -- fires immediately at the point
// queueInbound() fails (checkRecv()'s delayed-flood scheduling queue).
void Beebo::logRxQueueFull() {
#ifdef RX_DISPOSITION
  if (!monring.enabled() || !monring.allocated()) return;
  uint32_t cum = _mgr->getRxQueueFullCount();
  EventRecord rec{};
  rec.event_type = EVENT_RX_QUEUE_FULL;
  memcpy(&rec.data[0], &cum, 4);
  monring.appendEvent(rec, (uint32_t)getRTCClock()->getCurrentTime());
#endif
}

// beebo: node link (BLE/WiFi) queue-full counters live in
// ble_interface/wifi_interface with no Dispatcher-level hook available (see
// MonRing.h's EVENT_LINK_TX_QUEUE_FULL comment) -- poll each tick and log
// one record per direction whose lifetime count rose since last checked.
// LINK_TX/LINK_RX sum BLE + WiFi (only one is ever the locked session at a
// time -- see MultiSerialInterface).
void Beebo::appendLinkQueueDropEvents() {
  uint32_t now = (uint32_t)getRTCClock()->getCurrentTime();
  uint32_t link_tx = ble_interface.getSendQueueFullCount() + wifi_interface.getSendQueueFullCount();
  uint32_t link_rx = ble_interface.getRecvQueueFullCount();

  if (link_tx != _last_link_tx_queue_full) {
    EventRecord rec{};
    rec.event_type = EVENT_LINK_TX_QUEUE_FULL;
    memcpy(&rec.data[0], &link_tx, 4);
    monring.appendEvent(rec, now);
    _last_link_tx_queue_full = link_tx;
  }
  if (link_rx != _last_link_rx_queue_full) {
    EventRecord rec{};
    rec.event_type = EVENT_LINK_RX_QUEUE_FULL;
    memcpy(&rec.data[0], &link_rx, 4);
    monring.appendEvent(rec, now);
    _last_link_rx_queue_full = link_rx;
  }
}

// beebo: log one admin-visible setting change (MON_SETTING,
// SETTING_* key -- see MonRing.h). old/new are the raw u32 LE bit pattern
// (float settings float-bit-cast, matching PrefsTlvField.get_raw/set_raw's
// own convention) so this stays generic across bool/int/float settings
// without a second encoding scheme.
void Beebo::appendSettingChangedEvent(uint8_t setting_key, uint32_t old_raw, uint32_t new_raw, uint8_t source) {
  SettingRecord rec{};
  rec.setting = setting_key;
  rec.source = source;
  rec.old_value = old_raw;
  rec.new_value = new_raw;
  monring.appendSetting(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

// beebo: log one MON_COMMAND record for a binary-protocol command -- see
// MonRing.h's comment and checkRecvFrame()'s inclusion filter (this
// function itself does no filtering; the caller decides eligibility).
void Beebo::appendCommandRunEvent(uint16_t command_id) {
  CommandRecord rec{};
  rec.source = EVENT_SOURCE_BINARY;
  memcpy(&rec.command[0], &command_id, 2);
  monring.appendCommand(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

// beebo: log one MON_COMMAND record for a text-CLI command -- see
// MonRing.h's comment and handleCommand()'s inclusion filter (this function
// itself does no filtering; the caller decides eligibility). `label` is
// truncated (not padded beyond what memset already zeroed) to the 12 bytes
// CommandRecord.command holds.
void Beebo::appendTextCommandRunEvent(const char* label) {
  CommandRecord rec{};
  rec.source = EVENT_SOURCE_TEXT_CLI;
  size_t n = strlen(label);
  if (n > 12) n = 12;
  memcpy(&rec.command[0], label, n);
  monring.appendCommand(rec, (uint32_t)getRTCClock()->getCurrentTime());
}

// beebo: the single point where either pref store actually reaches flash --
// every setter just mutates its RAM cache and marks a dirty bit (see
// Beebo.h's savePrefs). Writes only what's dirty, so a command that touched
// one store never rewrites the other, and clears each bit only on a
// successful write (a failed /com_prefs open stays dirty and retries at the
// next flush rather than silently losing the change).
void Beebo::writeDirtyPrefs() {
  // beebo: _role_state->prefs is
  // self-contained (knows its own role and its own dirty state), so this
  // is the single place that decides which file a dirty write lands in,
  // by reading _board.role -- every call site just marks
  // _role_state->prefs.dirty (via savePrefs()) without needing to know or guess
  // which file that corresponds to. cli.savePrefs() (which would write
  // stock /com_prefs) is never called here -- /com_prefs is only ever
  // read, once, as a first-boot seed (see loadRoleState()'s repeater branch).
  if (_role_state->prefs.dirty) {
#if BEEBO_ENABLE_REPEATER_ROLE
    if (_board.role == NODE_ROLE_REPEATER) {
      _store->saveBeeboRepeaterPrefs(_role_state->prefs, _board, static_cast<ComPrefs*>(&_role_state->prefs), sizeof(ComPrefs));
    } else
#endif
    {
      _store->saveBeeboCompanionPrefs(_role_state->prefs, _board, sensors.node_lat, sensors.node_lon);
    }
    _role_state->prefs.dirty = false;
  }
  // beebo: BeeboBoardPrefs unification -- role/board_password/board_name's
  // own file, genuinely untouched by the role-switch park/load handoff
  // (see BeeboBoardPrefs.h). Separate dirty bit/file from both roles' own,
  // always written here regardless of role.
  if (_board_dirty) {
    _store->saveBeeboBoardPrefs(_board);
    _board_dirty = false;
  }
}

// beebo: re-read persisted prefs from flash and re-apply the radio-affecting
// ones, discarding any RAM-only changes made while save_prefs was off. Used by
// 'set save_prefs restore' to undo a measurement sweep without a reboot.
// Drops both stores' RAM state: NodePrefs is re-read here, and the /com_prefs
// cache is invalidated so the next read reloads it from flash.
void Beebo::reloadPrefs() {
  // beebo: the per-role state store -- re-read only the live role's own
  // file, into its own resident slot -- each role has its own physically
  // separate storage now, so there's no other slot to accidentally touch
  // (the old version unconditionally reloaded companion's file into the
  // shared single slot even while repeater was live, and vice versa).
  _role_state->prefs.dirty = _board_dirty = false;
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    _store->loadBeeboRepeaterPrefs(_role_state->prefs, _board, static_cast<ComPrefs*>(&_role_state->prefs), sizeof(ComPrefs));
  } else
#endif
  {
    _store->loadBeeboCompanionPrefs(_role_state->prefs, _board, sensors.node_lat, sensors.node_lon);
  }
  _store->loadBeeboBoardPrefs(_board);
  // beebo: re-normalize -- loadBeeboBoardPrefs() writes role directly
  sanitizeNodeRole();
  // beebo: keep _role_state/self_id/cli's prefs pointer consistent with
  // _board.role in the (unlikely) case sanitizeNodeRole() just fell back
  // away from a persisted role this binary doesn't support.
  _role_state = &role_state_store[_board.role];
  self_id = _role_state->identity;
#if BEEBO_ENABLE_REPEATER_ROLE
  cli.setPrefs(&_role_state->prefs);
#endif
  clampRadioPrefs();
  applyRadioPrefs();
  pushActiveDedupWindow();
}

const char *Beebo::getNodeName() {
  return _role_state->prefs.node_name;
}
NodePrefs *Beebo::getNodePrefs() {
  return &_role_state->prefs;
}
uint32_t Beebo::getBLEPin() {
  return _active_ble_pin;
}

struct FreqRange {
  uint32_t lower_freq, upper_freq;
};

static FreqRange repeat_freq_ranges[] = {
  #ifdef ALLOWED_REPEAT_FREQ_RANGE
  ALLOWED_REPEAT_FREQ_RANGE
  #else
  { 433000, 433000 },
  { 869495, 869495 },
  { 918000, 918000 }
  #endif
};

bool Beebo::isValidClientRepeatFreq(uint32_t f) const {
  for (int i = 0; i < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]); i++) {
    auto r = &repeat_freq_ranges[i];
    if (f >= r->lower_freq && f <= r->upper_freq) return true;
  }
  return false;
}

void Beebo::startInterface(BaseSerialInterface &serial) {
  _serial = &serial;
  serial.enable();
}

int Beebo::fillMonRingFrame(uint8_t *out, uint32_t after_seq, size_t max_len, uint32_t *returned, bool first_page, bool reset) {
  int i = 0;
  out[i++] = RESP_CODE_BEEBO;
  out[i++] = BEEBO_RESP_MONRING;
  out[i++] = monring.enabled() ? 1 : 0;
  uint32_t cap = monring.capacity(), cnt = monring.count();
  uint32_t oldest = monring.oldestSeq(), next = monring.nextSeq();
  memcpy(&out[i], &cap, 4); i += 4;
  memcpy(&out[i], &cnt, 4); i += 4;
  memcpy(&out[i], &oldest, 4); i += 4;
  memcpy(&out[i], &next, 4); i += 4;
  // beebo: monitoring-window scalars for `monitor` stats, all reflecting only
  // what is CURRENTLY resident in the ring (not lifetime totals — evicted
  // records are excluded): start/end = epoch of the oldest/newest resident
  // capture (0 until the first), so the client renders an absolute window +
  // elapsed; both are capture-time so a paused ring does not drift.
  // rx/tx/sync/radio/env/batt/tune/event/setting/command = resident per-kind
  // record counts. Sent in every header, once per request/page.
  uint32_t start = monring.startTime(), end = monring.endTime();
  uint32_t rx_count = monring.rxCount(), tx_count = monring.txCount();
  uint32_t sync_count = monring.syncCount(), radio_count = monring.radioCount();
  uint32_t env_count = monring.envCount(), batt_count = monring.battCount();
  uint32_t tune_count = monring.tuneCount(), event_count = monring.eventCount();
  uint32_t setting_count = monring.settingCount(), command_count = monring.commandCount();
  memcpy(&out[i], &start, 4); i += 4;
  memcpy(&out[i], &end, 4); i += 4;
  memcpy(&out[i], &rx_count, 4); i += 4;
  memcpy(&out[i], &tx_count, 4); i += 4;
  memcpy(&out[i], &sync_count, 4); i += 4;
  memcpy(&out[i], &radio_count, 4); i += 4;
  memcpy(&out[i], &env_count, 4); i += 4;
  memcpy(&out[i], &batt_count, 4); i += 4;
  memcpy(&out[i], &tune_count, 4); i += 4;
  memcpy(&out[i], &event_count, 4); i += 4;
  memcpy(&out[i], &setting_count, 4); i += 4;
  memcpy(&out[i], &command_count, 4); i += 4;
  // Lifetime RX-drop counters -- the two dispositions with no MON_RX record
  // to reconstruct from a download (see MonRing::bumpRxDropCount()). Every
  // other RX/TX condition breakdown was removed from this header; recreate
  // it offline via `beebo monitor download` instead.
  uint32_t rx_pool_exhausted = monring.rxPoolExhaustedCount();
  uint32_t rx_parse_error = monring.rxParseErrorCount();
  memcpy(&out[i], &rx_pool_exhausted, 4); i += 4;
  memcpy(&out[i], &rx_parse_error, 4); i += 4;
  // Lifetime queue-full drop counters -- these live in their owning class
  // (PacketManager / SerialBLEInterface / SerialWifiInterface), read fresh
  // here rather than mirrored into MonRing. LINK_TX/LINK_RX sum BLE + WiFi.
  uint32_t mesh_tx_queue_full = _mgr->getTxQueueFullCount();
  uint32_t mesh_rx_queue_full = _mgr->getRxQueueFullCount();
  uint32_t link_tx_queue_full = ble_interface.getSendQueueFullCount() + wifi_interface.getSendQueueFullCount();
  uint32_t link_rx_queue_full = ble_interface.getRecvQueueFullCount();
  memcpy(&out[i], &mesh_tx_queue_full, 4); i += 4;
  memcpy(&out[i], &mesh_rx_queue_full, 4); i += 4;
  memcpy(&out[i], &link_tx_queue_full, 4); i += 4;
  memcpy(&out[i], &link_rx_queue_full, 4); i += 4;
  // beebo: QoS ("confirm ratio" -- delivery quality, what the tuning
  // optimizer maximizes) and SoH (is this node's own infrastructure
  // intact) computed fresh from the same lifetime counters above, every
  // header request -- not gated by tuning being enabled or role, unlike
  // the repeater-only tuning tick's own reward computation (which now
  // delegates to the same MonRing::computeQos() this uses, see
  // TuneController.h). 0-10000 scaled, same convention as TuneRecord.
  // reward_before. ros_count is the companion raw-volume half of the
  // goodput reward redesign (DYNAMIC_OPTIMIZER_PLAN.md, 2026-08-24) -- QoS
  // alone can't tell a node routing 1 packet/hour at 100% from one routing
  // 1000/hour at 100%, this can. rx_drop is no longer part of QoS's
  // denominator (see MonRing::computeQos()'s own comment) -- it remains a
  // SoH input only, via the counters already gathered above.
  MonRing::QosStats qos_stats;
  qos_stats.ack_success_count = getAckSuccessCount();
  qos_stats.ack_timeout_count = getAckTimeoutCount();
  qos_stats.echo_attempt_count = ((SimpleMeshTables*)getTables())->getEchoAttemptCount();
  qos_stats.echo_success_count = ((SimpleMeshTables*)getTables())->getEchoSuccessCount();
  MonRing::SohStats soh_stats;
  soh_stats.tx_pool_full_count = _tx_pool_full_count;
  soh_stats.rx_pool_exhausted_count = rx_pool_exhausted;
  soh_stats.tx_cad_timeout_count = _cad_timeout_count;
  soh_stats.rx_start_timeout_count = _rx_start_timeout_count;
  soh_stats.tx_queue_full_count = mesh_tx_queue_full;
  soh_stats.rx_queue_full_count = mesh_rx_queue_full;
  soh_stats.link_tx_queue_full_count = link_tx_queue_full;
  soh_stats.link_rx_queue_full_count = link_rx_queue_full;
  soh_stats.tx_ack_overflow_count = getAckOverflowCount();
  soh_stats.tx_echo_overflow_count = ((SimpleMeshTables*)getTables())->getEchoOverflowCount();
  soh_stats.rx_activity_count = getNumRecvFlood() + getNumRecvDirect();
  soh_stats.tx_activity_count = getNumSentFlood() + getNumSentDirect();
  // Raw SoH-input counters not otherwise in this header (tx_pool_full/
  // cad_timeout/rx_start_timeout aren't -- those live in Beebo, not MonRing/
  // PacketManager/transports, unlike everything above), plus
  // rx_dedup_table_full_count (excluded from SoH itself, but still a salient
  // diagnostic counter -- see MonRing.h's EVENT_RX_DEDUP_TABLE_FULL comment;
  // gated to only count evictions still within the live window, see
  // SimpleMeshTables::hasSeen()) -- sent so `beebo check` can show them
  // without a full ring download.
  uint32_t rx_dedup_table_full = ((SimpleMeshTables*)getTables())->getDedupEvictedCount();
  memcpy(&out[i], &_tx_pool_full_count, 4); i += 4;
  memcpy(&out[i], &_cad_timeout_count, 4); i += 4;
  memcpy(&out[i], &_rx_start_timeout_count, 4); i += 4;
  memcpy(&out[i], &soh_stats.tx_ack_overflow_count, 4); i += 4;
  memcpy(&out[i], &soh_stats.tx_echo_overflow_count, 4); i += 4;
  memcpy(&out[i], &rx_dedup_table_full, 4); i += 4;
  // beebo: repeater-role allowPacketForward()'s three flood-decline causes
  // (see EVENT_MAX_HOP_NO_FWD/EVENT_REGION_NO_FWD/EVENT_LOOP_NO_FWD,
  // MonRing.h) -- same "raw diagnostic counter, sent so `beebo monitor
  // status`/`beebo check` can show it without a full ring download" reason
  // as rx_dedup_table_full above; excluded from SoH for the same reason too
  // (configured policy drops, not internal faults).
  memcpy(&out[i], &_max_hop_no_fwd_count, 4); i += 4;
  memcpy(&out[i], &_region_no_fwd_count, 4); i += 4;
  memcpy(&out[i], &_loop_no_fwd_count, 4); i += 4;
  uint16_t qos = MonRing::computeQos(qos_stats);
  uint16_t soh = MonRing::computeSoh(soh_stats);
  uint32_t ros_count = MonRing::computeRos(qos_stats);
  // beebo: confirmable-attempt count behind qos's ratio -- lets the CLI
  // tell "no exposure yet" apart from "attempts happened, none succeeded"
  // (both read as qos=0 otherwise). See MonRing::computeQosExposure().
  uint32_t qos_exposure = MonRing::computeQosExposure(qos_stats);
  memcpy(&out[i], &qos, 2); i += 2;
  memcpy(&out[i], &soh, 2); i += 2;
  memcpy(&out[i], &ros_count, 4); i += 4;
  memcpy(&out[i], &qos_exposure, 4); i += 4;
  int rec_hdr = i;
  i += 2;  // reserve returned-count (total: real + injected)
  int injected_hdr = i;
  i += 1;  // reserve injected-count byte (0-3): lets a paged reader (BLE/USB,
           // no BULK_XFER) subtract only the real records from its cursor too
  // beebo: 1 if the caller's requested after_seq was overridden because the
  // client-supplied after_ts predates the ring's current oldest resident
  // record (reboot/clear/wraparound since it last read) -- see
  // BEEBO_CMD_GET_MONRING. Known upfront (unlike returned/injected), so
  // written immediately rather than reserved-and-backfilled.
  out[i++] = reset ? 1 : 0;
  *returned = 0;
  size_t room = (max_len > (size_t)i) ? (max_len - (size_t)i) : 0;  // never underflow
  // First page of a read: walk the fixed sync/radio/env prefix that must
  // precede any RX/TX, one slot at a time, against the real record stream
  // starting at `from`. A slot whose real record is already there (matched
  // in order) needs nothing extra — serialize() below will emit it as-is.
  // A slot that's missing (evicted, or genuinely never written) gets its
  // stashed start-ref spliced in instead. This never skips or duplicates a
  // real record: the peek cursor only advances on an actual slot match, and
  // serialize() always starts at the untouched `from`. Continuations must
  // not repeat this — the caller has already seen these references by then.
  uint32_t injected = 0;
  if (first_page) {
    uint32_t from = (after_seq > monring.oldestSeq()) ? after_seq : monring.oldestSeq();
    uint32_t peek_pos = from;
    MonRecord live[3];
    size_t n = 0;
    MonRecord rec;
    static const uint8_t kSlotKind[3] = { MON_SYNC, MON_RADIO, MON_ENV };
    for (int s = 0; s < 3; s++) {
      if (monring.peek(peek_pos, &rec) && rec.kind == kSlotKind[s]) {
        peek_pos++;  // real record covers this slot; nothing to inject
      } else if (monring.emitStartRef(kSlotKind[s], &live[n])) {
        n++;
      }
    }
    size_t avail = room / sizeof(MonRecord);
    size_t take = (n < avail) ? n : avail;
    memcpy(&out[i], live, take * sizeof(MonRecord));
    i += take * sizeof(MonRecord);
    room -= take * sizeof(MonRecord);
    injected = (uint32_t)take;
  }
  i += monring.serialize(&out[i], room, after_seq, returned);  // *returned = real ring record count only
  // *returned stays ring-only: callers use it to advance the after_seq cursor,
  // and injected records are synthetic (not real ring seqs) — folding them in
  // here would overshoot the cursor.
  uint32_t total = *returned + injected;  // real + injected: what the client must parse out of this frame
  out[rec_hdr + 0] = total & 0xFF;
  out[rec_hdr + 1] = (total >> 8) & 0xFF;
  out[injected_hdr] = (uint8_t)injected;
  return i;
}

// beebo: NodePrefs-backed PREFS_TLV_FIELDS entries -- WIFI_SSID/
// TRANSPORT_CONFIG/MONRING_CONFIG. Called by both PREFS_TLV_FIELDS
// (BeeboRepeater.cpp) and their matching individual BEEBO_CMD_GET_X/SET_X
// handler above, same one-implementation-two-call-sites pattern as the
// ComPrefs fields (see Beebo.h's PrefsTlvKey comment). set_raw never
// reboots and never sets a *_pending live-apply flag itself -- the
// individual handlers do that themselves right after calling it, since
// "signal main.cpp to apply this live" is specific to how each of those two
// handlers is invoked, not a property of the stored value.
// beebo: live-apply side effects (marking the RIGHT role's NodePrefs
// dirty) only make sense when `role` is also the currently-live one; a
// write targeting a non-live role's slot persists immediately via
// persistRoleSlot() instead, same convention as every other role-targeted
// setter in this file.
int Beebo::tlvGetWifiSsid(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
  const char* ssid = self->role_state_store[role].prefs.wifi_ssid;
  size_t n = strlen(ssid);
  if (n > max_len) n = max_len;
  memcpy(out, ssid, n);
  return (int)n;
}
bool Beebo::tlvSetWifiSsid(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.wifi_ssid) - 1) len = sizeof(slot.prefs.wifi_ssid) - 1;
  memcpy(slot.prefs.wifi_ssid, in, len);
  slot.prefs.wifi_ssid[len] = '\0';
  if (role == self->_board.role) {
    self->savePrefs();
    self->_wifi_creds_pending = true;
  } else {
    persistRoleSlot(self, role, slot);
  }
  return true;
}

// beebo: the role-targeted path -- role-targeted binary
// access to node.transport.*/node.wifi.*/node.ble.pin, needed because
// SET_WIFI_CREDS/CMD_SET_DEVICE_PIN above are live-role-scoped by design
// (same as their text-CLI equivalents) with no way to reach the non-live
// role's own independent copy -- `beebo config pull/push --role` silently
// mislabeled the live role's data as the target role's for these fields
// until this addition. Each function below targets role_state_store[role]
// explicitly. Live-apply (the RAM-shadow/hardware-facing side effects
// CMD_SET_DEVICE_PIN's own handler performs) only happens when `role`
// also happens to be the currently-live role -- writing to a non-live
// role's slot just persists, taking effect the next time that role
// actually becomes live, same convention as every other role-targeted
// setter (tlvSetMultiAcks etc. above).
//
uint32_t Beebo::tlvGetTransportConfig(Beebo* self, uint8_t role) {
  BeeboPrefs& p = self->role_state_store[role].prefs;
  return (uint32_t)p.ble_enabled | ((uint32_t)p.tcp_enabled << 8) | ((uint32_t)p.usb_enabled << 16);
}
bool Beebo::tlvSetTransportConfig(Beebo* self, uint8_t role, uint32_t raw) {
  BeeboRoleState& slot = self->role_state_store[role];
  bool old_ble = slot.prefs.ble_enabled != 0;
  bool old_tcp = slot.prefs.tcp_enabled != 0;
  bool new_ble = (raw & 0xFF) != 0;
  bool new_tcp = ((raw >> 8) & 0xFF) != 0;
  slot.prefs.usb_enabled = ((raw >> 16) & 0xFF) ? 1 : 0;
  if (new_ble && new_tcp) {
    // Mutually exclusive -- clear whichever one this write didn't just turn
    // on, so the caller's actual intent wins instead of a fixed tcp/ble
    // preference (a caller toggling only one of the two, the common case,
    // always sends the other back unchanged). If both are newly turning on
    // in the same write, or neither changed (both already on -- a stale
    // payload), fall back to keeping tcp.
    bool ble_turning_on = new_ble && !old_ble;
    bool tcp_turning_on = new_tcp && !old_tcp;
    if (ble_turning_on && !tcp_turning_on) {
      new_tcp = false;
    } else {
      new_ble = false;
    }
  }
  slot.prefs.ble_enabled = new_ble ? 1 : 0;
  slot.prefs.tcp_enabled = new_tcp ? 1 : 0;
  bool remote_off = !slot.prefs.ble_enabled
      && !(slot.prefs.tcp_enabled && slot.prefs.wifi_ssid[0] != '\0');
  if (remote_off) slot.prefs.usb_enabled = 1;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) self->_transport_config_pending = true;
  return true;
}

bool Beebo::tlvGetWifiPwdSet(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.wifi_pwd[0] != '\0';
}
// TLV_STRING get_str for PREFS_TLV_WIFI_PWD: never returns the plaintext
// password (matches GET_WIFI_PWD_SET/GET_*_WIFI_PWD_SET, which only ever
// hand back "is it set"), just the same boolean packed as a 1-byte string
// so the field can share PREFS_TLV_WIFI_PWD's key with its (real,
// plaintext) set_str counterpart below.
int Beebo::tlvGetWifiPwdSetStr(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
  out[0] = tlvGetWifiPwdSet(self, role) ? 1 : 0;
  return 1;
}
// Password-only counterpart to tlvSetWifiSsid -- writes wifi_pwd without
// touching wifi_ssid, so PREFS_TLV_WIFI_SSID/PREFS_TLV_WIFI_PWD can be set
// independently in the same batch or separately. Unlike
// BEEBO_CMD_SET_WIFI_CREDS's combined ssid\0pwd\0 payload, an empty value
// here clears the password rather than leaving it unchanged -- with
// separate keys, the caller simply omits the key it doesn't want to touch.
bool Beebo::tlvSetWifiPwd(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.wifi_pwd) - 1) len = sizeof(slot.prefs.wifi_pwd) - 1;
  memcpy(slot.prefs.wifi_pwd, in, len);
  slot.prefs.wifi_pwd[len] = '\0';
  if (role == self->_board.role) {
    self->savePrefs();
    self->_wifi_creds_pending = true;
  } else {
    persistRoleSlot(self, role, slot);
  }
  return true;
}
// beebo: same write-only shape as tlvSetWifiPwd -- get_str never returns
// the plaintext, only a 1-byte "is it set" flag (non-default for
// password, non-empty for guest_password, which has no compiled-in
// default). Always role=NODE_ROLE_REPEATER in practice (ComPrefs only
// exists there); guarded by BEEBO_ENABLE_REPEATER_ROLE since these fields
// don't exist at all in a companion-only static build.
int Beebo::tlvGetRepeaterPasswordSetStr(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  out[0] = strcmp(self->role_state_store[role].prefs.password, ADMIN_PASSWORD) != 0 ? 1 : 0;
#else
  out[0] = 0;
#endif
  return 1;
}
bool Beebo::tlvSetRepeaterPassword(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.password) - 1) len = sizeof(slot.prefs.password) - 1;
  memcpy(slot.prefs.password, in, len);
  slot.prefs.password[len] = '\0';
  persistRoleSlot(self, role, slot);
#endif
  return true;
}
int Beebo::tlvGetGuestPasswordSetStr(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  out[0] = self->role_state_store[role].prefs.guest_password[0] != 0 ? 1 : 0;
#else
  out[0] = 0;
#endif
  return 1;
}
bool Beebo::tlvSetGuestPassword(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.guest_password) - 1) len = sizeof(slot.prefs.guest_password) - 1;
  memcpy(slot.prefs.guest_password, in, len);
  slot.prefs.guest_password[len] = '\0';
  persistRoleSlot(self, role, slot);
#endif
  return true;
}
// beebo: BeeboBoardPrefs fields -- one value for the whole device, `role`
// is accepted (every PrefsTlvField accessor has the same signature) but
// ignored; always touches self->_board regardless of what role byte the
// caller passed. Same persistence mechanism as SET_OWNER_PASSWORD/
// SET_BOARD_NAME used before retirement (_board_dirty + flushDirtyPrefs()),
// not persistRoleSlot() -- board.role/board_password/board_name live
// outside role_state_store entirely.
int Beebo::tlvGetBoardName(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
  size_t n = strlen(self->_board.board_name);
  if (n > max_len) n = max_len;
  memcpy(out, self->_board.board_name, n);
  return (int)n;
}
bool Beebo::tlvSetBoardName(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
  if (len > sizeof(self->_board.board_name) - 1) len = sizeof(self->_board.board_name) - 1;
  memcpy(self->_board.board_name, in, len);
  self->_board.board_name[len] = '\0';
  self->_board_dirty = true;
  return true;
}
int Beebo::tlvGetOwnerPasswordSetStr(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
  out[0] = self->_board.board_password[0] != 0 ? 1 : 0;
  return 1;
}
bool Beebo::tlvSetOwnerPassword(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
  if (len > sizeof(self->_board.board_password) - 1) len = sizeof(self->_board.board_password) - 1;
  memcpy(self->_board.board_password, in, len);
  self->_board.board_password[len] = '\0';
  self->_board_dirty = true;
  return true;
}
uint32_t Beebo::tlvGetBlePin(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.ble_pin;
}
bool Beebo::tlvSetBlePin(Beebo* self, uint8_t role, uint32_t pin) {
  if (pin != 0 && (pin < 100000 || pin > 999999)) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.ble_pin = pin;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    // beebo: same live-apply as CMD_SET_DEVICE_PIN's own handler -- only
    // meaningful when this write also targets the currently-live role.
    self->_active_ble_pin = (pin == 0) ? BLE_PIN_CODE : pin;
    self->ble_interface.setPinCode(self->_active_ble_pin);
  }
  return true;
}

// beebo: monring.setConfig() is a live hardware-facing side effect (the
// running MonRing instance's active config), so it only fires when `role`
// is also the currently-live role -- writing a non-live role's slot just
// persists, taking effect the next time that role becomes live
// (initMonRing() reads it back in then).
uint32_t Beebo::tlvGetMonringConfig(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.monring_config;
}
bool Beebo::tlvSetMonringConfig(Beebo* self, uint8_t role, uint32_t raw) {
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.monring_config = (uint8_t)raw;
  if (role == self->_board.role) {
    self->monring.setConfig(slot.prefs.monring_config);
    self->savePrefs();
  } else {
    persistRoleSlot(self, role, slot);
  }
  return true;
}

// beebo: per-event-type MON_EVENT capture bitmask (bit N = MonRing.h's
// EVENT_* id N), now persisted per role like monring_config above --
// previously RAM-only via the individual GET/SET_MONRING_EVENT_MASK
// opcodes (reset to all-1s/capture-everything every boot); folded into
// PREFS_TLV so it survives reboot per role, same live-vs-parked-slot
// side-effect rule as tlvSetMonringConfig.
uint32_t Beebo::tlvGetMonringEventMask(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.monring_event_mask;
}
bool Beebo::tlvSetMonringEventMask(Beebo* self, uint8_t role, uint32_t raw) {
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.monring_event_mask = raw;
  if (role == self->_board.role) {
    self->monring.setEventTypeMask(raw);
    self->savePrefs();
  } else {
    persistRoleSlot(self, role, slot);
  }
  return true;
}

// beebo: the role-targeted path -- writes into a specific
// role's own resident slot (role_state_store[role]), which
// may not be the currently-live role. flushDirtyPrefs() (called by every
// opcode handler right after one of these setters) only ever flushes
// _role_state->prefs -- the LIVE role's slot -- so a write into a
// currently-non-live role's slot must be persisted immediately here
// instead of just setting .dirty and trusting the caller's flush.
void Beebo::persistRoleSlot(Beebo* self, uint8_t role, BeeboRoleState& slot) {
  if (&slot == self->_role_state) {
    slot.prefs.dirty = true;
    return;
  }
  if (role == NODE_ROLE_COMPANION) {
    self->_store->saveBeeboCompanionPrefs(slot.prefs, self->_board, sensors.node_lat, sensors.node_lon);
  }
#if BEEBO_ENABLE_REPEATER_ROLE
  else if (role == NODE_ROLE_REPEATER) {
    self->_store->saveBeeboRepeaterPrefs(slot.prefs, self->_board, static_cast<ComPrefs*>(&slot.prefs), sizeof(ComPrefs));
  }
#endif
}

// beebo: repeater's own independent multi_acks/path_hash_mode/lat/lon
// (see Beebo.h's PrefsTlvKey comment). BeeboPrefs already carries these
// bytes (same struct shape as
// companion's, ComPrefs is #define NodePrefs ComPrefs) -- this is
// "expose bytes the file format already has room for", not new storage.
// Clamping mirrors those stock entry points: multi_acks constrained 0/1
// (CommonCLI.cpp's own "multi.acks" setter does this too); path_hash_mode
// rejects >=3; lat/lon reject out-of-range, same bounds as
// CMD_SET_ADVERT_LATLON. Targets whichever role's slot the caller names
// explicitly -- NOT self->_role_state->prefs (whichever role is currently
// live).
//
// beebo: deliberately NOT gated by `#if BEEBO_ENABLE_REPEATER_ROLE` (unlike
// some of this file's other genuinely repeater-only functions): these five
// are called with either role. role_state_store[] always exists for both
// roles on every build and persistRoleSlot() already branches safely per
// role internally, so gating the whole body on BEEBO_ENABLE_REPEATER_ROLE
// would silently no-op the companion path too on a companion-only static
// build.
uint32_t Beebo::tlvGetMultiAcks(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.multi_acks;
}
bool Beebo::tlvSetMultiAcks(Beebo* self, uint8_t role, uint32_t raw) {
  return persistScalarField(self, role, self->role_state_store[role].prefs.multi_acks, raw ? 1 : 0);
}

uint32_t Beebo::tlvGetPathHashMode(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.path_hash_mode;
}
bool Beebo::tlvSetPathHashMode(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw >= 3) return false;
  return persistScalarField(self, role, self->role_state_store[role].prefs.path_hash_mode, raw);
}

uint32_t Beebo::tlvGetLat(Beebo* self, uint8_t role) {
  int32_t lat = (int32_t)(self->role_state_store[role].prefs.node_lat * 1000000.0);
  uint32_t raw; memcpy(&raw, &lat, 4);
  return raw;
}
bool Beebo::tlvSetLat(Beebo* self, uint8_t role, uint32_t raw) {
  int32_t lat; memcpy(&lat, &raw, 4);
  if (lat > 90 * 1000000 || lat < -90 * 1000000) return false;
  return persistScalarField(self, role, self->role_state_store[role].prefs.node_lat, ((double)lat) / 1000000.0);
}

uint32_t Beebo::tlvGetLon(Beebo* self, uint8_t role) {
  int32_t lon = (int32_t)(self->role_state_store[role].prefs.node_lon * 1000000.0);
  uint32_t raw; memcpy(&raw, &lon, 4);
  return raw;
}
bool Beebo::tlvSetLon(Beebo* self, uint8_t role, uint32_t raw) {
  int32_t lon; memcpy(&lon, &raw, 4);
  if (lon > 180 * 1000000 || lon < -180 * 1000000) return false;
  return persistScalarField(self, role, self->role_state_store[role].prefs.node_lon, ((double)lon) / 1000000.0);
}

// beebo: repeater's own independent advert_loc_policy, same pattern as
// multi_acks/path_hash_mode above. Companion's own copy is untouched --
// still reachable via stock
// CMD_SET_OTHER_PARAMS. Clamping mirrors CommonCLI.cpp's own "gps advert"
// setter (0-2), which is compiled out on beebo since ENV_INCLUDE_GPS=0 --
// this opcode is the repeater role's only reachable way to view/change it.
uint32_t Beebo::tlvGetAdvLocPolicy(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.advert_loc_policy;
}
bool Beebo::tlvSetAdvLocPolicy(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw >= 3) return false;
  return persistScalarField(self, role, self->role_state_store[role].prefs.advert_loc_policy, raw);
}

// beebo: board/battery BeeboBasePrefs fields (see Beebo.h's PrefsTlvKey
// comment) -- same role-generic, not-gated-by-BEEBO_ENABLE_REPEATER_ROLE
// pattern as multi_acks/lat/lon above, since BeeboBasePrefs is always
// compiled for both roles. board/radio_driver calls only happen when role
// is the currently-live one, matching what the (now-retired) individual
// opcodes did.
uint32_t Beebo::tlvGetRadioFemRxgain(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.radio_fem_rxgain;
}
bool Beebo::tlvSetRadioFemRxgain(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw > 1) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.radio_fem_rxgain = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role && board.canControlLoRaFemLna()) {
    if (board.setLoRaFemLnaEnabled(raw != 0)) radio_driver.resetAGC();
  }
  return true;
}

uint32_t Beebo::tlvGetRadioRxgain(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.rx_boosted_gain;
}
bool Beebo::tlvSetRadioRxgain(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw > 1) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.rx_boosted_gain = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setRxBoostedGainMode(raw);
    radio_driver.resetAGC();
  }
  return true;
}

// beebo: LoRa modulation params -- role-generic NodePrefs fields, same
// mirror-into-hardware-only-if-live convention as radio_fem_rxgain/
// radio_rxgain above. Range checks mirror CMD_SET_RADIO_PARAMS's own
// (Hz-based there; these fields are already stored in the prefs' own
// units -- MHz/kHz -- so the bounds are that same range divided by 1000).
// Unlike CMD_SET_RADIO_PARAMS (which rewrites freq/bw/sf/cr atomically in
// one opcode), each of these is set independently -- re-sending all four
// current values to radio_driver.setParams() on every one of them (rather
// than tracking "did anything actually change") is deliberately the same
// unconditional resend set_radio()/_set_one() has always done; the radio
// driver re-applying identical params it already has is a no-op in
// practice, and matching that existing shape means no new "did this
// actually change" bug can be introduced here.
uint32_t Beebo::tlvGetRadioFreq(Beebo* self, uint8_t role) {
  uint32_t raw; float v = self->role_state_store[role].prefs.freq;
  memcpy(&raw, &v, 4);
  return raw;
}
bool Beebo::tlvSetRadioFreq(Beebo* self, uint8_t role, uint32_t raw) {
  float v; memcpy(&v, &raw, 4);
  if (v < 150.0f || v > 2500.0f) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.freq = v;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setParams(slot.prefs.freq, slot.prefs.bw, slot.prefs.sf, slot.prefs.cr);
  }
  return true;
}

uint32_t Beebo::tlvGetRadioBw(Beebo* self, uint8_t role) {
  uint32_t raw; float v = self->role_state_store[role].prefs.bw;
  memcpy(&raw, &v, 4);
  return raw;
}
bool Beebo::tlvSetRadioBw(Beebo* self, uint8_t role, uint32_t raw) {
  float v; memcpy(&v, &raw, 4);
  if (v < 7.0f || v > 500.0f) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.bw = v;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setParams(slot.prefs.freq, slot.prefs.bw, slot.prefs.sf, slot.prefs.cr);
  }
  return true;
}

uint32_t Beebo::tlvGetRadioSf(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.sf;
}
bool Beebo::tlvSetRadioSf(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw < 5 || raw > 12) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.sf = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setParams(slot.prefs.freq, slot.prefs.bw, slot.prefs.sf, slot.prefs.cr);
  }
  return true;
}

uint32_t Beebo::tlvGetRadioCr(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.cr;
}
bool Beebo::tlvSetRadioCr(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw < 5 || raw > 8) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.cr = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setParams(slot.prefs.freq, slot.prefs.bw, slot.prefs.sf, slot.prefs.cr);
  }
  return true;
}

uint32_t Beebo::tlvGetRadioTxpower(Beebo* self, uint8_t role) {
  int8_t v = self->role_state_store[role].prefs.tx_power_dbm;
  return (uint32_t)(int32_t)v;
}
bool Beebo::tlvSetRadioTxpower(Beebo* self, uint8_t role, uint32_t raw) {
  int8_t v = (int8_t)(int32_t)raw;
  if (v < -9 || v > MAX_LORA_TX_POWER) return false;
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.tx_power_dbm = v;
  persistRoleSlot(self, role, slot);
  if (role == self->_board.role) {
    radio_driver.setTxPower(slot.prefs.tx_power_dbm);
  }
  return true;
}

// beebo: BOARD_BATTERY_PREFS.md -- these seven all address self->_board
// (BeeboBoardPrefs), not role_state_store[role].prefs, and always apply
// live/persist regardless of `role`: there's one physical VBAT ADC/
// battery per board, not one per role, so a board-scoped field is always
// "live" no matter which role's TLV request wrote it -- the previous
// `if (role == self->_board.role) ...` live-apply guards only existed
// because these fields used to (incorrectly) pretend to be per-role.
uint32_t Beebo::tlvGetAdcMultiplier(Beebo* self, uint8_t role) {
  // beebo: _board.adc_multiplier is the persisted override, 0.0f meaning
  // "use board default" (see setAdcMultiplier()) -- report the resolved,
  // actually-in-effect value here instead of that raw possibly-0 field, or
  // a never-calibrated node reads back as 0 and callers computing a new
  // multiplier as a ratio against it (e.g. `beebo battery calibrate`) get
  // stuck multiplying by 0 forever.
  uint32_t raw; float v = board.getAdcMultiplier();
  memcpy(&raw, &v, 4);
  return raw;
}
bool Beebo::tlvSetAdcMultiplier(Beebo* self, uint8_t role, uint32_t raw) {
  float v; memcpy(&v, &raw, 4);
  v = constrain(v, 0.0f, 10.0f);
  self->_board.adc_multiplier = v;
  self->_board_dirty = true;
  board.setAdcMultiplier(v);
  return true;
}

uint32_t Beebo::tlvGetAdcResolution(Beebo* self, uint8_t role) {
  return self->_board.adc_resolution_bits;
}
bool Beebo::tlvSetAdcResolution(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw != 10 && raw != 12) return false;
  self->_board.adc_resolution_bits = (uint8_t)raw;
  self->_board_dirty = true;
  board.setAdcResolution((uint8_t)raw);
  return true;
}

uint32_t Beebo::tlvGetBattPresent(Beebo* self, uint8_t role) {
  return self->_board.batt_present;
}
bool Beebo::tlvSetBattPresent(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw > (uint32_t)BATT_PRESENT_YES) return false;
  return persistBoardField(self, self->_board.batt_present, raw);
}

uint32_t Beebo::tlvGetBattSamplePeriod(Beebo* self, uint8_t role) {
  return self->_board.batt_sample_period_secs;
}
bool Beebo::tlvSetBattSamplePeriod(Beebo* self, uint8_t role, uint32_t raw) {
  return persistBoardField(self, self->_board.batt_sample_period_secs, raw);
}

uint32_t Beebo::tlvGetBattSampleWindow(Beebo* self, uint8_t role) {
  return self->_board.batt_sample_window_secs;
}
bool Beebo::tlvSetBattSampleWindow(Beebo* self, uint8_t role, uint32_t raw) {
  return persistBoardField(self, self->_board.batt_sample_window_secs, raw);
}


// beebo: companion's own write-side counterparts to the role-generic
// accessors above / BEEBO_CMD_GET_COMPANION_* above -- fixes the mirror-image write gap
// documented in BUGS.md ('companion.* writes issued while repeater is the
// live role corrupt repeater's own slot'). CMD_SET_OTHER_PARAMS/
// CMD_SET_PATH_HASH_MODE/CMD_SET_ADVERT_LATLON/CMD_SET_DEVICE_NAME keep
// their existing always-live-role-targeting semantics (real legacy-app
// compatibility requirement -- see CLAUDE.md's Backward compatibility
// section); these new opcodes are the role-explicit alternative
// companion.* (prefs.py) now calls instead. Same clamping as the stock
// entry points they mirror. Unlike most of this file's other companion-only
// state, companion's BeeboCompanionPrefs is compiled into every build
// (role-agnostic struct, same as owner_password/board_name) -- no
// BEEBO_ENABLE_COMPANION_ROLE guard needed, only the runtime
// isNodeRoleBuiltIn() check callers already apply (same reasoning as
// BEEBO_CMD_GET_COMPANION_* handlers).
uint32_t Beebo::tlvGetManualAddContacts(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.manual_add_contacts;
}
bool Beebo::tlvSetManualAddContacts(Beebo* self, uint8_t role, uint32_t raw) {
  return persistScalarField(self, role, self->role_state_store[role].prefs.manual_add_contacts, raw ? 1 : 0);
}
uint32_t Beebo::tlvGetAutoaddConfig(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.autoadd_config;
}
bool Beebo::tlvSetAutoaddConfig(Beebo* self, uint8_t role, uint32_t raw) {
  return persistScalarField(self, role, self->role_state_store[role].prefs.autoadd_config, raw);
}
// beebo: companion's own client_repeat -- see PREFS_TLV_COMPANION_REPEAT's
// own comment (Beebo.h) for why this exists alongside the still-live
// CMD_SET_RADIO_PARAMS/CMD_DEVICE_QUERY path rather than replacing it.
// Enabling repeat must still be rejected outside the node's allowed
// repeat-frequency ranges here too, same as CMD_SET_RADIO_PARAMS's own
// isValidClientRepeatFreq() check just above -- this is a real regulatory
// constraint, not a formality, and this TLV path is a second, independent
// way to flip the same client_repeat field, so it needs the same guard or
// it becomes a silent bypass (caught by
// test_companion_repeat_rejects_out_of_band_frequency_fail on real
// hardware 2026-08-14).
uint32_t Beebo::tlvGetCompanionRepeat(Beebo* self, uint8_t role) {
  return self->role_state_store[role].prefs.client_repeat;
}
bool Beebo::tlvSetCompanionRepeat(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw) {
    uint32_t freq_khz = (uint32_t)(self->role_state_store[role].prefs.freq * 1000.0f);
    if (!self->isValidClientRepeatFreq(freq_khz)) return false;
  }
  return persistScalarField(self, role, self->role_state_store[role].prefs.client_repeat, raw ? 1 : 0);
}

// beebo: runs an admin command locally against this device's own live role (the 'self'
// sentinel -- see matchAdminSelfCommand() and plans/ADMIN_SELF_COMMAND.md), and delivers
// the reply back to the app through the same queued-message path a real remote admin
// reply from `target` would take (Beebo::onCommandDataRecv() -> queueMessage()), so the
// app's UI sees an ordinary admin-command response with no special casing needed there.
void Beebo::handleAdminSelfCommand(const ContactInfo& target, char* command) {
  char reply[160];
  reply[0] = 0;
  uint32_t timestamp = getRTCClock()->getCurrentTimeUnique();
  handleCommand(timestamp, command, reply);
  int text_len = strlen(reply);
  if (text_len > 0) {
    queueMessage(target, TXT_TYPE_CLI_DATA, NULL, timestamp, NULL, 0, reply);
  }
}

void Beebo::handleCmdFrame(size_t len) {
  if (cmd_frame[0] == CMD_DEVICE_QUERY && len >= 2) { // sent when app establishes connection
    app_target_ver = cmd_frame[1];                    // which version of protocol does app understand
    // beebo: new session -> drop any bulk-transfer grant from a previous app, so
    // a plain app connecting next never inherits large frames / streaming.
    _app_max_tx = MAX_FRAME_SIZE;
    _app_stream = false;
    if (_monread.active) monring.resumeAfterRead();  // abandoned mid-stream: don't leave capture paused
    _monread.active = false;
    _statread.active = false;
    _neighread.active = false;
    _pathread.active = false;

    int i = 0;
    out_frame[i++] = RESP_CODE_DEVICE_INFO;
    out_frame[i++] = FIRMWARE_VER_CODE;
    out_frame[i++] = MAX_CONTACTS / 2;   // v3+
    out_frame[i++] = MAX_GROUP_CHANNELS; // v3+
    memcpy(&out_frame[i], &_role_state->prefs.ble_pin, 4);
    i += 4;
    memset(&out_frame[i], 0, 12);
    strcpy((char *)&out_frame[i], FIRMWARE_BUILD_DATE);
    i += 12;
    StrHelper::strzcpy((char *)&out_frame[i], board.getManufacturerName(), 40);
    i += 40;
    StrHelper::strzcpy((char *)&out_frame[i], FIRMWARE_VERSION, 20);
    i += 20;
    out_frame[i++] = _role_state->prefs.client_repeat;   // v9+
    // beebo: path_hash_mode is one of the 14 SharedPrefs fields (see
    // NodePrefs.h) -- no role branch needed since both roles share the field.
    out_frame[i++] = _role_state->prefs.path_hash_mode;  // v10+
    { // v14+ wifi_rssi (int16, little-endian; INT16_MIN sentinel when unavailable)
      int16_t wifi_rssi = INT16_MIN;
#if defined(ESP32)
      if (WiFi.status() == WL_CONNECTED) wifi_rssi = (int16_t) WiFi.RSSI();
#endif
      memcpy(&out_frame[i], &wifi_rssi, 2);
      i += 2;
    }
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_APP_START &&
             len >= 8) { // sent when app establishes connection, respond with node ID
    //  cmd_frame[1..7]  reserved future
    char *app_name = (char *)&cmd_frame[8];
    cmd_frame[len] = 0; // make app_name null terminated
    MESH_DEBUG_PRINTLN("App %s connected", app_name);

    _iter_started = false; // stop any left-over ContactsIterator
    _neighread.active = false; // stop any left-over GET_NEIGHBORS stream
    _pathread.active = false;  // stop any left-over GET_ADVERT_PATHS stream
    int i = 0;
    out_frame[i++] = RESP_CODE_SELF_INFO;
    // beebo: the app-facing transport (BLE/WiFi/USB)
    // is unrelated to node.role, so an app can connect to a repeater-role
    // node and needs to be told the truth here, not hardcoded ADV_TYPE_CHAT.
    out_frame[i++] = isRepeater() ? ADV_TYPE_REPEATER : ADV_TYPE_CHAT;
    out_frame[i++] = _role_state->prefs.tx_power_dbm;
    out_frame[i++] = MAX_LORA_TX_POWER;
    memcpy(&out_frame[i], self_id.pub_key, PUB_KEY_SIZE);
    i += PUB_KEY_SIZE;

    // beebo: live-role-aware, same rationale as adv_type/pubkey/name above
    // -- self_info always describes whichever role is live right now.
    // beebo: lat/lon here is the role's own PREFS-mode coordinate (what
    // CMD_SET_ADVERT_LATLON/"set lat"/"lon" write, what companion.coords/
    // repeater.coords report), not the shared SHARE-mode sensors.node_lat/
    // lon -- matches CMD_SET_ADVERT_LATLON's own field now (see that
    // handler's comment).
    int32_t lat, lon;
    uint8_t multi_acks, advert_loc_policy;
#if BEEBO_ENABLE_REPEATER_ROLE
    if (isRepeater()) {
      lat = (_role_state->prefs.node_lat * 1000000.0);
      lon = (_role_state->prefs.node_lon * 1000000.0);
      multi_acks = _role_state->prefs.multi_acks;
      advert_loc_policy = _role_state->prefs.advert_loc_policy;
    } else
#endif
    {
      lat = (_role_state->prefs.node_lat * 1000000.0);
      lon = (_role_state->prefs.node_lon * 1000000.0);
      multi_acks = _role_state->prefs.multi_acks;
      advert_loc_policy = _role_state->prefs.advert_loc_policy;
    }
    memcpy(&out_frame[i], &lat, 4);
    i += 4;
    memcpy(&out_frame[i], &lon, 4);
    i += 4;
    out_frame[i++] = multi_acks; // new v7+
    out_frame[i++] = advert_loc_policy;
    out_frame[i++] = (_role_state->prefs.telemetry_mode_env << 4) | (_role_state->prefs.telemetry_mode_loc << 2) |
                     (_role_state->prefs.telemetry_mode_base); // v5+
    out_frame[i++] = _role_state->prefs.manual_add_contacts;

    uint32_t freq = _role_state->prefs.freq * 1000;
    memcpy(&out_frame[i], &freq, 4);
    i += 4;
    uint32_t bw = _role_state->prefs.bw * 1000;
    memcpy(&out_frame[i], &bw, 4);
    i += 4;
    out_frame[i++] = _role_state->prefs.sf;
    out_frame[i++] = _role_state->prefs.cr;

    // beebo: SETTINGS_ISOLATION follow-up -- name now swaps with live role,
    // same as type (ADV_TYPE_REPEATER/_CHAT, above) and pubkey (self_id,
    // above) already do, instead of always reporting the companion
    // persona's own _role_state->prefs.node_name regardless of live role (a real
    // inconsistency: two of these three fields were already live-role-
    // aware, this one silently wasn't). companion.name/repeater.name
    // settings leaves get either role's name irrespective of live role via
    // GET_PREFS_TLV(KEY_NAME) instead -- self_info is only ever meant to
    // describe "whichever role is live right now", matching its
    // type/pubkey fields.
    const char* name = isRepeater() ? getRoleName(NODE_ROLE_REPEATER) : _role_state->prefs.node_name;
    int tlen = strlen(name); // revisit: UTF_8 ??
    memcpy(&out_frame[i], name, tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (isRepeater() && (
        cmd_frame[0] == CMD_SEND_TXT_MSG || cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG ||
        cmd_frame[0] == CMD_SEND_CHANNEL_DATA || cmd_frame[0] == CMD_GET_CONTACTS ||
        cmd_frame[0] == CMD_RESET_PATH || cmd_frame[0] == CMD_ADD_UPDATE_CONTACT ||
        cmd_frame[0] == CMD_REMOVE_CONTACT || cmd_frame[0] == CMD_SHARE_CONTACT ||
        cmd_frame[0] == CMD_GET_CONTACT_BY_KEY || cmd_frame[0] == CMD_EXPORT_CONTACT ||
        cmd_frame[0] == CMD_IMPORT_CONTACT || cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE ||
        cmd_frame[0] == CMD_HAS_CONNECTION || cmd_frame[0] == CMD_LOGOUT ||
        cmd_frame[0] == CMD_GET_CHANNEL || cmd_frame[0] == CMD_SET_CHANNEL ||
        cmd_frame[0] == CMD_SET_DEFAULT_FLOOD_SCOPE || cmd_frame[0] == CMD_GET_DEFAULT_FLOOD_SCOPE)) {
    // beebo: role separation -- these are companion-only chat/contact/channel
    // commands (BaseChatMesh's contract); a node currently running as
    // repeater must refuse them rather than silently acting as a companion
    // over whatever transport (BLE/WiFi/USB) the app happens to be connected
    // through -- a companion command reachable in
    // repeater mode risk was previously left unaddressed for this legacy
    // command set (only CMD_APP_START's self-info reply was role-branched).
    // CMD_SET/GET_DEFAULT_FLOOD_SCOPE added here for the same reason as the other per-role dirty-routed fields:
    // "reverted" note: default_scope_name/key is companion's own mechanism
    // (never given independent repeater-side storage, see
    // saveBeeboRepeaterPrefs()/loadBeeboRepeaterPrefs()); repeater has its
    // own, more capable default-scope mechanism already
    // (RegionMap/getDefaultScope(), GET/SET_REGION_DEFAULT).
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
  } else if (cmd_frame[0] == CMD_SEND_TXT_MSG && len >= 14) {
    int i = 1;
    uint8_t txt_type = cmd_frame[i++];
    uint8_t attempt = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    uint8_t *pub_key_prefix = &cmd_frame[i];
    i += 6;
    ContactInfo *recipient = lookupContactByPubKey(pub_key_prefix, 6);
    if (recipient && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA)) {
      char *text = (char *)&cmd_frame[i];
      int tlen = len - i;
      text[tlen] = 0; // ensure null

      char *self_rest = (txt_type == TXT_TYPE_CLI_DATA) ? matchAdminSelfCommand(text) : NULL;
      if (self_rest) {
        // beebo: 'self' sentinel (plans/ADMIN_SELF_COMMAND.md) -- run the admin command
        // locally against this device's own live role instead of relaying it to
        // <recipient>. Gated on hasAdminLogin(recipient) -- a real admin CMD_LOGIN to
        // <recipient> succeeded at some point this boot session (persists like a
        // cached password, no reachability requirement at self-command time -- see
        // last_admin_login_pubkey's comment in Beebo.h for why hasConnectionTo()
        // can't be used here instead).
        if (!hasAdminLogin(recipient->id.pub_key)) {
          writeErrFrame(ERR_CODE_BAD_STATE);
        } else {
          handleAdminSelfCommand(*recipient, self_rest);
          out_frame[0] = RESP_CODE_SENT;
          out_frame[1] = 0; // direct, not flood
          uint32_t zero = 0;
          memcpy(&out_frame[2], &zero, 4); // no ack expected
          memcpy(&out_frame[6], &zero, 4); // no timeout
          _serial->writeFrame(out_frame, 10);
        }
      } else {
        uint32_t est_timeout;
        int result;
        uint32_t expected_ack;
        uint32_t tx_pkt_hash = 0;
        if (txt_type == TXT_TYPE_CLI_DATA) {
          msg_timestamp = getRTCClock()->getCurrentTimeUnique(); // Use node's RTC instead of app timestamp to avoid tripping replay protection
          result = sendCommandData(*recipient, msg_timestamp, attempt, text, est_timeout);
          expected_ack = 0; // no Ack expected
        } else {
          result = sendMessage(*recipient, msg_timestamp, attempt, text, expected_ack, est_timeout, &tx_pkt_hash);
        }
        // TODO: add expected ACK to table
        if (result == MSG_SEND_FAILED) {
          writeErrFrame(ERR_CODE_TABLE_FULL);
        } else {
          if (expected_ack) {
            // beebo: if the slot we're
            // about to reuse still has ack != 0, an earlier send hasn't been
            // resolved success or failure yet (checkAckTableTimeouts() hasn't
            // caught it) -- the table wrapped faster than EXPECTED_ACK_TABLE_SIZE
            // sends could resolve, a real starvation event, not a no-op.
            if (expected_ack_table[next_ack_idx].ack != 0) {
              ack_overflow_count++;
              emitAckOverflowEvent(expected_ack_table[next_ack_idx].tx_pkt_hash,
                                    _ms->getMillis() - expected_ack_table[next_ack_idx].msg_sent);
            }
            expected_ack_table[next_ack_idx].msg_sent = _ms->getMillis(); // add to circular table
            expected_ack_table[next_ack_idx].ack = expected_ack;
            expected_ack_table[next_ack_idx].timeout_ms = est_timeout;
            expected_ack_table[next_ack_idx].tx_pkt_hash = tx_pkt_hash;
            expected_ack_table[next_ack_idx].contact = recipient;
            next_ack_idx = (next_ack_idx + 1) % EXPECTED_ACK_TABLE_SIZE;
          }

          out_frame[0] = RESP_CODE_SENT;
          out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
          memcpy(&out_frame[2], &expected_ack, 4);
          memcpy(&out_frame[6], &est_timeout, 4);
          _serial->writeFrame(out_frame, 10);
        }
      }
    } else {
      writeErrFrame(recipient == NULL
                        ? ERR_CODE_NOT_FOUND
                        : ERR_CODE_UNSUPPORTED_CMD); // unknown recipient, or unsupported TXT_TYPE_*
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_TXT_MSG) { // send GroupChannel text msg
    int i = 1;
    uint8_t txt_type = cmd_frame[i++]; // should be TXT_TYPE_PLAIN
    uint8_t channel_idx = cmd_frame[i++];
    uint32_t msg_timestamp;
    memcpy(&msg_timestamp, &cmd_frame[i], 4);
    i += 4;
    const char *text = (char *)&cmd_frame[i];

    if (txt_type != TXT_TYPE_PLAIN) {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
    } else {
      ChannelDetails channel;
      bool success = getChannel(channel_idx, channel);
      if (success && sendGroupMessage(msg_timestamp, channel.channel, _role_state->prefs.node_name, text, len - i)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
      }
    }
  } else if (cmd_frame[0] == CMD_SEND_CHANNEL_DATA) { // send GroupChannel datagram
    if (len < 4) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }
    int i = 1;
    uint8_t channel_idx = cmd_frame[i++];
    uint8_t path_len = cmd_frame[i++];

    // validate path len, allowing 0xFF for flood
    if (!mesh::Packet::isValidPathLen(path_len) && path_len != OUT_PATH_UNKNOWN) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA invalid path size: %d", path_len);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      return;
    }

    // parse provided path if not flood
    uint8_t path[MAX_PATH_SIZE];
    if (path_len != OUT_PATH_UNKNOWN) {
      i += mesh::Packet::writePath(path, &cmd_frame[i], path_len);
    }

    uint16_t data_type = ((uint16_t)cmd_frame[i]) | (((uint16_t)cmd_frame[i + 1]) << 8);
    i += 2;
    const uint8_t *payload = &cmd_frame[i];
    int payload_len = (len > (size_t)i) ? (int)(len - i) : 0;

    ChannelDetails channel;
    if (!getChannel(channel_idx, channel)) {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    } else if (data_type == DATA_TYPE_RESERVED) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (payload_len > MAX_CHANNEL_DATA_LENGTH) {
      MESH_DEBUG_PRINTLN("CMD_SEND_CHANNEL_DATA payload too long: %d > %d", payload_len, MAX_CHANNEL_DATA_LENGTH);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (sendGroupData(channel.channel, path, path_len, data_type, payload, payload_len)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACTS) { // get Contact list
    if (_iter_started) {
      writeErrFrame(ERR_CODE_BAD_STATE); // iterator is currently busy
    } else {
      if (len >= 5) { // has optional 'since' param
        memcpy(&_iter_filter_since, &cmd_frame[1], 4);
      } else {
        _iter_filter_since = 0;
      }

      uint8_t reply[5];
      reply[0] = RESP_CODE_CONTACTS_START;
      uint32_t count = getNumContacts(); // total, NOT filtered count
      memcpy(&reply[1], &count, 4);
      _serial->writeFrame(reply, 5);

      // start iterator
      _iter = startContactsIterator();
      _iter_started = true;
      _most_recent_lastmod = 0;
    }
  } else if (cmd_frame[0] == CMD_SET_ADVERT_NAME && len >= 2) {
    int nlen = len - 1;
    if (nlen > sizeof(_role_state->prefs.node_name) - 1) nlen = sizeof(_role_state->prefs.node_name) - 1; // max len
    memcpy(_role_state->prefs.node_name, &cmd_frame[1], nlen);
    _role_state->prefs.node_name[nlen] = 0; // null terminator
    // beebo: the per-role dirty-routing pattern -- node_name is the
    // live role's own persisted name (see the text-CLI "set name "
    // handler's own comment above), and this legacy binary opcode is
    // reachable while repeater is live, same as CMD_SET_TUNING_PARAMS/etc.
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_ADVERT_LATLON && len >= 9) {
    // beebo: writes _role_state->prefs.node_lat/lon (the live role's own
    // PREFS-mode coordinate), not the shared sensors.node_lat/lon SHARE
    // register. sensors.node_lat/lon is reserved for a real live GPS
    // reading (SHARE mode); this stock opcode is how apps set a
    // fixed/preferred location (PREFS mode) in its absence.
    // beebo: a prior version of this
    // comment claimed an "always companion regardless of live role"
    // contract, mirroring CMD_SET_OTHER_PARAMS/CMD_SET_PATH_HASH_MODE --
    // but those two turned out to be bugs, not a deliberate
    // design. node_lat/node_lon is a single, unified, park/load-swapped
    // RAM field (confirmed by SET_LAT/SET_LON mutating this exact same
    // field for either role) -- there was
    // never a genuinely separate "companion's own copy" to protect while
    // repeater is live; the old contract would have silently overwritten
    // repeater's live coordinate with a value meant for companion. Now
    // that savePrefs() itself is role-aware (see its own comment), this
    // opcode's plain savePrefs() call correctly routes to whichever role
    // is actually live, same as everywhere else -- no branch needed here.
    int32_t lat, lon, alt = 0;
    memcpy(&lat, &cmd_frame[1], 4);
    memcpy(&lon, &cmd_frame[5], 4);
    if (len >= 13) {
      memcpy(&alt, &cmd_frame[9], 4); // for FUTURE support
    }
    if (lat <= 90 * 1E6 && lat >= -90 * 1E6 && lon <= 180 * 1E6 && lon >= -180 * 1E6) {
      _role_state->prefs.node_lat = ((double)lat) / 1000000.0;
      _role_state->prefs.node_lon = ((double)lon) / 1000000.0;
      savePrefs();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid geo coordinate
    }
  } else if (cmd_frame[0] == CMD_GET_DEVICE_TIME) {
    uint8_t reply[5];
    reply[0] = RESP_CODE_CURR_TIME;
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply[1], &now, 4);
    _serial->writeFrame(reply, 5);
  } else if (cmd_frame[0] == CMD_SET_DEVICE_TIME && len >= 5) {
    uint32_t secs;
    memcpy(&secs, &cmd_frame[1], 4);
    uint32_t curr = getRTCClock()->getCurrentTime();
    if (secs >= curr) {
      getRTCClock()->setCurrentTime(secs);
      transport_log.log(TLOG_CLOCK_SET, secs);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SEND_SELF_ADVERT) {
    // beebo: unlike CMD_EXPORT_CONTACT/etc above, this opcode was never in
    // the repeater-refusal list, so it's reachable while repeater is live
    // -- but it built a companion-identity advert (_role_state->prefs.node_name/
    // sensors.node_lat/lon) unconditionally, same bug createSelfAdvertPacket()
    // had before its own role-aware fix. Reuse that (now-correct) helper
    // instead of re-duplicating the role branch here.
    mesh::Packet* pkt = createSelfAdvertPacket();
    if (pkt) {
      if (len >= 2 && cmd_frame[1] == 1) { // optional param (1 = flood, 0 = zero hop)
        unsigned long delay_millis = 0;
        TransportKey default_scope;
        getDefaultScope(this->_board.role, default_scope);
        sendFloodScoped(default_scope, pkt, delay_millis);
      } else {
        sendZeroHop(pkt);
      }
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_RESET_PATH && len >= 1 + 32) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      // recipient->lastmod = ??   shouldn't be needed, app already has this version of contact
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // unknown contact
    }
  } else if (cmd_frame[0] == CMD_ADD_UPDATE_CONTACT && len >= 1 + 32 + 2 + 1) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    uint32_t last_mod = getRTCClock()->getCurrentTime();  // fallback value if not present in cmd_frame
    if (recipient) {
      updateContactFromFrame(*recipient, last_mod, cmd_frame, len);
      recipient->lastmod = last_mod;
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      ContactInfo contact;
      updateContactFromFrame(contact, last_mod, cmd_frame, len);
      contact.lastmod = last_mod;
      contact.sync_since = 0;
      if (addContact(contact)) {
        dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_REMOVE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient && removeContact(*recipient)) {
      _store->deleteBlobByKey(pub_key, PUB_KEY_SIZE);
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found, or unable to remove
    }
  } else if (cmd_frame[0] == CMD_SHARE_CONTACT) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      if (shareContactZeroHop(*recipient)) {
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // unable to send
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_CONTACT_BY_KEY) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *contact = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (contact) {
      writeContactRespFrame(RESP_CODE_CONTACT, *contact);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // not found
    }
  } else if (cmd_frame[0] == CMD_EXPORT_CONTACT) {
    if (len < 1 + PUB_KEY_SIZE) {
      // export SELF -- companion-only reachable (CMD_EXPORT_CONTACT is in
      // the repeater-refusal list above), so createSelfAdvertPacket()'s
      // companion branch is always what runs here; reused instead of
      // duplicating its NONE/SHARE/PREFS branch inline.
      mesh::Packet* pkt = createSelfAdvertPacket();
      if (pkt) {
        pkt->header |= ROUTE_TYPE_FLOOD; // would normally be sent in this mode

        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        uint8_t out_len = pkt->writeTo(&out_frame[1]);
        releasePacket(pkt); // undo the obtainNewPacket()
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL); // Error
      }
    } else {
      uint8_t *pub_key = &cmd_frame[1];
      ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
      uint8_t out_len;
      if (recipient && (out_len = exportContact(*recipient, &out_frame[1])) > 0) {
        out_frame[0] = RESP_CODE_EXPORT_CONTACT;
        _serial->writeFrame(out_frame, out_len + 1);
      } else {
        writeErrFrame(ERR_CODE_NOT_FOUND); // not found
      }
    }
  } else if (cmd_frame[0] == CMD_IMPORT_CONTACT && len > 2 + 32 + 64) {
    if (importContact(&cmd_frame[1], len - 1)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SYNC_NEXT_MESSAGE) {
    int out_len;
    if ((out_len = getFromOfflineQueue(out_frame)) > 0) {
      _serial->writeFrame(out_frame, out_len);
    } else {
      out_frame[0] = RESP_CODE_NO_MORE_MESSAGES;
      _serial->writeFrame(out_frame, 1);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_PARAMS) {
    int i = 1;
    uint32_t freq;
    memcpy(&freq, &cmd_frame[i], 4);
    i += 4;
    uint32_t bw;
    memcpy(&bw, &cmd_frame[i], 4);
    i += 4;
    uint8_t sf = cmd_frame[i++];
    uint8_t cr = cmd_frame[i++];
    uint8_t repeat = 0;  // default - false
    if (len > i) {
      repeat = cmd_frame[i++];   // FIRMWARE_VER_CODE  9+
    }

    if (repeat && !isValidClientRepeatFreq(freq)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (freq >= 150000 && freq <= 2500000 && sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7000 &&
        bw <= 500000) {
      _role_state->prefs.sf = sf;
      _role_state->prefs.cr = cr;
      _role_state->prefs.freq = (float)freq / 1000.0;
      _role_state->prefs.bw = (float)bw / 1000.0;
      _role_state->prefs.client_repeat = repeat;
      // beebo: the per-role dirty-routing pattern -- freq/sf/cr/bw are per-role
      // BeeboBasePrefs fields (settings.yaml, scope: role), reachable via
      // this legacy binary opcode while repeater is live, same
      // dirty-routing rationale as the other per-role fields. An earlier pass
      // incorrectly assumed the read side was already covered, but it wasn't
      // only (consumers now read the unified, role-aware _role_state->prefs);
      // this write-side gap was never actually closed until now.
      savePrefs();

      radio_driver.setParams(_role_state->prefs.freq, _role_state->prefs.bw, _role_state->prefs.sf, _role_state->prefs.cr);
      MESH_DEBUG_PRINTLN("OK: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);

      writeOKFrame();
    } else {
      MESH_DEBUG_PRINTLN("Error: CMD_SET_RADIO_PARAMS: f=%d, bw=%d, sf=%d, cr=%d", freq, bw, (uint32_t)sf,
                         (uint32_t)cr);
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_SET_RADIO_TX_POWER) {
    int8_t power = (int8_t)cmd_frame[1];
    if (power < -9 || power > MAX_LORA_TX_POWER) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _role_state->prefs.tx_power_dbm = power;
      // beebo: the per-role dirty-routing pattern -- same dirty-routing
      // rationale as CMD_SET_RADIO_PARAMS above.
      savePrefs();
      radio_driver.setTxPower(_role_state->prefs.tx_power_dbm);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SET_TUNING_PARAMS) {
    int i = 1;
    uint32_t rx, af;
    memcpy(&rx, &cmd_frame[i], 4);
    i += 4;
    memcpy(&af, &cmd_frame[i], 4);
    i += 4;
    _role_state->prefs.rx_delay_base = ((float)rx) / 1000.0f;
    _role_state->prefs.airtime_factor = ((float)af) / 1000.0f;
    // beebo: the per-role dirty-routing pattern -- legacy binary opcode, reachable
    // while repeater role is live (same connection, hot-switchable role); must
    // route the dirty-write to whichever role's file is actually active, same
    // as the text-CLI "rxdelay"/"af" equivalents, or it would persist the
    // live (possibly repeater) values into the other role's file.
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_TUNING_PARAMS) {
    uint32_t rx = _role_state->prefs.rx_delay_base * 1000, af = _role_state->prefs.airtime_factor * 1000;
    int i = 0;
    out_frame[i++] = RESP_CODE_TUNING_PARAMS;
    memcpy(&out_frame[i], &rx, 4); i += 4;
    memcpy(&out_frame[i], &af, 4); i += 4;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SET_OTHER_PARAMS) {
    _role_state->prefs.manual_add_contacts = cmd_frame[1];
    if (len >= 3) {
      _role_state->prefs.telemetry_mode_base = cmd_frame[2] & 0x03; // v5+
      _role_state->prefs.telemetry_mode_loc = (cmd_frame[2] >> 2) & 0x03;
      _role_state->prefs.telemetry_mode_env = (cmd_frame[2] >> 4) & 0x03;

      if (len >= 4) {
        _role_state->prefs.advert_loc_policy = cmd_frame[3];
        if (len >= 5) {
          _role_state->prefs.multi_acks = cmd_frame[4];
        }
      }
    }
    // beebo: the per-role dirty-routing pattern -- same dirty-routing rationale
    // as CMD_SET_TUNING_PARAMS above; manual_add_contacts/telemetry_mode_*/
    // advert_loc_policy/multi_acks are all per-role persisted fields.
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_PATH_HASH_MODE && cmd_frame[1] == 0 && len >= 3) {
    if (cmd_frame[2] >= 3) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      _role_state->prefs.path_hash_mode = cmd_frame[2];
      // beebo: the per-role dirty-routing pattern -- same dirty-routing rationale
      // as CMD_SET_TUNING_PARAMS above.
      savePrefs();
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_REBOOT && memcmp(&cmd_frame[1], "reboot", 6) == 0) {
    if (dirty_contacts_expiry) { // is there are pending dirty contacts/ACL write needed?
#if BEEBO_ENABLE_REPEATER_ROLE
      if (isRepeater()) acl.save(_store->getPrimaryFS());
      else saveContacts();
#else
      saveContacts();
#endif
    }
    board.reboot();
  } else if (cmd_frame[0] == CMD_GET_BATT_AND_STORAGE) {
    uint8_t reply[11];
    int i = 0;
    reply[i++] = RESP_CODE_BATT_AND_STORAGE;
    uint16_t battery_millivolts = board.getBattMilliVolts();
    uint32_t used = _fs_used_kb;    // beebo: cached; refreshed off the hot path in loop()
    uint32_t total = _fs_total_kb;
    memcpy(&reply[i], &battery_millivolts, 2); i += 2;
    memcpy(&reply[i], &used, 4); i += 4;
    memcpy(&reply[i], &total, 4); i += 4;
    _serial->writeFrame(reply, i);
  } else if (cmd_frame[0] == CMD_EXPORT_PRIVATE_KEY) {
#if ENABLE_PRIVATE_KEY_EXPORT
    // beebo: PER_ROLE_IDENTITY -- optional trailing role byte (len >= 2)
    // targets a specific role's identity regardless of which is currently
    // live, per PROTOCOL_UNIFICATION/backward-compat: binary must be able
    // to reach every role's state from any transport. No role byte (len
    // == 1, upstream's original framing) keeps the pre-split behavior of
    // exporting whatever's currently active.
    uint8_t role = _board.role;
    if (len >= 2) {
      if (cmd_frame[1] != NODE_ROLE_COMPANION && cmd_frame[1] != NODE_ROLE_REPEATER) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        return;
      }
      role = cmd_frame[1];
    }
    mesh::LocalIdentity id;
    if (role == _board.role) {
      id = self_id; // live identity, no extra flash read needed
    } else if (!_store->loadRoleIdentity(role, id)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // that role has never been initialized on this device
      return;
    }
    uint8_t reply[65];
    reply[0] = RESP_CODE_PRIVATE_KEY;
    id.writeTo(&reply[1], 64);
    _serial->writeFrame(reply, 65);
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_IMPORT_PRIVATE_KEY && len >= 65) {
#if ENABLE_PRIVATE_KEY_IMPORT
    // beebo: PER_ROLE_IDENTITY -- same optional trailing role byte as
    // CMD_EXPORT_PRIVATE_KEY (len >= 66). No role byte (len == 65,
    // upstream's original framing) keeps importing into whatever role is
    // currently active.
    uint8_t role = _board.role;
    if (len >= 66) {
      if (cmd_frame[65] != NODE_ROLE_COMPANION && cmd_frame[65] != NODE_ROLE_REPEATER) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        return;
      }
      role = cmd_frame[65];
    }
    if (!mesh::LocalIdentity::validatePrivateKey(&cmd_frame[1])) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid key
    } else {
        mesh::LocalIdentity identity;
        identity.readFrom(&cmd_frame[1], 64);
        if (_store->saveRoleIdentity(role, identity)) {
          writeOKFrame();
          if (role == _board.role) {
            role_state_store[role].identity = identity;
            self_id = identity;
            // re-load contacts, to invalidate ecdh shared_secrets (companion-
            // only state, see begin()'s role guard)
            if (isCompanion()) {
              resetContacts();
              _store->loadContacts(this);
            }
#if BEEBO_ENABLE_REPEATER_ROLE
            // repeater's ACL shared secrets self-heal the same way on next
            // load (ClientACL::load() recomputes them from self_id), but
            // that only runs at boot/role-switch -- if repeater is live
            // right now, recompute in place immediately.
            if (isRepeater()) acl.load(_store->getPrimaryFS(), self_id);
#endif
          }
        } else {
          writeErrFrame(ERR_CODE_FILE_IO_ERROR);
        }
    }
#else
    writeDisabledFrame();
#endif
  } else if (cmd_frame[0] == CMD_SEND_RAW_DATA && len >= 6) {
    int i = 1;
    int8_t path_len = cmd_frame[i++];
    if (path_len >= 0 && i + path_len + 4 <= len) { // minimum 4 byte payload
      uint8_t *path = &cmd_frame[i];
      i += path_len;
      auto pkt = createRawData(&cmd_frame[i], len - i);
      if (pkt) {
        sendDirect(pkt, path, path_len);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    } else {
      writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // flood, not supported (yet)
    }
  } else if (cmd_frame[0] == CMD_SEND_LOGIN && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    char *password = (char *)&cmd_frame[1 + PUB_KEY_SIZE];
    cmd_frame[len] = 0; // ensure null terminator in password
    if (recipient) {
      uint32_t est_timeout;
      int result = sendLogin(*recipient, password, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        memcpy(&pending_login, recipient->id.pub_key, 4); // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &pending_login, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_ANON_REQ && len > 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    ContactInfo anon;
    if (recipient == NULL) { // FIRMWARE_VER_CODE 13+, allow non-contact requests
      memset(&anon, 0, sizeof(anon));
      memcpy(anon.id.pub_key, pub_key, PUB_KEY_SIZE);
      anon.out_path_len = 0;   // default to zero-hop direct
      anon.type = ADV_TYPE_NONE;  // unknown

      if (addContact(anon)) recipient = &anon;
    }
    uint8_t *data = &cmd_frame[1 + PUB_KEY_SIZE];
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendAnonReq(*recipient, data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this to onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL); // contacts full
    }
  } else if (cmd_frame[0] == CMD_SEND_STATUS_REQ && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_STATUS, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        // FUTURE:  pending_status = tag;  // match this in onContactResponse()
        memcpy(&pending_status, recipient->id.pub_key, 4); // legacy matching scheme
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_PATH_DISCOVERY_REQ && cmd_frame[1] == 0 && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[2];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      // 'Path Discovery' is just a special case of flood + Telemetry req
      uint8_t req_data[9];
      req_data[0] = REQ_TYPE_GET_TELEMETRY_DATA;
      req_data[1] = ~(TELEM_PERM_BASE);  // NEW: inverse permissions mask (ie. we only want BASE telemetry)
      memset(&req_data[2], 0, 3);  // reserved
      getRNG()->random(&req_data[5], 4);   // random blob to help make packet-hash unique
      auto save = recipient->out_path_len;    // temporarily force sendRequest() to flood
      recipient->out_path_len = OUT_PATH_UNKNOWN;
      int result = sendRequest(*recipient, req_data, sizeof(req_data), tag, est_timeout);
      recipient->out_path_len = save;
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_discovery = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len >= 4 + PUB_KEY_SIZE) {  // can deprecate, in favour of CMD_SEND_BINARY_REQ
    uint8_t *pub_key = &cmd_frame[4];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, REQ_TYPE_GET_TELEMETRY_DATA, tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_telemetry = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_SEND_TELEMETRY_REQ && len == 4) {  // 'self' telemetry request
    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);
    // query other sensors -- target specific
    sensors.querySensors(0xFF, telemetry);

    int i = 0;
    out_frame[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    out_frame[i++] = 0; // reserved
    memcpy(&out_frame[i], self_id.pub_key, 6);
    i += 6; // pub_key_prefix
    uint8_t tlen = telemetry.getSize();
    memcpy(&out_frame[i], telemetry.getBuffer(), tlen);
    i += tlen;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_BINARY_REQ && len >= 2 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient) {
      uint8_t *req_data = &cmd_frame[1 + PUB_KEY_SIZE];
      uint32_t tag, est_timeout;
      int result = sendRequest(*recipient, req_data, len - (1 + PUB_KEY_SIZE), tag, est_timeout);
      if (result == MSG_SEND_FAILED) {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      } else {
        clearPendingReqs();
        pending_req = tag; // match this in onContactResponse()
        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      }
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // contact not found
    }
  } else if (cmd_frame[0] == CMD_HAS_CONNECTION && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    if (hasConnectionTo(pub_key)) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_LOGOUT && len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &cmd_frame[1];
    stopConnection(pub_key);
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_CHANNEL && len >= 2) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    if (getChannel(channel_idx, channel)) {
      int i = 0;
      out_frame[i++] = RESP_CODE_CHANNEL_INFO;
      out_frame[i++] = channel_idx;
      strcpy((char *)&out_frame[i], channel.name);
      i += 32;
      memcpy(&out_frame[i], channel.channel.secret, 16);
      i += 16; // NOTE: only 128-bit supported
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 32) {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD); // not supported (yet)
  } else if (cmd_frame[0] == CMD_SET_CHANNEL && len >= 2 + 32 + 16) {
    uint8_t channel_idx = cmd_frame[1];
    ChannelDetails channel;
    StrHelper::strncpy(channel.name, (char *)&cmd_frame[2], 32);
    memset(channel.channel.secret, 0, sizeof(channel.channel.secret));
    memcpy(channel.channel.secret, &cmd_frame[2 + 32], 16); // NOTE: only 128-bit supported
    if (setChannel(channel_idx, channel)) {
      saveChannels();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND); // bad channel_idx
    }
  } else if (cmd_frame[0] == CMD_SIGN_START) {
    out_frame[0] = RESP_CODE_SIGN_START;
    out_frame[1] = 0; // reserved
    uint32_t len = MAX_SIGN_DATA_LEN;
    memcpy(&out_frame[2], &len, 4);
    _serial->writeFrame(out_frame, 6);

    if (sign_data) {
      free(sign_data);
    }
    sign_data = (uint8_t *)malloc(MAX_SIGN_DATA_LEN);
    sign_data_len = 0;
  } else if (cmd_frame[0] == CMD_SIGN_DATA && len > 1) {
    if (sign_data == NULL || sign_data_len + (len - 1) > MAX_SIGN_DATA_LEN) {
      writeErrFrame(sign_data == NULL ? ERR_CODE_BAD_STATE : ERR_CODE_TABLE_FULL); // error: too long
    } else {
      memcpy(&sign_data[sign_data_len], &cmd_frame[1], len - 1);
      sign_data_len += (len - 1);
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_SIGN_FINISH) {
    if (sign_data) {
      self_id.sign(&out_frame[1], sign_data, sign_data_len);

      free(sign_data); // don't need sign_data now
      sign_data = NULL;

      out_frame[0] = RESP_CODE_SIGNATURE;
      _serial->writeFrame(out_frame, 1 + SIGNATURE_SIZE);
    } else {
      writeErrFrame(ERR_CODE_BAD_STATE);
    }
  } else if (cmd_frame[0] == CMD_SEND_TRACE_PATH && len > 10 && len - 10 < MAX_PACKET_PAYLOAD-5) {
    uint8_t path_len = len - 10;
    uint8_t flags = cmd_frame[9];
    uint8_t path_sz = flags & 0x03;  // NEW v1.11+
    if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) { // make sure is multiple of path_sz
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      uint32_t tag, auth;
      memcpy(&tag, &cmd_frame[1], 4);
      memcpy(&auth, &cmd_frame[5], 4);
      auto pkt = createTrace(tag, auth, flags);
      if (pkt) {
        sendDirect(pkt, &cmd_frame[10], path_len);

        uint32_t t = _radio->getEstAirtimeFor(pkt->payload_len + pkt->path_len + 2);
        uint32_t est_timeout = calcDirectTimeoutMillisFor(t, path_len >> path_sz);

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (cmd_frame[0] == CMD_SET_DEVICE_PIN && len >= 5) {

    // get pin from command frame
    uint32_t pin;
    memcpy(&pin, &cmd_frame[1], 4);

    // ensure pin is zero, or a valid 6 digit pin
    if (pin == 0 || (pin >= 100000 && pin <= 999999)) {
      _role_state->prefs.ble_pin = pin;
      savePrefs();
      // beebo: must apply live, not just persist -- two RAM copies shadow
      // _role_state->prefs.ble_pin and both have to move with it:
      // _active_ble_pin (the 0 -> BLE_PIN_CODE fallback resolved at load,
      // read by getBLEPin() when a transport toggle calls
      // ble_interface.begin()) and the BLE stack's own _pin_code. A ble
      // off/on cycle alone does NOT pick a new pin up -- that path takes
      // initRadio(), not begin(), so it would re-apply the stale cached copy.
      _active_ble_pin = (pin == 0) ? BLE_PIN_CODE : pin;   // same fallback begin() resolves
      ble_interface.setPinCode(_active_ble_pin);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_CUSTOM_VARS) {
    out_frame[0] = RESP_CODE_CUSTOM_VARS;
    char *dp = (char *)&out_frame[1];
    for (int i = 0; i < sensors.getNumSettings() && dp - (char *)&out_frame[1] < 140; i++) {
      if (i > 0) {
        *dp++ = ',';
      }
      strcpy(dp, sensors.getSettingName(i));
      dp = strchr(dp, 0);
      *dp++ = ':';
      strcpy(dp, sensors.getSettingValue(i));
      dp = strchr(dp, 0);
    }
    _serial->writeFrame(out_frame, dp - (char *)out_frame);
  } else if (cmd_frame[0] == CMD_SET_CUSTOM_VAR && len >= 4) {
    cmd_frame[len] = 0;
    char *sp = (char *)&cmd_frame[1];
    char *np = strchr(sp, ':'); // look for separator char
    if (np) {
      *np++ = 0; // modify 'cmd_frame', replace ':' with null
      bool success = sensors.setSettingValue(sp, np);
      if (success) {
        #if ENV_INCLUDE_GPS == 1
        // Update node preferences for GPS settings. gps_enabled/gps_interval
        // are unified, park-loaded BeeboBasePrefs fields like rx_delay_base/
        // advert_loc_policy/etc -- savePrefs() routes by live role itself
        // no branch needed here.
        if (strcmp(sp, "gps") == 0) {
          _role_state->prefs.gps_enabled = (np[0] == '1') ? 1 : 0;
          savePrefs();
        } else if (strcmp(sp, "gps_interval") == 0) {
          uint32_t interval_seconds = atoi(np);
          _role_state->prefs.gps_interval = constrain(interval_seconds, 0, 86400);
          savePrefs();
        }
        #endif
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (cmd_frame[0] == CMD_GET_ADVERT_PATH && len >= PUB_KEY_SIZE+2) {
    // FUTURE use:  uint8_t reserved = cmd_frame[1];
    uint8_t *pub_key = &cmd_frame[2];
    AdvertPath* found = NULL;
    for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) {
      auto p = &advert_paths[i];
      if (memcmp(p->pubkey_prefix, pub_key, sizeof(p->pubkey_prefix)) == 0) {
        found = p;
        break;
      }
    }
    if (found) {
      int i = 0;
      out_frame[i++] = RESP_CODE_ADVERT_PATH;
      memcpy(&out_frame[i], &found->recv_timestamp, 4); i += 4;
      out_frame[i++] = found->path_len;
      i += mesh::Packet::writePath(&out_frame[i], found->path, found->path_len);
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (cmd_frame[0] == CMD_GET_STATS && len >= 2) {
    uint8_t stats_type = cmd_frame[1];
    if (stats_type == STATS_TYPE_CORE) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_CORE;
      // beebo: a status query already forces a live ADC read here, so use
      // it to opportunistically drive the trend classifier too if due.
      uint16_t battery_mv = updateBattTrend(true);
      uint32_t uptime_secs = _ms->getMillis() / 1000;
      uint8_t queue_len = (uint8_t)_mgr->getOutboundTotal();
      memcpy(&out_frame[i], &battery_mv, 2); i += 2;
      memcpy(&out_frame[i], &uptime_secs, 4); i += 4;
      memcpy(&out_frame[i], &_err_flags, 2); i += 2;
      out_frame[i++] = queue_len;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_RADIO) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_RADIO;
      // beebo: bench IR-drop test (board.state.quiet) sleeps the radio --
      // getLastRSSI()/getLastSNR() are live SX1262 register reads
      // (RadioLibWrapper::getLastRSSI/getLastSNR -> _radio->getRSSI/
      // getSNR()) that would otherwise touch the sleeping chip and stall
      // the whole reply for however long that takes to (not) resolve.
      // Report zeros instead of touching hardware while asleep; airtime
      // counters are plain RAM counters, safe to report either way.
      int16_t noise_floor = 0;
      int8_t last_rssi = 0;
      int8_t last_snr = 0;
      if (!_bench_quiet) {
        noise_floor = (int16_t)_radio->getNoiseFloor();
        last_rssi = (int8_t)radio_driver.getLastRSSI();
        last_snr = (int8_t)(radio_driver.getLastSNR() * 4); // scaled by 4 for 0.25 dB precision
      }
      uint32_t tx_air_secs = getTotalAirTime() / 1000;
      uint32_t rx_air_secs = getReceiveAirTime() / 1000;
      memcpy(&out_frame[i], &noise_floor, 2); i += 2;
      out_frame[i++] = last_rssi;
      out_frame[i++] = last_snr;
      memcpy(&out_frame[i], &tx_air_secs, 4); i += 4;
      memcpy(&out_frame[i], &rx_air_secs, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_PACKETS) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_PACKETS;
      uint32_t recv = radio_driver.getPacketsRecv();
      uint32_t sent = radio_driver.getPacketsSent();
      uint32_t n_sent_flood = getNumSentFlood();
      uint32_t n_sent_direct = getNumSentDirect();
      uint32_t n_recv_flood = getNumRecvFlood();
      uint32_t n_recv_direct = getNumRecvDirect();
      uint32_t n_recv_errors = radio_driver.getPacketsRecvErrors();
      uint32_t n_direct_dups = ((SimpleMeshTables *)getTables())->getNumDirectDups();
      uint32_t n_flood_dups = ((SimpleMeshTables *)getTables())->getNumFloodDups();
      memcpy(&out_frame[i], &recv, 4); i += 4;
      memcpy(&out_frame[i], &sent, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_sent_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_flood, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_direct, 4); i += 4;
      memcpy(&out_frame[i], &n_recv_errors, 4); i += 4;
      memcpy(&out_frame[i], &n_direct_dups, 4); i += 4;
      memcpy(&out_frame[i], &n_flood_dups, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_SYSTEM) {
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = STATS_TYPE_SYSTEM;
      uint32_t free_heap = ESP.getFreeHeap();
      uint32_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
      uint32_t flash_size = ESP.getFlashChipSize();
      // Totals so the app can show used/free against capacity (beebo). Flash
      // app/OTA partition = current sketch + free sketch space. sketch size
      // never changes at runtime but ESP.getSketchSize()/getFreeSketchSpace()
      // re-scan the app partition on flash each call (~300ms) -- cache once.
      uint32_t total_heap = ESP.getHeapSize();
      uint32_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
      static uint32_t sketch_used = 0, sketch_total = 0;
      static bool sketch_size_cached = false;
      if (!sketch_size_cached) {
        sketch_used = ESP.getSketchSize();
        sketch_total = sketch_used + ESP.getFreeSketchSpace();
        sketch_size_cached = true;
      }
      int16_t mcu_temp = _mcu_temp_scaled; // beebo: cached; refreshed off the hot path in loop()
      memcpy(&out_frame[i], &free_heap, 4); i += 4;
      memcpy(&out_frame[i], &free_psram, 4); i += 4;
      memcpy(&out_frame[i], &flash_size, 4); i += 4;
      memcpy(&out_frame[i], &mcu_temp, 2); i += 2;
      // Appended after mcu_temp so older apps that stop at 16 bytes still parse.
      memcpy(&out_frame[i], &total_heap, 4); i += 4;
      memcpy(&out_frame[i], &total_psram, 4); i += 4;
      memcpy(&out_frame[i], &sketch_used, 4); i += 4;
      memcpy(&out_frame[i], &sketch_total, 4); i += 4;
      // beebo: messages queued for the companion to sync (awaiting read), so a
      // headless node's backlog is visible from `beebo status`. Append-only.
      uint16_t pending_msgs = (uint16_t)offline_queue_len;
      memcpy(&out_frame[i], &pending_msgs, 2); i += 2;
      // beebo: resident contact count, so `beebo status` can show it without
      // downloading the whole list (which CMD_GET_CONTACTS would). Append-only.
      uint16_t num_contacts = (uint16_t)getNumContacts();
      memcpy(&out_frame[i], &num_contacts, 2); i += 2;
      // beebo: monitor ring state (enabled flag + fill vs capacity), so
      // `beebo status` can show monitoring at a glance. Append-only.
      out_frame[i++] = monring.enabled() ? 1 : 0;
      uint32_t monring_count = monring.count();
      uint32_t monring_cap = monring.capacity();
      memcpy(&out_frame[i], &monring_count, 4); i += 4;
      memcpy(&out_frame[i], &monring_cap, 4); i += 4;
      _serial->writeFrame(out_frame, i);
    } else if (stats_type == STATS_TYPE_TRANSPORT || stats_type == STATS_TYPE_PROFILE) {
      // Paginated fetch: optional 2-byte LE start offset in cmd_frame[2..3].
      // Response: [STATS][type][total LE16][offset LE16][events...]
      //
      // beebo: same page-size/streaming uplevel as BEEBO_CMD_GET_MONRING --
      // a full ring is small (TLOG_MAX_EVENTS records) but was always paged
      // at the legacy 176-byte MAX_FRAME_SIZE with one command round trip
      // per page (e.g. ~57 round trips for a 1024-event ring), unlike
      // GET_MONRING which already rides _app_max_tx/_app_stream once
      // negotiated via SET_XFER_CAPS. Every extra round trip is a fresh
      // chance to hit the rare single-command USB stall this was chasing;
      // fewer, bigger, self-driving pages cuts that exposure directly.
      // If the app negotiated streaming, a bare offset==0 request just arms
      // the loop pump (see checkSerialInterface()) instead of replying
      // immediately -- same shape as BEEBO_CMD_GET_MONRING's _monread. An
      // explicit offset!=0 (a pre-streaming app, or an app that decided not
      // to negotiate) always gets the legacy single-page reply.
      uint16_t offset = (len >= 4) ? (cmd_frame[2] | ((uint16_t)cmd_frame[3] << 8)) : 0;
      // Mark the read boundary once per fetch (first page) so the next debuglog
      // clearly shows which events are new.
      if (stats_type == STATS_TYPE_TRANSPORT && offset == 0) transport_log.log(TLOG_DEBUGLOG_READ);
      if (_app_stream && offset == 0) {
        _statread.active = true;
        _statread.kind = stats_type;
        _statread.offset = 0;
        return;
      }
      int i = 0;
      out_frame[i++] = RESP_CODE_STATS;
      out_frame[i++] = stats_type;
      int hdr = i;
      i += 4;  // reserve total + offset
      uint16_t total = 0;
      size_t page_cap = _app_max_tx;   // larger paged frames if negotiated (else 176)
      if (stats_type == STATS_TYPE_TRANSPORT)
        i += transport_log.serialize(&out_frame[i], page_cap - i, offset, &total);
      else
        i += profile_log.serialize(&out_frame[i], page_cap - i, offset, &total);
      out_frame[hdr + 0] = total & 0xFF;
      out_frame[hdr + 1] = (total >> 8) & 0xFF;
      out_frame[hdr + 2] = offset & 0xFF;
      out_frame[hdr + 3] = (offset >> 8) & 0xFF;
      _serial->writeFrame(out_frame, i);
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // invalid stats sub-type
    }
  } else if (cmd_frame[0] == CMD_FACTORY_RESET && memcmp(&cmd_frame[1], "reset", 5) == 0) {
    if (_serial) {
      MESH_DEBUG_PRINTLN("Factory reset: disabling serial interface to prevent reconnects (BLE/WiFi)");
      _serial->disable(); // Phone app disconnects before we can send OK frame so it's safe here
    }
    bool success = _store->formatFileSystem();
    if (success) {
      writeOKFrame();
      delay(1000);
      board.reboot();  // doesn't return
    } else {
      writeErrFrame(ERR_CODE_FILE_IO_ERROR);
    }
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 0) {
    if (len >= 2 + 16) {
      memcpy(send_scope.key, &cmd_frame[2], sizeof(send_scope.key));  // set scope override TransportKey
    } else {
      memset(send_scope.key, 0, sizeof(send_scope.key));  // reset scope override
    }
    send_unscoped = false;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_FLOOD_SCOPE_KEY && len >= 2 && cmd_frame[1] == 1) {  // ver 12+
    send_unscoped = true;
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_SET_DEFAULT_FLOOD_SCOPE && len >= 1) {
    // beebo: companion-only, unconditionally;
    // repeater is refused above (see the repeater-refusal branch). This
    // is companion's own default-scope mechanism, never given independent
    // repeater-side storage -- repeater has its own, more capable one
    // already (RegionMap/getDefaultScope(), GET/SET_REGION_DEFAULT).
    if (len >= 1+31+16) {
      int n = strlen((char *) &cmd_frame[1]);
      if (n > 0 && n < 31) {
        strcpy(_role_state->prefs.default_scope_name, (char *) &cmd_frame[1]);
        memcpy(_role_state->prefs.default_scope_key, &cmd_frame[1+31], 16);
        savePrefs();
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      memset(_role_state->prefs.default_scope_name, 0, sizeof(_role_state->prefs.default_scope_name));  // set default scope to null
      memset(_role_state->prefs.default_scope_key, 0, sizeof(_role_state->prefs.default_scope_key));
      savePrefs();
      writeOKFrame();
    }
  } else if (cmd_frame[0] == CMD_GET_DEFAULT_FLOOD_SCOPE) {
    out_frame[0] = RESP_CODE_DEFAULT_FLOOD_SCOPE;
    if (strlen(_role_state->prefs.default_scope_name) > 0) {
      memcpy(&out_frame[1], _role_state->prefs.default_scope_name, 31);
      memcpy(&out_frame[1+31], _role_state->prefs.default_scope_key, 16);
      _serial->writeFrame(out_frame, 1+31+16);
    } else {
      _serial->writeFrame(out_frame, 1);   // no name or key means null
    }
  } else if (cmd_frame[0] == CMD_SEND_CONTROL_DATA && len >= 2 && (cmd_frame[1] & 0x80) != 0) {
    auto resp = createControlData(&cmd_frame[1], len - 1);
    if (resp) {
      sendZeroHop(resp);
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_SET_AUTOADD_CONFIG) {
    _role_state->prefs.autoadd_config = cmd_frame[1];
    if (len >= 3) {
      _role_state->prefs.autoadd_max_hops = min(cmd_frame[2], (uint8_t)64);
    }
    savePrefs();
    writeOKFrame();
  } else if (cmd_frame[0] == CMD_GET_AUTOADD_CONFIG) {
    int i = 0;
    out_frame[i++] = RESP_CODE_AUTOADD_CONFIG;
    out_frame[i++] = _role_state->prefs.autoadd_config;
    out_frame[i++] = _role_state->prefs.autoadd_max_hops;
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_GET_ALLOWED_REPEAT_FREQ) {
    int i = 0;
    out_frame[i++] = RESP_ALLOWED_REPEAT_FREQ;
    for (int k = 0; k < sizeof(repeat_freq_ranges)/sizeof(repeat_freq_ranges[0]) && i + 8 < sizeof(out_frame); k++) {
      auto r = &repeat_freq_ranges[k];
      memcpy(&out_frame[i], &r->lower_freq, 4); i += 4;
      memcpy(&out_frame[i], &r->upper_freq, 4); i += 4;
    }
    _serial->writeFrame(out_frame, i);
  } else if (cmd_frame[0] == CMD_SEND_RAW_PACKET && len >= 4) {
    auto pkt = obtainNewPacket();
    if (pkt) {
      uint8_t priority = cmd_frame[1];
      if (tryParsePacket(pkt, &cmd_frame[2], len - 2)) {
        sendPacket(pkt, priority, 0);
        writeOKFrame();
      } else {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      }
    } else {
      writeErrFrame(ERR_CODE_TABLE_FULL);
    }
  } else if (cmd_frame[0] == CMD_BEEBO && len >= 2) {
    uint8_t* sub = &cmd_frame[1];
    int sub_len = len - 1;
  if (sub[0] == BEEBO_CMD_APP_DISCONNECT) {
    MESH_DEBUG_PRINTLN("App disconnect requested");
    writeOKFrame();
    _pending_disconnect = true;
  } else if (sub[0] == BEEBO_CMD_SEND_POKE && sub_len >= 1 + PUB_KEY_SIZE) {
    uint8_t *pub_key = &sub[1];
    ContactInfo *recipient = lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
    if (recipient == NULL) {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    } else if (recipient->out_path_len == OUT_PATH_UNKNOWN) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG); // poke is direct-route only, need a known path
    } else {
      uint8_t dest_hash[PATH_HASH_SIZE];
      recipient->id.copyHashTo(dest_hash);

      uint32_t tag = getRTCClock()->getCurrentTimeUnique();
      auto pkt = createPoke(POKE_SUBTYPE_REQUEST, dest_hash, recipient->getSharedSecret(self_id),
                            (uint8_t *) &tag, 4);
      if (pkt) {
        sendDirect(pkt, recipient->out_path, recipient->out_path_len);

        uint32_t t = _radio->getEstAirtimeFor(pkt->getRawLength());
        uint32_t est_timeout = calcDirectTimeoutMillisFor(t, recipient->out_path_len);

        out_frame[0] = RESP_CODE_SENT;
        out_frame[1] = 0;
        memcpy(&out_frame[2], &tag, 4);
        memcpy(&out_frame[6], &est_timeout, 4);
        _serial->writeFrame(out_frame, 10);
      } else {
        writeErrFrame(ERR_CODE_TABLE_FULL);
      }
    }
  } else if (sub[0] == BEEBO_CMD_GET_SAVE_PREFS) {
    out_frame[0] = RESP_CODE_OK;
    uint32_t value = getSavePrefs() ? 1 : 0;
    memcpy(&out_frame[1], &value, 4);
    _serial->writeFrame(out_frame, 5);
  } else if (sub[0] == BEEBO_CMD_SET_SAVE_PREFS && sub_len >= 2) {
    uint8_t value = sub[1];
    if (value == 0) {                // off: hold all pref writes in RAM
      setSavePrefs(false);
      writeOKFrame();
    } else if (value == 1) {         // on: re-enable saving (no commit)
      setSavePrefs(true);
      writeOKFrame();
    } else if (value == 2) {         // restore: reload from flash, resume saving
      reloadPrefs();
      setSavePrefs(true);
      writeOKFrame();
    } else if (value == 3) {         // commit: force-write current RAM state now
      commitPrefs();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (sub[0] == BEEBO_CMD_GET_BATT_STATE) {
    // beebo: on-demand classify while still INIT (CLI: "UNKNOWN"), so a
    // caller doesn't have to wait on updateBattTrend()'s own due timer
    // (see its _batt_boot_settled comment) to learn a voltage already
    // known right now. Uses local scratch variables, NOT
    // _cached_batt_mv/_batt_state -- writing straight into those raced
    // the very next periodic loop() sample, whose ADC jitter could
    // immediately trip classifyBattTrend()'s ungated CHARGED-exit check
    // (see that function's own comment) and flip a boundary-voltage
    // CHARGED right back to DISCHARGING. Keeping this local means it can
    // never disturb the periodic sampler's real state.
    uint32_t value = _batt_state;
    if (_batt_state == BATT_STATE_INIT && radioIsIdle()) {
      uint16_t mv = board.getBattMilliVolts();
      uint16_t scratch_ref = mv;
      uint8_t scratch_state = BATT_STATE_INIT;
      value = classifyBattTrend(mv, scratch_ref, scratch_state,
                                 _board.batt_present, _board.adc_resolution_bits,
                                 BATT_FULL_MV);
    }
    out_frame[0] = RESP_CODE_OK;
    memcpy(&out_frame[1], &value, 4);
    _serial->writeFrame(out_frame, 5);
  } else if (sub[0] == BEEBO_CMD_SET_BATT_STATE && sub_len >= 2) {
    uint8_t value = sub[1];
    if (_board.batt_present == BATT_PRESENT_NO) {
      // beebo: batt_present==no already pins the classifier to PLUGGED (see
      // classifyBattTrend()/resetBattTrendRef()) -- forcing any other state
      // here would just get overwritten by the next sample, so reject it
      // outright instead of silently doing nothing.
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (value == BATT_STATE_INIT || value == BATT_STATE_CHARGING || value == BATT_STATE_DISCHARGING
        || value == BATT_STATE_CHARGED || value == BATT_STATE_PLUGGED) {
      _batt_state = value;
      // beebo: re-seed the anchor to the current reading -- forcing a state
      // with a stale ref_mv would immediately re-trigger a trend edge on the
      // next sample and undo the forced state.
      _cached_batt_mv = board.getBattMilliVolts();
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    }
  } else if (sub[0] == BEEBO_CMD_GET_NODE_ROLE) {
    out_frame[0] = RESP_CODE_OK;
    uint32_t value = _board.role;
    memcpy(&out_frame[1], &value, 4);
    _serial->writeFrame(out_frame, 5);
  } else if (sub[0] == BEEBO_CMD_SET_NODE_ROLE && sub_len >= 2) {
    // beebo: always reboots on an actual change -- see
    // requestNodeRoleSwitch()'s own comment. The ack is written before the
    // reboot so the transport carrying this command gets to deliver it.
    switch (requestNodeRoleSwitch(sub[1], EVENT_SOURCE_BINARY)) {
      case NODE_ROLE_SWITCH_ERR_ARG:
      case NODE_ROLE_SWITCH_ERR_BUILTIN:
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        break;
      case NODE_ROLE_SWITCH_ERR_BUSY:
        writeErrFrame(ERR_CODE_BAD_STATE);
        break;
      case NODE_ROLE_SWITCH_NOOP:
        writeOKFrame();
        break;
      case NODE_ROLE_SWITCH_REBOOTING:
        writeOKFrame();
        delay(1000);
        board.reboot();  // doesn't return
        break;
    }
  // beebo: multi_role -- ComPrefs (/com_prefs) field accessors, same
  // GET/SET-pair-per-field convention as the NodePrefs accessors above
  // (e.g. GET_NODE_ROLE/SET_NODE_ROLE, GET_ADC_MULTIPLIER/SET_ADC_MULTIPLIER):
  // scalar GET replies RESP_CODE_OK + 4B LE value; scalar SET payload width
  // matches the field's own storage width. Offsets/semantics here must stay
  // in lockstep with handleCommand()'s "get"/"set" text-CLI branches, which
  // read/write the exact same ComPrefs bytes via readComPrefsField()/
  // writeComPrefsField().
  // beebo: every field below is now table-driven (PREFS_TLV_FIELDS,
  // Beebo.h/BeeboRepeater.cpp) -- these individual handlers call the exact
  // same tlvGet*/tlvSet* functions GET_PREFS_TLV/SET_PREFS_TLV use, so the
  // two paths can't drift apart. Only the wire framing (RESP_CODE_OK+4B for
  // a scalar GET, RESP_CODE_BEEBO+sub+bytes for a string GET, OK/ERR frame
  // for a SET) still lives here, matching the plain BEEBO_CMD_GET/SET_*
  // convention every other field in this file uses.
  } else if (sub[0] == BEEBO_CMD_GET_ACK_STATS) {
    // beebo: "TX reception confirmation".
    // Lifetime counts, RAM-only (BaseChatMesh's own _ack_success_count/
    // _ack_timeout_count, incremented at onAckRecv()/onContactPathRecv()'s
    // already-correct ACK-match points and Beebo::checkAckTableTimeouts()'s
    // per-slot sweep; ack_overflow_count is the starvation counter
    // from that same sweep's insert site -- this opcode is purely a read,
    // no new detection). Three u32 values don't fit RESP_CODE_OK's fixed
    // 4-byte payload (the stock meshcore library's generic OK-frame parser
    // only ever reads bytes[1:5] into "value", silently dropping anything
    // after) -- needs its own RESP_CODE_BEEBO sub-id and a custom reader
    // patch client-side, same as GET_FULL_VERSION above.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_ACK_STATS;
    uint32_t success = getAckSuccessCount();
    uint32_t timeout = getAckTimeoutCount();
    uint32_t overflow = getAckOverflowCount();
    memcpy(&out_frame[2], &success, 4);
    memcpy(&out_frame[6], &timeout, 4);
    memcpy(&out_frame[10], &overflow, 4);
    _serial->writeFrame(out_frame, 14);
  } else if (sub[0] == BEEBO_CMD_GET_ECHO_COUNT) {
    // beebo: flood-echo side. Four u32
    // values, same RESP_CODE_BEEBO wrapper as GET_ACK_STATS above (this used
    // to be a single-value RESP_CODE_OK scalar before echo_timeout_count/
    // echo_overflow_count/self_tx_direct_count existed). self_tx_direct_count
    // is fetched over the wire here (no longer dead write-only bookkeeping)
    // but deliberately not rendered as its own row in `status packets` --
    // every direct-routed self-transmission is excluded from ECHO tracking
    // with no exceptions, and the same population is already shown by the
    // TX > Direct row, so a dedicated row would just restate that value.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_ECHO_STATS;
    SimpleMeshTables *tables = (SimpleMeshTables *)getTables();
    uint32_t heard = tables->getEchoSuccessCount();
    uint32_t not_heard = tables->getEchoTimeoutCount();
    uint32_t overflow = tables->getEchoOverflowCount();
    uint32_t self_tx_direct = tables->getSelfTxDirectCount();
    memcpy(&out_frame[2], &heard, 4);
    memcpy(&out_frame[6], &not_heard, 4);
    memcpy(&out_frame[10], &overflow, 4);
    memcpy(&out_frame[14], &self_tx_direct, 4);
    _serial->writeFrame(out_frame, 18);
  } else if (sub[0] == BEEBO_CMD_REBOOT_WITH_TIME && sub_len >= 5) {
    // beebo: recovery path for a device whose own clock is wrong (e.g.
    // stuck ahead of every real clock) -- an ordinary reboot persists this
    // device's own current time via board.reboot(), which just re-persists
    // the same bad value forever. This persists the caller's timestamp
    // instead, same pre-reboot housekeeping as CMD_REBOOT otherwise.
    uint32_t secs;
    memcpy(&secs, &sub[1], 4);
    if (dirty_contacts_expiry) {
#if BEEBO_ENABLE_REPEATER_ROLE
      if (isRepeater()) acl.save(_store->getPrimaryFS());
      else saveContacts();
#else
      saveContacts();
#endif
    }
    board.rebootWithTime(secs);
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: the whole GET/SET_REGION_* cluster below talks to region_map
  // directly (a RegionMap member that only exists in this build config --
  // see Beebo.h), so it's compiled out entirely for a companion-only static
  // build rather than gated per-branch; an unmatched opcode there falls
  // through to the ERR_CODE_UNSUPPORTED_CMD catch-all at the end of this
  // chain, same as any other opcode this build doesn't implement.
  } else if (sub[0] == BEEBO_CMD_GET_REGION_HOME) {
    // beebo: region_map is RegionMap-backed (see
    // Beebo.h's own member), not a ComPrefs/NodePrefs field, so this isn't
    // wired through tlvGet*/tlvSet*/GET_PREFS_TLV like the rest of
    // repeater.* -- it has its own separate load()/save() pair instead.
    // loadRoleState() (begin()) guarantees region_map.load() has actually
    // run even if this device booted straight into companion role and
    // never switched into repeater this session -- otherwise a home region
    // saved on a previous boot would read back as unset simply because it
    // was never loaded into RAM.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_REGION_HOME;
    RegionEntry* home = region_map.getHomeRegion();
    const char* name = home ? home->name : "";
    if (*name == '#') name++;   // RegionMap.cpp's skip_hash() convention (file-static, reimplemented here)
    int name_len = strlen(name);
    if (name_len > 0) memcpy(&out_frame[2], name, name_len);
    _serial->writeFrame(out_frame, 2 + name_len);
  } else if (sub[0] == BEEBO_CMD_SET_REGION_HOME) {
    if (sub_len <= 1) {
      // empty payload -- clear the home region, same as CommonCLI's text-CLI
      // 'region home' with no argument doesn't do (that's get-only there),
      // but a real "unset" action is useful over the binary protocol.
      region_map.setHomeRegion(NULL);
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    } else {
      char name[32];
      size_t name_len = min((size_t)sub_len - 1, sizeof(name) - 1);
      memcpy(name, &sub[1], name_len);
      name[name_len] = 0;
      RegionEntry* entry = region_map.findByName(name);
      if (entry == NULL) {
        // beebo: no binary opcode defines/edits regions yet (RepeaterLink
        // text-CLI 'region put <name>' only, see protocol.yaml's desc) --
        // setting home to a name that was never defined is a caller error,
        // not something this command can silently fix up.
        writeErrFrame(ERR_CODE_NOT_FOUND);
      } else {
        region_map.setHomeRegion(entry);
        region_map.save(_store->getPrimaryFS(), "/beebo_regions");
        writeOKFrame();
      }
    }
  } else if (sub[0] == BEEBO_CMD_GET_REGION_DEFAULT) {
    // beebo: restores stock simple_repeater's own default-scope mechanism
    // (RegionMap's independent default_id/getDefaultRegion(), distinct from
    // home_id above) -- see getDefaultScope()'s own comment.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_REGION_DEFAULT;
    RegionEntry* def = region_map.getDefaultRegion();
    const char* name = def ? def->name : "";
    if (*name == '#') name++;   // RegionMap.cpp's skip_hash() convention (file-static, reimplemented here)
    int name_len = strlen(name);
    if (name_len > 0) memcpy(&out_frame[2], name, name_len);
    _serial->writeFrame(out_frame, 2 + name_len);
  } else if (sub[0] == BEEBO_CMD_SET_REGION_DEFAULT) {
    if (sub_len <= 1) {
      // empty payload -- clear the default region, mirroring SET_REGION_HOME's
      // own empty-payload convention (CommonCLI's text-CLI 'region default'
      // with no argument is get-only, same asymmetry as 'region home').
      region_map.setDefaultRegion(NULL);
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    } else {
      char name[32];
      size_t name_len = min((size_t)sub_len - 1, sizeof(name) - 1);
      memcpy(name, &sub[1], name_len);
      name[name_len] = 0;
      // beebo: unlike SET_REGION_HOME, auto-creates -- matches CommonCLI.cpp's
      // real "region default <name>" handler exactly (handleRegionCmd's own
      // putRegion() fallback), not SET_REGION_HOME's stricter ERR_CODE_NOT_FOUND.
      RegionEntry* entry = region_map.findByName(name);
      PutRegionError put_err = PUT_REGION_OK;
      if (entry == NULL) {
        entry = region_map.putRegion(name, 0, 0, &put_err);  // auto-create under wildcard root
        if (entry) entry->flags = 0;  // allow-flood
      }
      if (entry == NULL) {
        writeErrFrame(put_err == PUT_REGION_ILLEGAL_NAME ? ERR_CODE_ILLEGAL_ARG : ERR_CODE_TABLE_FULL);
      } else {
        region_map.setDefaultRegion(entry);
        region_map.save(_store->getPrimaryFS(), "/beebo_regions");
        writeOKFrame();
      }
    }
  } else if (sub[0] == BEEBO_CMD_GET_REGION_LIST) {
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_REGION_LIST;
    // mask=0, invert=false: every defined region matches (see
    // RegionMap::exportNamesTo()'s own "!(flags & mask)" test), not just
    // flood-allowed ones -- this is a read-only inventory, not the
    // flood-scoping query handleAnonRegionsReq() runs over the mesh.
    int n = region_map.exportNamesTo((char*)&out_frame[2], MAX_FRAME_SIZE - 2, 0);
    _serial->writeFrame(out_frame, 2 + n);
  } else if (sub[0] == BEEBO_CMD_GET_REGION_TREE) {
    // beebo: full hierarchical export (name + parent nesting + home/flood-
    // flag markers) -- same content CommonCLI's bare 'region' command
    // shows, unlike GET_REGION_LIST's flat names-only inventory.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_REGION_TREE;
    size_t n = region_map.exportTo((char*)&out_frame[2], MAX_FRAME_SIZE - 2);
    _serial->writeFrame(out_frame, 2 + n);
  } else if (sub[0] == BEEBO_CMD_SET_REGION_PUT && sub_len >= 2) {
    // beebo: full parity with CommonCLI's text-CLI 'region put <name>
    // [parent]' -- payload is name\0parent\0 (SET_WIFI_CREDS' two-string
    // convention), parent empty/omitted meaning the wildcard root. Unlike
    // the text-CLI, which needs a separate 'region save' afterward, this
    // auto-persists on success (same choice as SET_REGION_HOME above).
    const char* name = (const char*)&sub[1];
    size_t name_len = strnlen(name, (size_t)sub_len - 1);
    const char* parent_name = (name_len + 1 < (size_t)sub_len - 1) ? (name + name_len + 1) : "";
    RegionEntry* parent = (*parent_name != 0) ? region_map.findByName(parent_name) : &region_map.getWildcard();
    if (parent == NULL) {
      writeErrFrame(ERR_CODE_NOT_FOUND);   // named parent doesn't exist
    } else {
      PutRegionError put_err = PUT_REGION_OK;
      RegionEntry* region = region_map.putRegion(name, parent->id, 0, &put_err);
      if (region == NULL) {
        writeErrFrame(put_err == PUT_REGION_ILLEGAL_NAME ? ERR_CODE_ILLEGAL_ARG : ERR_CODE_TABLE_FULL);
      } else {
        region->flags = 0;   // CommonCLI's own post-put reset: flood-allowed by default
        region_map.save(_store->getPrimaryFS(), "/beebo_regions");
        writeOKFrame();
      }
    }
  } else if (sub[0] == BEEBO_CMD_SET_REGION_REMOVE && sub_len >= 2) {
    char name[32];
    size_t name_len = min((size_t)sub_len - 1, sizeof(name) - 1);
    memcpy(name, &sub[1], name_len);
    name[name_len] = 0;
    RegionEntry* region = region_map.findByName(name);
    if (region == NULL) {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    } else if (!region_map.removeRegion(*region)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);   // still has child regions
    } else {
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_SET_REGION_ALLOWF && sub_len >= 2) {
    char name[32];
    size_t name_len = min((size_t)sub_len - 1, sizeof(name) - 1);
    memcpy(name, &sub[1], name_len);
    name[name_len] = 0;
    RegionEntry* region = region_map.findByName(name);
    if (region == NULL) {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    } else {
      region->flags &= ~REGION_DENY_FLOOD;
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_SET_REGION_DENYF && sub_len >= 2) {
    char name[32];
    size_t name_len = min((size_t)sub_len - 1, sizeof(name) - 1);
    memcpy(name, &sub[1], name_len);
    name[name_len] = 0;
    RegionEntry* region = region_map.findByName(name);
    if (region == NULL) {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    } else {
      region->flags |= REGION_DENY_FLOOD;
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_SET_REGION_DEF && sub_len >= 2) {
    // beebo: full parity with CommonCLI's text-CLI 'region def <segments>'
    // (src/helpers/CommonCLI.cpp's processRegionDefSegment()/handleRegionCmd()
    // 'region def' branch) -- reimplemented here rather than reused since
    // that logic is file-static in CommonCLI.cpp and multi_role doesn't
    // link against CommonCLI at all (it talks to RegionMap directly).
    // Space-separated tokens, each 'name' or 'name|jump'/'name,jump'.
    char payload[MAX_FRAME_SIZE];
    size_t payload_len = min((size_t)sub_len - 1, sizeof(payload) - 1);
    memcpy(payload, &sub[1], payload_len);
    payload[payload_len] = 0;

    // beebo: a chain must be all-or-nothing. putRegion() commits each
    // segment (num_regions++) the instant it succeeds, so a failure on a
    // *later* segment (bad jump target, empty name/jump, table full) used
    // to leave every earlier segment permanently in the table -- the
    // command as a whole reports an error, giving the caller no reason to
    // think anything was created, while num_regions quietly grows on every
    // retry of a chain with a typo further down it. created_ids tracks
    // every segment actually created (not re-parented -- putRegion()
    // reuses an existing entry by name without incrementing num_regions,
    // and reparenting an existing entry is never this command's own leak
    // to roll back) so a failure can undo them, last-created first --
    // always safe since a segment's parent is always an entry created
    // earlier in this same chain (or pre-existing), never a later one, so
    // nothing still has a leftover entry as its parent by the time this
    // walks back to it.
    uint16_t created_ids[MAX_REGION_ENTRIES];
    int created_count = 0;

    RegionEntry* cursor = &region_map.getWildcard();
    int err_code = 0;
    char* p = payload;
    while (*p == ' ') p++;
    while (*p) {
      char* tok = p;
      while (*p && *p != ' ') p++;
      if (*p) *p++ = '\0';
      while (*p == ' ') p++;

      char* jump = nullptr;
      for (char* q = tok; *q; q++) {
        if (*q == '|' || *q == ',') {
          *q = '\0';
          jump = q + 1;
          break;
        }
      }
      char* name = tok;
      if (*name == '\0') { err_code = ERR_CODE_ILLEGAL_ARG; break; }
      if (jump && *jump == '\0') { err_code = ERR_CODE_ILLEGAL_ARG; break; }

      int count_before = region_map.getCount();
      PutRegionError put_err = PUT_REGION_OK;
      RegionEntry* r = region_map.putRegion(name, cursor->id, 0, &put_err);
      if (r == NULL) {
        err_code = put_err == PUT_REGION_ILLEGAL_NAME ? ERR_CODE_ILLEGAL_ARG : ERR_CODE_TABLE_FULL;
        break;
      }
      r->flags = 0;   // flood-allowed by default, same as SET_REGION_PUT
      if (region_map.getCount() > count_before && created_count < MAX_REGION_ENTRIES) {
        created_ids[created_count++] = r->id;
      }

      if (jump) {
        RegionEntry* j = region_map.findByNamePrefix(jump);
        if (j == NULL) { err_code = ERR_CODE_NOT_FOUND; break; }
        cursor = j;
      } else {
        cursor = r;
      }
    }
    if (err_code) {
      for (int i = created_count - 1; i >= 0; i--) {
        RegionEntry* r = region_map.findById(created_ids[i]);
        if (r != NULL) region_map.removeRegion(*r);
      }
      writeErrFrame(err_code);
    } else {
      region_map.save(_store->getPrimaryFS(), "/beebo_regions");
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_SET_REGION_SAVE) {
    // beebo: every other SET_REGION_* opcode already auto-persists on
    // success, making this a no-op in practice -- kept for backward
    // compatibility with CommonCLI's text-CLI 'region save', which callers
    // may still send out of habit after a 'region put'/'region def'.
    if (region_map.save(_store->getPrimaryFS(), "/beebo_regions")) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_FILE_IO_ERROR);
    }
#endif
  } else if (sub[0] == BEEBO_CMD_CLEAR_PROFILE) {
    profile_log.clear();
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_GET_PREFS_TLV && sub_len >= 2) {
    // beebo: sub[1] is a leading role byte (NODE_ROLE_COMPANION/
    // NODE_ROLE_REPEATER/NODE_ROLE_LIVE), scoping the whole triplet stream
    // to one role per call. sub[2] (optional, defaults to 0) is the
    // PREFS_TLV_FIELDS start index to resume from -- the table has grown
    // past what a single BLE-capped (176B) frame can hold, so the reply is
    // paginated: out_frame[2] carries next_index (== PREFS_TLV_FIELD_COUNT
    // when this page reached the end) for the caller to request the next
    // page with, same "keep calling until done" shape as GET_MONRING/
    // GET_NEIGHBORS. Page cap must track _app_max_tx (the size this app
    // actually negotiated via CMD_SET_XFER_CAPS), not the transport's raw
    // getMaxSendFrameSize() -- a BULK_XFER-capable transport (WiFi/USB) can
    // physically carry up to MAX_SEND_FRAME_SIZE, but an app that never
    // negotiated stays capped at its own frame-size limit (e.g. meshcore-py's
    // hardcoded 300-byte parser ceiling), and silently drops anything larger
    // with no error visible to the caller. Same pattern GET_MONRING already
    // uses at _app_max_tx below.
    uint8_t start_index = (sub_len >= 3) ? sub[2] : 0;
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_PREFS_TLV;
    size_t next_index = 0;
    size_t page_cap = _app_max_tx;
    if (page_cap > MAX_SEND_FRAME_SIZE) page_cap = MAX_SEND_FRAME_SIZE;  // out_frame is only sized for this much
    int n = encodePrefsTlv(resolveRoleByte(sub[1]), start_index, &out_frame[3], page_cap - 3, &next_index);
    out_frame[2] = (next_index >= PREFS_TLV_FIELD_COUNT) ? 0xFF : (uint8_t)next_index;
    _serial->writeFrame(out_frame, 3 + n);
  } else if (sub[0] == BEEBO_CMD_SET_PREFS_TLV && sub_len >= 3) {
    // beebo: apply every triplet to its own store's RAM cache, then flush at
    // the batch end -- at most one write per store, however many fields the
    // payload changed, and correct for a payload mixing the two stores (a
    // NodePrefs-backed field like wifi_ssid used to fall through the
    // ComPrefs-only flush that stood here and never reach flash at all).
    // beebo: sub[1] is the leading role byte, same as GET_PREFS_TLV above;
    // triplets start at sub[2].
    uint8_t role = resolveRoleByte(sub[1]);
    beginPrefsBatch();
    size_t pos = 2;
    bool all_ok = true;
    while (pos < (size_t)sub_len) {
      if (!applyPrefsTlvTriplet(role, sub, sub_len, pos)) all_ok = false;
    }
    endPrefsBatch();
    if (all_ok) writeOKFrame(); else writeErrFrame(ERR_CODE_ILLEGAL_ARG);
  } else if (sub[0] == BEEBO_CMD_GET_FULL_VERSION) {
    // beebo: DEVICE_INFO's 'ver' field is a fixed 20 bytes (upstream MeshCore
    // wire format, shared with the meshcore Python library's frame parser --
    // widening it there would desync every field after it) -- too short for
    // beebo's role/mode-suffixed FIRMWARE_VERSION (e.g. "...multirole.dbg").
    // This sub-command exists purely to hand back the untruncated string.
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_FULL_VERSION;
    size_t ver_len = strlen(FIRMWARE_VERSION);
    memcpy(&out_frame[2], FIRMWARE_VERSION, ver_len);
    _serial->writeFrame(out_frame, 2 + ver_len);
  } else if (sub[0] == BEEBO_CMD_GET_TUNE_ENABLED) {
    // beebo: RAM-only, like BEEBO_CMD_GET/SET_QUIET below -- no tlvGet*/
    // tlvSet* wrapper (nothing to persist) and no flushDirtyPrefs() on SET.
    // Dynamic-tuning optimizer on/off ; always
    // reads back 0 after a reboot. Mirrors handleCommand()'s "get"/"set
    // tune.enabled" text-CLI keys (same _tune_enabled field), so both paths
    // read the exact same live state -- USB text CLI and this binary path
    // (reachable over BLE/TCP/USB) can't drift apart.
    out_frame[0] = RESP_CODE_OK;
    memset(&out_frame[1], 0, 4);
    out_frame[1] = _tune_enabled ? 1 : 0;
    _serial->writeFrame(out_frame, 5);  // 5B: app lib only parses "value" out of a 5B OK frame
  } else if (sub[0] == BEEBO_CMD_SET_TUNE_ENABLED && sub_len >= 2) {
    setTuneEnabled(sub[1] != 0, EVENT_SOURCE_BINARY);
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_GET_TUNE_APPLIED_MASK) {
    // beebo: per-param live-actuation promotion (RAM-only, default 0 =
    // every param observe-only -- see _tune_applied_mask's declaration and
    // TuneController::Decision/isAppliable()).
    out_frame[0] = RESP_CODE_OK;
    memset(&out_frame[1], 0, 4);
    out_frame[1] = _tune_applied_mask;
    _serial->writeFrame(out_frame, 5);
  } else if (sub[0] == BEEBO_CMD_SET_TUNE_APPLIED_MASK && sub_len >= 2) {
    setTuneAppliedMask(sub[1], EVENT_SOURCE_BINARY);
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_GET_QUIET) {
    out_frame[0] = RESP_CODE_OK;
    memset(&out_frame[1], 0, 4);
    out_frame[1] = _bench_quiet ? 1 : 0;
    _serial->writeFrame(out_frame, 5);  // 5B: app lib only parses "value" out of a 5B OK frame
  } else if (sub[0] == BEEBO_CMD_SET_QUIET && sub_len >= 2) {
    if (sub[1]) {
      // beebo: bench IR-drop test -- sleep the radio + FEM and suspend
      // mesh/radio work in loop(), leaving only the VBAT ADC path (polled
      // via CMD_GET_STATS/STATS_TYPE_CORE) live for a bench meter to read
      // board current in isolation.
      board.loRaFEMControl.setSleepModeEnable();
      radio_driver.powerOff();
      _bench_quiet = true;
    } else {
      // beebo: live wake, no reboot needed -- same FEM-restore call
      // onAfterTransmit() already uses after every normal TX
      // (HeltecV4Board.cpp), plus RadioLibWrapper::wake() (mirrors
      // stopCW()'s own sleep-to-active recovery sequence). Clearing
      // _bench_quiet immediately lets loop()'s skip_radio drop on the
      // very next iteration, resuming mesh/radio work with no delay.
      board.loRaFEMControl.setRxModeEnable();
      radio_driver.wake();
      _bench_quiet = false;
    }
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_SET_CW && sub_len >= 2) {
    // beebo: bench CW carrier for spectrum-analyzer power measurement.
    // [1]=on (0=stop, 1=start), optional [2..3]=max duration in seconds (LE).
    uint8_t on = sub[1];
    if (on) {
      uint16_t max_secs = (sub_len >= 4) ? (sub[2] | (sub[3] << 8)) : 30;
      radio_driver.startCW((uint32_t)max_secs * 1000);
    } else {
      radio_driver.stopCW();
    }
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_SET_TX_OPTIMIZE && sub_len >= 2) {
    // beebo: select RadioLib PA optimization. [1]=1 -> efficiency table (default,
    // non-monotonic low end); [1]=0 -> fixed PA config (monotonic). Re-applies
    // the current TX power so the change takes effect immediately.
    radio_driver.setTxPowerOptimize(sub[1] != 0);
    radio_driver.setTxPower(_role_state->prefs.tx_power_dbm);
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_GET_NEIGHBORS) {
    // beebo: stream the direct (zero-hop) neighbour table. START (count + our
    // current clock, so the app can age each heard_timestamp) is sent here;
    // one NEIGHBOR frame per loop() tick then follows, paced against
    // isWriteBusy() the same way the contacts iterator is (see
    // appendLinkQueueDropEvents' _neighread branch below) -- MAX_NEIGHBOURS
    // (16) entries dumped inline in one round trip could overflow the
    // transport's 6-slot send_queue in a single burst otherwise.
    if (_neighread.active) {
      writeErrFrame(ERR_CODE_BAD_STATE); // stream already busy
    } else {
      uint32_t count = 0;
      for (int i = 0; i < MAX_NEIGHBOURS; i++) if (neighbours[i].heard_timestamp) count++;
      int n = 0;
      out_frame[n++] = RESP_CODE_BEEBO;
      out_frame[n++] = BEEBO_RESP_NEIGHBORS_START;
      memcpy(&out_frame[n], &count, 4); n += 4;
      uint32_t now = getRTCClock()->getCurrentTime();
      memcpy(&out_frame[n], &now, 4); n += 4;
      _serial->writeFrame(out_frame, n);
      _neighread.active = true;
      _neighread.index = 0;
    }
  } else if (sub[0] == BEEBO_CMD_SET_NEIGHBOR_REMOVE && sub_len >= 3) {
    // beebo: mirrors CommonCLI's text-CLI 'neighbor.remove <hex>' -- prunes
    // one direct-neighbour table entry (RAM-only; re-populated as adverts
    // are heard again), matched the same prefix-compare way
    // Beebo::putNeighbour() matches an existing slot to an incoming advert
    // (compare over min(request len, stored pubkey_len) bytes).
    uint8_t len = sub[1];
    if (len > PUB_KEY_SIZE) len = PUB_KEY_SIZE;
    bool found = (len > 0 && sub_len >= 2 + (int)len) ? removeNeighborByPrefix(&sub[2], len) : false;
    if (found) {
      writeOKFrame();
    } else {
      writeErrFrame(ERR_CODE_NOT_FOUND);
    }
  } else if (sub[0] == BEEBO_CMD_GET_MONRING) {
    // beebo: read one page of the monitor ring, OLDEST first. Request carries
    // an optional 4-byte LE after_seq (absent or 0 = from the oldest record),
    // and an optional following 4-byte LE after_ts: the capture-time epoch of
    // the record at after_seq, as the client itself last recorded it. Records
    // come back oldest-first, walking forward toward the head; the client
    // fixes the stop point from the first page's `next` and repeats with
    // after_seq += returned until the cursor reaches `next`. The read is
    // bracketed by pauseForRead()/resumeAfterRead() so the ring is frozen
    // (no writes) for the whole walk, making the forward scan trivially safe.
    uint32_t after_seq = (sub_len >= 5) ? (sub[1] | ((uint32_t)sub[2] << 8)
                                        | ((uint32_t)sub[3] << 16)
                                        | ((uint32_t)sub[4] << 24)) : 0;
    uint32_t after_ts = (sub_len >= 9) ? (sub[5] | ((uint32_t)sub[6] << 8)
                                        | ((uint32_t)sub[7] << 16)
                                        | ((uint32_t)sub[8] << 24)) : 0;
    // Stats-only sentinel: return the ring header with zero records and WITHOUT
    // pausing capture, so `beebo monitor` (default) is fully non-mutating.
    if (after_seq == 0xFFFFFFFFu) {
      uint32_t returned = 0;
      int i = fillMonRingFrame(out_frame, 0, 0, &returned, false, false);  // max_len 0 -> header only
      _serial->writeFrame(out_frame, i);
      return;
    }
    // beebo: `_next_seq` resets to 0 on every reboot/clear (MonRing.h's
    // clear()/init()), so a resumed after_seq can, after enough traffic,
    // land back inside a numeric range that looks exactly as plausible as
    // before -- a plain seq comparison can't tell a genuine resume apart
    // from that aliasing. Capture time can: it only moves forward, so if
    // the ring's current oldest resident record is already newer than the
    // client's own after_ts, nothing it's asking to resume from is still
    // there regardless of what after_seq claims. Override to a full read
    // from the oldest record and report it via the reset flag, rather than
    // trusting a seq position that might belong to a different session
    // entirely.
    bool reset = false;
    if (after_seq != 0 && sub_len >= 9) {
      uint32_t oldest_ts = monring.startTime();
      if (oldest_ts != 0 && after_ts < oldest_ts) {
        after_seq = 0;
        reset = true;
      }
    }
    if (_app_stream) {
      // beebo: negotiated streaming -> arm the loop pump; frames are emitted one
      // per loop() from checkSerialInterface() so radio RX keeps being serviced.
      // Snapshot `next` as the stop point; the client no longer drives
      // termination in stream mode. Pause capture for the duration of the read.
      monring.pauseForRead();
      _monread.active = true;
      _monread.after_seq = after_seq;
      _monread.next = monring.nextSeq();
      _monread.reset = reset;
      // Only the very first request of a read may need start-refs spliced in;
      // emitStartRefs() itself stays empty until a kind has actually been
      // evicted (see MonRing::_store()), so this can't duplicate real records.
      _monread.first_page = (after_seq == 0);
      return;
    }
    size_t page_cap = _app_max_tx;   // #1: larger paged frames if negotiated (else 176)
    monring.pauseForRead();
    uint32_t returned = 0;
    // See the BULK_XFER arm site above: emitStartRefs() is self-limiting to
    // genuinely evicted kinds, so gating on after_seq==0 alone is enough.
    int i = fillMonRingFrame(out_frame, after_seq, page_cap, &returned, after_seq == 0, reset);
    monring.resumeAfterRead();
    _serial->writeFrame(out_frame, i);
  } else if (sub[0] == BEEBO_CMD_SET_MONRING && sub_len >= 2) {
    // beebo: control capture. op 0=pause, 1=resume, 2=clear (pause/resume
    // also persist, bit7 of monring_config). Setting the capture config
    // itself is SET_PREFS_TLV(KEY_MONRING_CONFIG)'s job now (this opcode's
    // old op=3 duplicated that exact same tlvSetMonringConfig call).
    switch (sub[1]) {
      case 0:
        _role_state->prefs.monring_config &= ~MON_CAP_ENABLED;
        monring.setConfig(_role_state->prefs.monring_config);
        // beebo: the per-role dirty-routing pattern -- same rationale
        // as tlvSetMonringConfig above.
        savePrefs();
        break;
      case 1:
        _role_state->prefs.monring_config |= MON_CAP_ENABLED;
        monring.setConfig(_role_state->prefs.monring_config);
        savePrefs();
        break;
      case 2:
        // beebo: not radioIsIdle()-verified -- see initMonRing()'s comment.
        resetBattTrendRef(_batt_state, _cached_batt_mv, _board.batt_present);
        monring.clear((uint32_t)getRTCClock()->getCurrentTime(), buildRadioRecord(), buildEnvRecord());
        break;
      default: writeErrFrame(ERR_CODE_ILLEGAL_ARG); return;
    }
    writeOKFrame();
  } else if (sub[0] == BEEBO_CMD_SET_XFER_CAPS && sub_len >= 4) {
    // beebo: negotiate bulk-transfer caps for this session. Grant
    // min(requested, this transport's send cap), never below the legacy 176 so
    // nothing shrinks; the reply echoes the granted caps. An old firmware lacks
    // this command and returns ERR, so a capable client detects the miss and
    // stays on the legacy paged path. Caps reset on each connect (DEVICE_QUERY).
    uint16_t want = sub[1] | ((uint16_t)sub[2] << 8);
    uint16_t cap = (uint16_t)_serial->getMaxSendFrameSize();
    _app_max_tx = want < cap ? want : cap;
    if (_app_max_tx < MAX_FRAME_SIZE) _app_max_tx = MAX_FRAME_SIZE;
    _app_stream = (sub[3] & 0x01) != 0;
    int i = 0;
    out_frame[i++] = RESP_CODE_BEEBO;
    out_frame[i++] = BEEBO_RESP_XFER_CAPS;
    out_frame[i++] = _app_max_tx & 0xFF;
    out_frame[i++] = _app_max_tx >> 8;
    out_frame[i++] = _app_stream ? 1 : 0;
    _serial->writeFrame(out_frame, i);
  // Beebo-specific OTA commands (128-130)
  } else if (sub[0] == BEEBO_CMD_OTA_BEGIN) {
    if (ota_partition != NULL) {
      esp_ota_abort(ota_handle);
      ota_handle = 0;
      ota_partition = NULL;
    }
    ota_priority = false;
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
      writeErrFrame(ERR_CODE_BAD_STATE);
    } else {
      // Payload: [total_size (4B LE)][priority (1B, optional, default 0)].
      // priority asks loop() to skip radio dispatch while this transfer is
      // active, on whichever transport is carrying it (see loop()) --
      // older CLIs that don't send the byte get priority=0, i.e. today's
      // behaviour. Chunks always stream straight to flash; esp_ota_write
      // batches partial sectors internally, so there is no PSRAM buffer.
      uint32_t total_size = 0;
      if (sub_len >= 5) {
        memcpy(&total_size, &sub[1], 4);
      }
      if (sub_len >= 6) {
        ota_priority = sub[5] != 0;
      }
      if (total_size > ota_partition->size) {
        ota_partition = NULL;
        writeErrFrame(ERR_CODE_BAD_STATE);
      } else {
        esp_err_t err = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
        if (err != ESP_OK) {
          ota_partition = NULL;
          writeErrFrame(ERR_CODE_BAD_STATE);
        } else {
          // -2: [CMD_BEEBO][BEEBO_CMD_OTA_WRITE] header bytes ahead of the
          // chunk in the incoming frame (was -1 pre-CMD_BEEBO-umbrella, when
          // OTA_WRITE frames carried only a single top-level opcode byte).
          uint16_t chunk_size = _serial->getMaxRecvFrameSize() - 2;
          out_frame[0] = RESP_CODE_BEEBO;
          out_frame[1] = BEEBO_RESP_OTA_BEGIN;
          memcpy(&out_frame[2], &chunk_size, 2);
          _serial->writeFrame(out_frame, 4);
          _ota_last_activity = _ms->getMillis();
        }
      }
    }
  } else if (sub[0] == BEEBO_CMD_OTA_WRITE && sub_len > 1) {
    if (ota_partition == NULL) {
      writeErrFrame(ERR_CODE_BAD_STATE);
    } else {
      _ota_last_activity = _ms->getMillis();
      // Stream the chunk directly to flash.
      esp_err_t err = esp_ota_write(ota_handle, &sub[1], sub_len - 1);
      if (err != ESP_OK) {
        esp_ota_abort(ota_handle);
        ota_handle = 0;
        ota_partition = NULL;
        writeErrFrame(ERR_CODE_FILE_IO_ERROR);
      } else {
        writeOKFrame();
      }
    }
  } else if (sub[0] == BEEBO_CMD_OTA_END) {
    if (ota_partition == NULL) {
      writeErrFrame(ERR_CODE_BAD_STATE);
    } else {
      esp_err_t err = esp_ota_end(ota_handle);
      if (err != ESP_OK) MESH_DEBUG_PRINTLN("esp_ota_end failed: 0x%x", err);
      if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(ota_partition);
        if (err != ESP_OK) MESH_DEBUG_PRINTLN("esp_ota_set_boot_partition failed: 0x%x", err);
      }
      ota_handle = 0;
      ota_partition = NULL;
      if (err != ESP_OK) {
        writeErrFrame(ERR_CODE_FILE_IO_ERROR);
      } else {
        writeOKFrame();
        // Defer the reboot so the OK response actually flushes over the
        // wire (writeOKFrame only queues it; it's sent when the send queue
        // drains in checkRecvFrame). Rebooting immediately would cut the
        // connection before the CLI sees the response.
        _ota_restart_time = futureMillis(750);
        // beebo: optional trailing 4B LE host timestamp (older CLIs send
        // none, sub_len == 1) -- see loopTransports()'s deferred-restart
        // block and _ota_restart_ts's own comment.
        _ota_restart_ts = 0;
        if (sub_len >= 5) memcpy(&_ota_restart_ts, &sub[1], 4);
      }
    }
  } else if (sub[0] == BEEBO_CMD_NODE_DISCOVER && sub_len >= 2) {
    // beebo: companion binary entry point for the "node discover" protocol
    // (see protocol.yaml and sendNodeDiscoverReq()) -- previously only
    // reachable via the USB-only text-CLI "discover.neighbors". Request is
    // fire-and-forget (zero-hop broadcast); matching responses trickle into
    // the direct-neighbour table over the following ~60s, same as any other
    // NODE_DISCOVER_RESP -- poll BEEBO_CMD_GET_NEIGHBORS after a short wait.
    uint8_t filter = sub[1];
    bool prefix_only = sub_len >= 3 && sub[2] != 0;
    if (filter == 0) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      sendNodeDiscoverReq(filter, prefix_only);
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_GET_ADVERT_PATHS) {
    // beebo: bulk counterpart to upstream's single-pubkey CMD_GET_ADVERT_PATH
    // -- advert_paths[] only ever holds ADVERT_PATH_TABLE_SIZE=16 entries
    // total, so streaming the whole thing (same START/entry/END shape as
    // BEEBO_CMD_GET_NEIGHBORS) and matching pubkey_prefix locally
    // client-side is strictly better than one CMD_GET_ADVERT_PATH per
    // contact -- see protocol.yaml. START is sent here; one ADVERT_PATH
    // frame per loop() tick then follows, paced against isWriteBusy() the
    // same way GET_NEIGHBORS/contacts are, rather than dumped inline in one
    // burst that could overflow the transport's 6-slot send_queue.
    if (_pathread.active) {
      writeErrFrame(ERR_CODE_BAD_STATE); // stream already busy
    } else {
      uint32_t count = 0;
      for (int i = 0; i < ADVERT_PATH_TABLE_SIZE; i++) if (advert_paths[i].recv_timestamp) count++;
      int n = 0;
      out_frame[n++] = RESP_CODE_BEEBO;
      out_frame[n++] = BEEBO_RESP_ADVERT_PATHS_START;
      memcpy(&out_frame[n], &count, 4); n += 4;
      _serial->writeFrame(out_frame, n);
      _pathread.active = true;
      _pathread.index = 0;
    }
  } else if (sub[0] == BEEBO_CMD_GET_PUBLIC_KEY && sub_len >= 2) {
    // beebo: PER_ROLE_IDENTITY -- self_info only ever reflects the live
    // role's self_id, so companion.public_key/repeater.public_key read
    // through here instead to reach either role's pubkey regardless of
    // which is live. Read-only: never falls back to generating/persisting
    // a new identity for a role that's never been switched into.
    uint8_t role = resolveRoleByte(sub[1]);
    if (role != NODE_ROLE_COMPANION && role != NODE_ROLE_REPEATER) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      mesh::LocalIdentity id;
      bool have_id;
      if (role == _board.role) {
        id = self_id;
        have_id = true;
      } else {
        have_id = _store->loadRoleIdentity(role, id);
      }
      if (!have_id) {
        writeErrFrame(ERR_CODE_NOT_FOUND);
      } else {
        out_frame[0] = RESP_CODE_BEEBO;
        out_frame[1] = BEEBO_RESP_PUBLIC_KEY;
        memcpy(&out_frame[2], id.pub_key, PUB_KEY_SIZE);
        _serial->writeFrame(out_frame, 2 + PUB_KEY_SIZE);
      }
    }
  } else if (sub[0] == BEEBO_CMD_GENERATE_IDENTITY && sub_len >= 2) {
#if ENABLE_PRIVATE_KEY_IMPORT
    // beebo: same destructive-write capability class as CMD_IMPORT_PRIVATE_KEY
    // (overwrites a role's identity), gated behind the same build flag.
    uint8_t role = resolveRoleByte(sub[1]);
    if (role != NODE_ROLE_COMPANION && role != NODE_ROLE_REPEATER) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      mesh::LocalIdentity id = generateFreshIdentity();
      if (!_store->saveRoleIdentity(role, id)) {
        writeErrFrame(ERR_CODE_FILE_IO_ERROR);
      } else {
        out_frame[0] = RESP_CODE_BEEBO;
        out_frame[1] = BEEBO_RESP_PUBLIC_KEY;
        memcpy(&out_frame[2], id.pub_key, PUB_KEY_SIZE);
        _serial->writeFrame(out_frame, 2 + PUB_KEY_SIZE);
        if (role == _board.role) {
          role_state_store[role].identity = id;
          self_id = id;
          // re-load contacts, to invalidate ecdh shared_secrets (companion-
          // only state, see begin()'s role guard) -- same as CMD_IMPORT_PRIVATE_KEY
          if (isCompanion()) {
            resetContacts();
            _store->loadContacts(this);
          }
#if BEEBO_ENABLE_REPEATER_ROLE
          if (isRepeater()) acl.load(_store->getPrimaryFS(), self_id);
#endif
        }
      }
    }
#else
    writeDisabledFrame();
#endif
  } else if (sub[0] == BEEBO_CMD_SET_NODE && sub_len >= 3) {
#if ENABLE_PRIVATE_KEY_IMPORT
    // beebo: IDENTITY_SWITCH -- atomic import + name-set + role-switch-if-
    // needed + reboot in one server-side call, so a host-side named-
    // identity "switch" primitive doesn't have a partial-failure window
    // between separate CMD_IMPORT_PRIVATE_KEY / SET_PREFS_TLV(KEY_NAME) /
    // SET_NODE_ROLE round trips. Payload: [role:1][name_len:1]
    // [name:name_len][privkey:64]. No reply -- the device restarts
    // immediately, same as REBOOT_WITH_TIME.
    uint8_t role = sub[1];
    uint8_t name_len = sub[2];
    if ((role != NODE_ROLE_COMPANION && role != NODE_ROLE_REPEATER)
        || !isNodeRoleBuiltIn(role)
        || sub_len != (size_t)(3 + name_len + 64)) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else if (isOTAActive() || _monread.active || _statread.active) {
      // beebo: checked before any write (same busy class
      // requestNodeRoleSwitch() itself refuses) so a busy device is left
      // completely untouched instead of ending up with a new identity but
      // a role switch that got rejected.
      writeErrFrame(ERR_CODE_BAD_STATE);
    } else {
      const uint8_t* name = &sub[3];
      const uint8_t* prv = &sub[3 + name_len];
      if (!mesh::LocalIdentity::validatePrivateKey(prv)) {
        writeErrFrame(ERR_CODE_ILLEGAL_ARG);
      } else {
        mesh::LocalIdentity identity;
        identity.readFrom(prv, 64);
        if (!_store->saveRoleIdentity(role, identity)) {
          writeErrFrame(ERR_CODE_FILE_IO_ERROR);
        } else {
          if (name_len > 0) {
            tlvSetName(this, role, name, name_len);
            // beebo: tlvSetName()/persistRoleSlot() only *immediately*
            // writes flash for a non-live role -- for the live role it
            // just marks role_state_store[role].prefs.dirty and defers to
            // the normal save_prefs/batch flush cycle, which never runs
            // here before the unconditional reboot below. Force it out
            // now (same unconditional-write reasoning
            // requestNodeRoleSwitch() itself uses for _board.role) so the
            // name change actually survives the reboot instead of being
            // silently lost.
            if (role == _board.role) commitPrefs();
          }
          // beebo: ARG/BUILTIN/BUSY already ruled out above -- only NOOP
          // (already this role) or REBOOTING (persisted) can come back
          // here. Either way this call always reboots next, so the result
          // itself needs no further handling.
          requestNodeRoleSwitch(role, EVENT_SOURCE_BINARY);
          board.reboot();  // doesn't return; no reply, matches REBOOT_WITH_TIME
        }
      }
    }
#else
    writeDisabledFrame();
#endif
  } else if (sub[0] == BEEBO_CMD_DEBUG_LOG_ENABLE && sub_len >= 2) {
    // beebo: DebugLog always targets usb_interface, so enabling it when USB
    // transport isn't even up on this build (board.usb_enabled=0) would
    // leave the stream with nowhere to go -- refuse instead of silently
    // accepting.
    if (!_usb_up) {
      writeErrFrame(ERR_CODE_BAD_STATE);
    } else {
      debug_log.setEnabled(sub[1] != 0);
      writeOKFrame();
    }
  } else if (sub[0] == BEEBO_CMD_GET_BOARD_ID) {
    // beebo: factory eFuse base MAC -- hardware-burned, stable across
    // reflashes/identity changes/role switches, unrelated to node.public_key
    // (ed25519) or node.role (shared). Not a wire MAC (no per-interface
    // offset, see esp_read_mac()), just a unique per-board ID.
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    out_frame[0] = RESP_CODE_BEEBO;
    out_frame[1] = BEEBO_RESP_BOARD_ID;
    memcpy(&out_frame[2], mac, 6);
    _serial->writeFrame(out_frame, 2 + 6);
  } else if (sub[0] == BEEBO_CMD_UNLOCK_ROLE_SECRETS && sub_len >= 1) {
    // beebo: SETTINGS_ISOLATION follow-up -- password-gated secret readback
    // for `beebo config backup`, checked against BeeboBoardPrefs.
    // board_password (GET_PREFS_TLV(KEY_OWNER_PASSWORD)) rather than ComPrefs.
    // password -- board_password is compiled into every build (static
    // companion/repeater included), so this opcode itself no longer needs
    // BEEBO_ENABLE_REPEATER_ROLE at all; only the repeater.guest.password
    // half of the reply does. An empty (never-set) owner_password always
    // fails closed -- it never accidentally matches an empty candidate --
    // never reveals which case applies (a mismatch and an unset password
    // both just get a plain ERROR, see protocol.yaml's desc) before
    // unlocking every secret this build actually has in one reply -- no
    // separate proof needed per secret, since a session that already
    // knows the owner password already has unrestricted write access to
    // all of them (a companion session can already overwrite any of them
    // blind, with no gate at all).
    char candidate[16];
    size_t pw_len = min((size_t)sub_len - 1, sizeof(candidate) - 1);
    memcpy(candidate, &sub[1], pw_len);
    candidate[pw_len] = 0;
    if (_board.board_password[0] == 0
        || strcmp(candidate, _board.board_password) != 0) {
      writeErrFrame(ERR_CODE_ILLEGAL_ARG);
    } else {
      // beebo: three NUL-delimited fields -- guest_password, wifi_password,
      // repeater_password -- always in this order, even on a build where
      // the repeater-only ones are empty (no ComPrefs at all), so the
      // client-side parse never has to special-case which fields exist.
      int j = 2;
      out_frame[0] = RESP_CODE_BEEBO;
      out_frame[1] = BEEBO_RESP_ROLE_SECRETS;
#if BEEBO_ENABLE_REPEATER_ROLE
      // beebo: repeater's own guest_password/password always come from its
      // own resident slot (role_state_store[NODE_ROLE_REPEATER]), NOT
      // _role_state (whichever role is currently live) -- fixed
      // 2026-08-10, same bug class as the SET_REPEATER_PASSWORD/
      // SET_GUEST_PASSWORD handlers above. wifi_pwd below is intentionally
      // still _role_state->prefs -- it's a companion-shared NodePrefs
      // field, not repeater-specific.
      const BeeboRoleState& rp_slot = role_state_store[NODE_ROLE_REPEATER];
      size_t gp_len = strlen(rp_slot.prefs.guest_password);
      memcpy(&out_frame[j], rp_slot.prefs.guest_password, gp_len); j += gp_len;
#endif
      out_frame[j++] = 0;
      size_t wp_len = strlen(_role_state->prefs.wifi_pwd);
      memcpy(&out_frame[j], _role_state->prefs.wifi_pwd, wp_len); j += wp_len;
      out_frame[j++] = 0;
#if BEEBO_ENABLE_REPEATER_ROLE
      size_t rp_len = strlen(rp_slot.prefs.password);
      memcpy(&out_frame[j], rp_slot.prefs.password, rp_len); j += rp_len;
#endif
      _serial->writeFrame(out_frame, j);
    }
  } else {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
  }
  } else {
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
    MESH_DEBUG_PRINTLN("ERROR: unknown command: %02X", cmd_frame[0]);
  }
}

static bool save_filter(const ContactInfo& c) {
  return c.type != ADV_TYPE_NONE;   // don't save the transient/anon entries
}

void Beebo::saveContacts() {
  _store->saveContacts(this, save_filter);
}

void Beebo::checkSerialInterface() {
  // beebo: raw-marker control byte, handled directly on usb_interface --
  // below and independent of _serial (the MultiSerialInterface aggregator)
  // entirely, so it works even while BLE/TCP holds the companion session,
  // and never triggers MultiSerialInterface::lockOn() the way a real
  // CMD_BEEBO frame over USB would (see pollRawControl()'s own comment).
  // Fire-and-forget by design -- no OK/ERR reply, this sits below the
  // app/session request-reply layer entirely.
  uint8_t raw_sub, raw_data;
  if (usb_interface.pollRawControl(raw_sub, raw_data)) {
    if (raw_sub == BEEBO_RAW_SUB_DEBUG_LOG_ENABLE) {
      debug_log.setEnabled(raw_data != 0);
    }
  }

  size_t len = _serial->checkRecvFrame(cmd_frame, _serial->getMaxRecvFrameSize());
  // beebo: while a monring stream is active (capture paused for it), a
  // well-behaved client sends nothing else until the terminator page — drop
  // any frame that arrives anyway rather than letting it abort the stream
  // and leave capture paused. The stream's own disconnect/reset paths are
  // the only things allowed to end it early (see CMD_DEVICE_QUERY handling).
  if (_monread.active || _statread.active) len = 0;
  if (len > 0) {
    // beebo: USB's DualModeSerialInterface delivers a
    // raw text-CLI line through this same checkRecvFrame() call, decided
    // fresh per command from its first byte (see DualModeSerialInterface.h).
    // Dispatch on the transport's own recorded classification
    // (lastRecvWasText(), BaseSerialInterface.h), not a guess from
    // cmd_frame[0]'s value -- opcode IDs aren't fully under this project's
    // control (shared with upstream MeshCore) and could collide with any
    // fixed byte-range heuristic in the future. Every non-dual-mode
    // interface always reports false here.
    if (_serial->lastRecvWasText()) {
      cmd_frame[len] = 0;  // null terminator, matching simple_repeater's own text CLI
      char reply[160];
      reply[0] = 0;
      handleCommand(0, (char*)cmd_frame, reply);  // NOTE: no sender_timestamp via local text CLI
      _serial->writeFrame((uint8_t*)reply, strlen(reply));
      return;
    }
    // Trace every command into the debug ring (recv before, done after) so a
    // stalled interactive session can be diagnosed post-mortem via debuglog,
    // and time it into the profiling ring (see `beebo profile`). Skip the
    // debuglog/profile fetches themselves so paginated reads don't perturb
    // their own rings, and skip OTA_WRITE (thousands of chunks per update)
    // for the same reason.
    bool trace = !((cmd_frame[0] == CMD_BEEBO && len >= 2 && cmd_frame[1] == BEEBO_CMD_OTA_WRITE)
                    || (cmd_frame[0] == CMD_GET_STATS && len >= 2
                        && (cmd_frame[1] == STATS_TYPE_TRANSPORT || cmd_frame[1] == STATS_TYPE_PROFILE)));
    if (trace) {
      // (cmd<<8)|sub so CMD_BEEBO sub-commands (OTA/WiFi/RF measurement/…)
      // and CMD_GET_STATS sub-types are distinguishable, unlike TLOG_CMD_*'s
      // old 1-byte outer-command-only detail. Only these two commands' second
      // byte is actually a sub-id; every other command's byte[1] is just
      // payload data (e.g. SET_RADIO_TX_POWER's power value), so folding it
      // in unconditionally would fragment their stats across many ids.
      // Also fed into transport_log.log() below (TransportLog.h's `detail`
      // is a full int32_t, plenty of room) so `beebo monitor transport`
      // shows the real sub-command instead of a generic "BEEBO"/"GET_STATS"
      // row for every one of these -- see debuglog.py's BEEBO_CMD_NAMES.
      uint16_t prof_id = (uint16_t)cmd_frame[0] << 8;
      if (len >= 2 && (cmd_frame[0] == CMD_BEEBO || cmd_frame[0] == CMD_GET_STATS))
        prof_id |= cmd_frame[1];
      transport_log.log(TLOG_CMD_RECV, prof_id);
      PROFILE_SCOPE(prof_id);
      // beebo: MON_COMMAND (MonRing.h) -- deliberately narrower than
      // `trace`: excludes every routine/repeated read path (all of
      // CMD_GET_STATS, plus the bulk-fetch CMD_BEEBO commands a UI polls or
      // pages through), not just the two `trace` already excludes, so this
      // stays an infrequent log of real admin/user actions (SET_*, OTA
      // begin/end, reboot, ...) instead of filling up with reads. Uses the
      // same command_id encoding as prof_id above, so an event's id is
      // directly comparable against a `beebo profile` capture.
      bool command_run_eligible = cmd_frame[0] != CMD_GET_STATS
          && !(cmd_frame[0] == CMD_BEEBO && len >= 2 &&
               (cmd_frame[1] == BEEBO_CMD_GET_MONRING ||
                cmd_frame[1] == BEEBO_CMD_GET_NEIGHBORS ||
                cmd_frame[1] == BEEBO_CMD_GET_PREFS_TLV));
      if (command_run_eligible) appendCommandRunEvent(prof_id);
      handleCmdFrame(len);
      transport_log.log(TLOG_CMD_DONE, prof_id);
    } else {
      handleCmdFrame(len);
    }
  } else if (_pending_disconnect && !_serial->isWriteBusy()) {
    _pending_disconnect = false;
    MESH_DEBUG_PRINTLN("Disconnecting serial interface (app request)");
    _serial->disconnectActive();
  } else if (_iter_started              // check if our ContactsIterator is 'running'
             && !_serial->isWriteBusy() // don't spam the Serial Interface too quickly!
  ) {
    ContactInfo contact;
    bool found = false;
    while (_iter.hasNext(this, contact)) {
      if (contact.type != ADV_TYPE_NONE) {
        found = true;
        break;
      }
    }

    if (found) {
      if (contact.lastmod > _iter_filter_since) { // apply the 'since' filter
        writeContactRespFrame(RESP_CODE_CONTACT, contact);
        if (contact.lastmod > _most_recent_lastmod) {
          _most_recent_lastmod = contact.lastmod; // save for the RESP_CODE_END_OF_CONTACTS frame
        }
      }
    } else { // EOF
      out_frame[0] = RESP_CODE_END_OF_CONTACTS;
      memcpy(&out_frame[1], &_most_recent_lastmod,
             4); // include the most recent lastmod, so app can update their 'since'
      _serial->writeFrame(out_frame, 5);
      _iter_started = false;
    }
  } else if (_monread.active && !_serial->isWriteBusy()) {
    // beebo: stream the monitor-ring read one frame per loop, paced by the transport
    // accepting the write. One frame/iteration keeps radio RX serviced; the read
    // resumes next loop. If writeFrame is refused (queue full) we leave the cursor
    // put and retry next iteration.
    uint32_t returned = 0;
    if (_monread.after_seq >= _monread.next) {
      // Cursor caught up to the snapshot taken when the stream was armed: emit
      // one empty terminator (header only, no records) so the client's stream
      // loop ends, then let capture resume.
      int n = fillMonRingFrame(out_frame, 0, 0, &returned, false, _monread.reset);
      // beebo: must be `== (size_t)n`, not `> 0` -- writeFrame() can return a
      // short (but nonzero) count on a partial write (see writeAll()'s own
      // comment in DualModeSerialInterface.cpp), which `> 0` would wrongly
      // treat as a fully-sent frame and advance past data that never
      // actually made it onto the wire.
      if (_serial->writeFrame(out_frame, n) == (size_t)n) {
        monring.resumeAfterRead();
        _monread.active = false;
      }
    } else {
      // Clamp to the *active* transport's current send limit: if the session
      // handed over to a smaller-frame transport after _app_max_tx was
      // negotiated, a page sized to the old limit would be refused by
      // writeFrame() forever (cursor never advances, read spins).
      size_t page_cap = _app_max_tx;
      size_t xport_cap = _serial->getMaxSendFrameSize();
      if (xport_cap < page_cap) page_cap = xport_cap;
      int n = fillMonRingFrame(out_frame, _monread.after_seq, page_cap, &returned, _monread.first_page, _monread.reset);
      // beebo: `== (size_t)n`, not `> 0` -- see the sibling check above.
      if (_serial->writeFrame(out_frame, n) == (size_t)n) {
        _monread.first_page = false;
        // returned==0 here would mean nothing left to send before reaching
        // `next` (shouldn't happen while paused, but guard against it too).
        if (returned == 0) {
          monring.resumeAfterRead();
          _monread.active = false;
        } else {
          _monread.after_seq += returned;
        }
      }
    }
  } else if (_statread.active && !_serial->isWriteBusy()) {
    // beebo: stream a GET_STATS(TRANSPORT/PROFILE) ring the same way
    // _monread streams GET_MONRING -- one page per loop, paced by the
    // transport accepting the write, so radio RX keeps being serviced.
    // Terminates on an empty/short page (offset reaches total), same stop
    // condition the legacy paginated reply already used.
    size_t page_cap = _app_max_tx;
    size_t xport_cap = _serial->getMaxSendFrameSize();
    if (xport_cap < page_cap) page_cap = xport_cap;
    int i = 0;
    out_frame[i++] = RESP_CODE_STATS;
    out_frame[i++] = _statread.kind;
    int hdr = i;
    i += 4;  // reserve total + offset
    uint16_t total = 0;
    size_t body_cap = page_cap > (size_t)i ? page_cap - i : 0;
    // beebo: serialize() returns bytes written, not records -- divide by the
    // per-record wire size to get the record count the offset/total fields
    // (and the client's own pagination) are actually counted in. Confirmed
    // live: using the raw byte count here overshoots `total` (a record
    // count) after a single page, ending the stream 9x/8x too early.
    const int per_event = (_statread.kind == STATS_TYPE_TRANSPORT) ? 9 : 8;
    int body_n = (_statread.kind == STATS_TYPE_TRANSPORT)
        ? transport_log.serialize(&out_frame[i], body_cap, _statread.offset, &total)
        : profile_log.serialize(&out_frame[i], body_cap, _statread.offset, &total);
    int n_records = body_n / per_event;
    i += body_n;
    out_frame[hdr + 0] = total & 0xFF;
    out_frame[hdr + 1] = (total >> 8) & 0xFF;
    out_frame[hdr + 2] = _statread.offset & 0xFF;
    out_frame[hdr + 3] = (_statread.offset >> 8) & 0xFF;
    // beebo: `== (size_t)i`, not `> 0` -- writeFrame() can short-write (see
    // DualModeSerialInterface.cpp's writeAll()); treating that as success
    // would advance the offset past data that never made it onto the wire.
    if (_serial->writeFrame(out_frame, i) == (size_t)i) {
      uint16_t new_offset = _statread.offset + (uint16_t)n_records;
      if (n_records == 0 || new_offset >= total) {
        _statread.active = false;
      } else {
        _statread.offset = new_offset;
      }
    }
  } else if (_neighread.active && !_serial->isWriteBusy()) {
    // beebo: stream GET_NEIGHBORS one entry per loop, same pacing as the
    // contacts iterator above.
    NeighbourInfo* nb = NULL;
    while (_neighread.index < MAX_NEIGHBOURS) {
      nb = &neighbours[_neighread.index++];
      if (nb->heard_timestamp) break;
      nb = NULL;
    }
    if (nb != NULL) {
      int j = 0;
      out_frame[j++] = RESP_CODE_BEEBO;
      out_frame[j++] = BEEBO_RESP_NEIGHBOR;
      memcpy(&out_frame[j], nb->pubkey, PUB_KEY_SIZE); j += PUB_KEY_SIZE;
      out_frame[j++] = nb->pubkey_len;                      // full (32) vs partial prefix
      out_frame[j++] = (uint8_t)nb->snr;                    // x4, signed
      out_frame[j++] = nb->type;                            // ADV_TYPE_*
      memcpy(&out_frame[j], &nb->heard_timestamp, 4); j += 4;
      memcpy(&out_frame[j], &nb->advert_timestamp, 4); j += 4;
      memcpy(&out_frame[j], &nb->lat, 4); j += 4;
      memcpy(&out_frame[j], &nb->lon, 4); j += 4;
      StrHelper::strzcpy((char*)&out_frame[j], nb->name, 32); j += 32;
      _serial->writeFrame(out_frame, j);
    } else { // EOF
      out_frame[0] = RESP_CODE_BEEBO;
      out_frame[1] = BEEBO_RESP_END_OF_NEIGHBORS;
      _serial->writeFrame(out_frame, 2);
      _neighread.active = false;
    }
  } else if (_pathread.active && !_serial->isWriteBusy()) {
    // beebo: stream GET_ADVERT_PATHS one entry per loop, same pacing as
    // GET_NEIGHBORS/contacts above.
    AdvertPath* p = NULL;
    while (_pathread.index < ADVERT_PATH_TABLE_SIZE) {
      p = &advert_paths[_pathread.index++];
      if (p->recv_timestamp) break;
      p = NULL;
    }
    if (p != NULL) {
      int j = 0;
      out_frame[j++] = RESP_CODE_BEEBO;
      out_frame[j++] = BEEBO_RESP_ADVERT_PATH;
      memcpy(&out_frame[j], p->pubkey_prefix, sizeof(p->pubkey_prefix)); j += sizeof(p->pubkey_prefix);
      StrHelper::strzcpy((char*)&out_frame[j], p->name, sizeof(p->name)); j += sizeof(p->name);
      memcpy(&out_frame[j], &p->recv_timestamp, 4); j += 4;
      out_frame[j++] = p->path_len;
      j += mesh::Packet::writePath(&out_frame[j], p->path, p->path_len);
      _serial->writeFrame(out_frame, j);
    } else { // EOF
      out_frame[0] = RESP_CODE_BEEBO;
      out_frame[1] = BEEBO_RESP_END_OF_ADVERT_PATHS;
      _serial->writeFrame(out_frame, 2);
      _pathread.active = false;
    }
  //} else if (!_serial->isWriteBusy()) {
  //  checkConnections();    // TODO - deprecate the 'Connections' stuff
  }
}

uint16_t Beebo::updateBattTrend(bool force_read) {
  // beebo: two-stage deadline -- soft_due opens at batt_sample_period_secs,
  // starting a window (batt_sample_window_secs wide) where a sample is only
  // taken if radioIsIdle(), to keep the reading clear of RX/TX-induced IR
  // drop; hard_due is the fallback at period_secs + window_secs that fires
  // regardless, so a busy radio can't starve the trend classifier.
  bool hard_due = monring.allocated() && millisHasNowPassed(_next_batt_deadline);
  bool soft_due = !hard_due && monring.allocated() && millisHasNowPassed(_next_batt_trigger);
  if (!hard_due && !soft_due && !force_read) return _cached_batt_mv;

  if (soft_due && !force_read && !radioIsIdle()) {
    return _cached_batt_mv;  // still inside the window -- retry next loop() tick
  }

  bool due = hard_due || soft_due;
  bool idle_now = radioIsIdle();
  uint16_t new_batt_mv = board.getBattMilliVolts();
  if (due) {
    uint32_t period_secs = _board.batt_sample_period_secs > 0
                            ? _board.batt_sample_period_secs : BATT_SAMPLE_PERIOD_DEFAULT_SECS;
    uint32_t window_secs = _board.batt_sample_window_secs > 0
                            ? _board.batt_sample_window_secs : BATT_SAMPLE_WINDOW_DEFAULT_SECS;
    _next_batt_trigger = futureMillis(period_secs * 1000UL);
    _next_batt_deadline = futureMillis((period_secs + window_secs) * 1000UL);
    // beebo: only feed idle (IR-drop-clear) readings into the trend anchor/
    // classifier -- a hard_due sample can land while the radio is busy, and
    // comparing that against a clean idle sample would misread IR-drop sag
    // recovery as a real charge/discharge edge. Non-idle hard_due samples
    // are still charted below, just not classified.
    if (idle_now) {
      if (_cached_batt_mv == 0 && !_batt_boot_settled) {
        // beebo: this is the first VBAT sample after boot -- it lands right
        // after boot's heavy load, before the battery has settled, so it's
        // charted below like any other sample but skipped here so it can't
        // become the trend anchor.
        _batt_boot_settled = true;
      } else {
        if (_cached_batt_mv == 0) {
          _cached_batt_mv = new_batt_mv;  // first-ever sample (post-settle boot, or post-reset) -- seed the anchor
        }
        _batt_state = classifyBattTrend(new_batt_mv, _cached_batt_mv, _batt_state,
                                         _board.batt_present, _board.adc_resolution_bits,
                                         BATT_FULL_MV);
      }
    }
  }

  // beebo: chart every live read taken here -- idle vs busy (see radioIsIdle())
  // -- so the Vbat trace can be studied against IR-drop/USB-plug events.
  BattRecord br{};
  br.batt_mv = new_batt_mv;
  if (idle_now) br.flags |= BATTREC_FLAG_IDLE;
  br.flags |= (_batt_state << BATTREC_STATE_SHIFT) & BATTREC_STATE_MASK;
  if (_serial->is24GUp()) br.flags |= BATTREC_FLAG_XPORT_24G;
  if (_serial->isUsbUp()) br.flags |= BATTREC_FLAG_XPORT_USB;
  monring.appendBatt(br, (uint32_t)getRTCClock()->getCurrentTime());

  return new_batt_mv;
}

void Beebo::loop() {
  // beebo: a host that Ctrl-C's or crashes mid-transfer never sends
  // OTA_END, so ota_partition (and therefore isOTAActive()/skip_radio/
  // ota_priority) would otherwise stay latched forever with no way back
  // short of a reboot -- abort the flash write and drop the session if it's
  // gone quiet this long. 30s is generous next to a single chunk's normal
  // round-trip (tens to low hundreds of ms even on a slow link).
  static const uint32_t OTA_IDLE_TIMEOUT_MS = 30000;
  if (isOTAActive() && millisHasNowPassed(_ota_last_activity + OTA_IDLE_TIMEOUT_MS)) {
    esp_ota_abort(ota_handle);
    ota_handle = 0;
    ota_partition = NULL;
    ota_priority = false;
  }

  // beebo: CMD_OTA_BEGIN's priority flag asks us to skip a radio dispatch
  // pass entirely while the transfer is active, so checkSerialInterface()
  // can drain OTA chunks back-to-back instead of sharing each loop()
  // iteration with mesh/radio work. Honoured on every transport -- BLE/USB
  // OTA benefits from a clear radio gap just as much as TCP does (a live
  // mesh's RX/routing work was found to throttle BLE/USB OTA throughput by
  // several times, not just add a fixed gap), and OTA is always a short,
  // bounded window, so the radio isn't starved for long regardless of which
  // transport is carrying it.
  // beebo: bench IR-drop test in progress -- radio + FEM are asleep, skip all
  // mesh/radio work so nothing pokes the radio back awake. checkSerialInterface()
  // below still runs, so CMD_SET_QUIET(0) and status queries stay responsive.
  bool skip_radio = (isOTAActive() && ota_priority) || _bench_quiet;

  // beebo: loop() split by role
  // (BeeboCompanion.cpp/BeeboRepeater.cpp), same rationale as begin()'s own
  // dispatch above -- see Beebo.h's declarations. Each owns its own radio-loop
  // call (BaseChatMesh::loop() for companion, Mesh::loop() directly for
  // repeater -- see loopRepeater()'s comment on why they differ), gated by
  // skip_radio, and calls checkSerialInterface() itself right after -- same
  // radio-loop-then-checkSerialInterface order this always had, preserved
  // even though repeater now also has per-tick advert-timer content that
  // must run after checkSerialInterface(), not before it (see loopRepeater()).
  // beebo: isCompanion()/isRepeater() are mutually exclusive (see
  // requestNodeRoleSwitch()), so exactly one of these runs; the `|| isCompanion()`
  // in loopRepeater()'s skip_radio arg is now always false in practice but
  // kept as a defensive no-op. Kept as two plain ifs (not else-if) for the
  // same line-coverage reason as begin()'s dispatch above.
#if BEEBO_ENABLE_COMPANION_ROLE
  if (isCompanion()) loopCompanion(skip_radio);
#endif
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) loopRepeater(skip_radio || isCompanion());
#endif

  // beebo: flood-echo side of the
  // pass/fail fix, role-agnostic (both companion self-originated sends and
  // repeater forwards populate the same echo ring) -- see
  // SimpleMeshTables::checkEchoTimeouts()'s own comment. Skipped along
  // with the rest of the radio-facing work under skip_radio, same as the
  // ack-table sweep above.
  if (!skip_radio) {
    ((SimpleMeshTables*)getTables())->checkEchoTimeouts();
  }

  // beebo: go through board.reboot()/rebootWithTime() rather than a bare
  // esp_restart() so the RTC time gets persisted
  // (beebo_persistRTCTimeForReboot()) before restart -- otherwise
  // ESP32RTCClock::begin() falls back to whatever was last saved at an
  // earlier reboot, which can be stale and make the clock jump backward
  // after every OTA update. _ota_restart_ts (BEEBO_CMD_OTA_END's optional
  // trailing host timestamp) takes priority when present -- see its own
  // comment -- so an OTA update on a device with a wrong live clock
  // persists the *host's* time instead of re-persisting the device's own.
  if (_ota_restart_time && millisHasNowPassed(_ota_restart_time)) {
    if (_ota_restart_ts) board.rebootWithTime(_ota_restart_ts);
    else board.reboot();
  }

  // beebo: refresh the cached battery reading — the ADC read blocks for
  // ~10-12ms (see _cached_batt_mv), and voltage moves slowly enough that a
  // 5-minute cadence loses nothing. Radio config and the rest of the env
  // sample are pushed fresh at every RX/TX capture instead (monring.noteRadio/
  // monring.sampleEnv in logRxRaw/logTx/logTxFail), so a config or noise/heap
  // change is always attributed to the right packet rather than lagging a poll.
  updateBattTrend();

  // beebo: refresh the slow-stat caches off the hot path — getStorageUsedKb() is
  // a live FS block-scan and getMCUTemperature() averages four ~76 ms sensor
  // reads, so the status commands return the cache instead of blocking on them.
  // Slow cadence: FS usage changes only on file writes and MCU temp drifts slowly.
  if (millisHasNowPassed(_next_slowstat_refresh)) {
    _next_slowstat_refresh = futureMillis(SLOWSTAT_REFRESH_MS);
    _fs_used_kb = _store->getStorageUsedKb();
    // One sample smoothed 3:1 into the cache instead of the board's 4-read
    // average: same smoothing over successive ticks, a quarter of the stall in
    // the mesh loop (each read blocks ~76 ms; the SX1262 buffers only one
    // packet, so a long stall risks dropping back-to-back receptions).
    int16_t sample = (int16_t)(temperatureRead() * 10);
    _mcu_temp_scaled = (int16_t)((3 * (int32_t)_mcu_temp_scaled + sample) / 4);
  }

  // beebo: cheap every-tick check (a couple of integer compares) -- not
  // role-gated, since the node's BLE/WiFi link congestion applies regardless
  // of role (companion or repeater). Faults and mesh-side
  // queue-full drops are immediate hooks now
  // (logFaultEvent()/logTxQueueFull()/logRxQueueFull()), not polled here.
  appendLinkQueueDropEvents();

#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: Phase A dynamic-tuning optimizer tick (see TuneController.h /
  // Repeater role only (these knobs only affect
  // live behaviour for the repeater path -- see getAirtimeBudgetFactor()/
  // calcRxDelay()/getRetransmitDelay()/getDirectRetransmitDelay()/
  // getAGCResetInterval() above), off by default until explicitly enabled
  // ("set tune.enabled on"). Per-param live actuation (_tune_applied_mask,
  // default 0) is a separate, narrower opt-in on top of that -- every param
  // stays observe-only until its own bit is set.
  if (isRepeater() && _tune_enabled && monring.allocated() &&
      millisHasNowPassed(_next_tune_tick)) {
    _next_tune_tick = futureMillis(TUNE_TICK_INTERVAL_MS);
    int16_t current_values[TuneController::NUM_PARAMS] = {
      (int16_t)(_role_state->prefs.rx_delay_base * 100.0f + 0.5f),
      (int16_t)(_role_state->prefs.tx_delay_factor * 100.0f + 0.5f),
      (int16_t)(_role_state->prefs.direct_tx_delay_factor * 100.0f + 0.5f),
      (int16_t)_role_state->prefs.agc_reset_interval,
      (int16_t)_role_state->prefs.interference_threshold,
      (int16_t)(_role_state->prefs.airtime_factor * 100.0f + 0.5f),
    };
    TuneController::TxConfirmStats tx_stats;
    tx_stats.ack_success_count = getAckSuccessCount();
    tx_stats.ack_timeout_count = getAckTimeoutCount();
    tx_stats.echo_attempt_count = ((SimpleMeshTables*)getTables())->getEchoAttemptCount();
    tx_stats.echo_success_count = ((SimpleMeshTables*)getTables())->getEchoSuccessCount();
    TuneController::Decision decision = tune_controller.tick(
      monring, (uint32_t)getRTCClock()->getCurrentTime(), current_values, tx_stats, _tune_applied_mask);
    if (decision.should_apply) {
      applyTuneDecision(decision.param_id, decision.value);
    }
  }
#endif

  // is there are pending dirty contacts/ACL write needed?
  if (dirty_contacts_expiry && millisHasNowPassed(dirty_contacts_expiry)) {
#if BEEBO_ENABLE_REPEATER_ROLE
    if (isRepeater()) acl.save(_store->getPrimaryFS());
    else
#endif
    if (isCompanion()) saveContacts();
    dirty_contacts_expiry = 0;
  }

  loopTransports();

#ifdef P_LORA_TX_LED
  updateStatusLed();
#endif
}

#ifdef P_LORA_TX_LED
#ifndef LED_BRIGHTNESS
#define LED_BRIGHTNESS 32 // 0=off .. 255=full (TX / RX / OTA peak)
#endif
#define LED_HEARTBEAT_PERIOD 5000 // heartbeat interval (ms)
#define LED_HEARTBEAT_ON_MS  40   // heartbeat pulse width (ms)
#define LED_HEARTBEAT_LEVEL  4    // heartbeat brightness (low)
#define LED_XPORT_LEVEL      2    // transport-activity brightness (very low)
#define LED_XPORT_FLASH_MS   40   // transport-activity flash hold (ms)
#define LED_TX_FLASH_MS      1000 // radio-TX blink: single ~1 s blink (high)
#define LED_RX_FLASH_MS      60   // radio-RX blink: short (high)
#define LED_OTA_FLASH_MS     60   // OTA rapid-flash half-period (ms)

void Beebo::updateStatusLed() {
  unsigned long now = millis();

  // Latch a long HIGH blink when the radio transmits. The sent counters bump
  // only after the radio reports the send finished (cheap inline getters).
  uint32_t sent_now = getNumSentFlood() + getNumSentDirect();
  if (sent_now != led_last_sent) {
    led_last_sent = sent_now;
    led_tx_flash_until = now + LED_TX_FLASH_MS;
  }

  // Latch a short HIGH blink when the radio receives a packet.
  uint32_t recv_now = getNumRecvFlood() + getNumRecvDirect();
  if (recv_now != led_last_recv) {
    led_last_recv = recv_now;
    led_rx_flash_until = now + LED_RX_FLASH_MS;
  }

  // Latch a LOW flash on companion-link activity (TX+RX frames). The counter
  // is a plain member increment in MultiSerialInterface — no added loop cost.
  uint32_t act_now = serial_interface.activityCount();
  if (act_now != led_last_activity) {
    led_last_activity = act_now;
    led_xport_flash_until = now + LED_XPORT_FLASH_MS;
  }

  // Choose the target level by priority; write the peripheral only on change.
  int level;
  if (isOTAActive()) {
    level = ((now / LED_OTA_FLASH_MS) & 1) ? LED_BRIGHTNESS : 0; // rapid flash
  } else if (now < led_tx_flash_until) {
    level = LED_BRIGHTNESS; // radio TX: long high blink
  } else if (now < led_rx_flash_until) {
    level = LED_BRIGHTNESS; // radio RX: short high blink
  } else {
    unsigned long t = now % LED_HEARTBEAT_PERIOD; // baseline heartbeat
    level = (t < LED_HEARTBEAT_ON_MS) ? LED_HEARTBEAT_LEVEL : 0;
    if (now < led_xport_flash_until && level < LED_XPORT_LEVEL)
      level = LED_XPORT_LEVEL; // transport activity: low intensity
  }

  if (level != led_level) {
    analogWrite(P_LORA_TX_LED, level);
    led_level = level;
  }
}
#endif // P_LORA_TX_LED

// beebo: per-tick transport management -- STA event draining, WiFi
// reconnect, live provisioning/toggle (CMD_SET_WIFI_CREDS/
// CMD_SET_TRANSPORT_CONFIG), and deferred teardown timers. Moved in from
// main.cpp's loop() (ran immediately after beebo.loop() returned there,
// same relative order preserved by running last in loop() here).
void Beebo::loopTransports() {
  // Re-check every tracked transport variable and log a line for each one
  // that changed since last tick -- see TLOG_XPORT_VAR_CHANGED's own
  // comment in TransportLog.h. This one call covers session start/end too
  // (via the "active" variable going 0 <-> nonzero), so no separate
  // start/end hook is needed here.
  _checkTransportStateChanges();

  // beebo: periodic low-level WiFi health sample -- see TLOG_WIFI_HEALTH's
  // own comment in TransportLog.h. Gated on _wifi_up (not tcp_enabled --
  // this is about the radio actually being live right now, same signal
  // applyTransportConfig() itself uses) so it stays silent while WiFi is
  // off, and unconditional on any session being live (RSSI/heap sampling
  // never touches deviceConnected/the listening socket).
  if (_wifi_up && millis() - _last_wifi_health_sample_ms >= WIFI_HEALTH_SAMPLE_MS) {
    _last_wifi_health_sample_ms = millis();
    uint16_t heap_kb = (uint16_t)(ESP.getFreeHeap() / 1024);
    int8_t rssi = (int8_t)WiFi.RSSI();
    uint8_t channel = (uint8_t)WiFi.channel();
    int32_t detail = (int32_t)heap_kb | ((int32_t)(uint8_t)rssi << 16) | ((int32_t)channel << 24);
    transport_log.log(TLOG_WIFI_HEALTH, detail);
  }

  // Drain WiFi STA events stashed by the (other-task) event handler into the
  // debug ring here, so all ring writes stay in the single loop() context.
  if (_sta_disc_reason >= 0) {
    // No (int8_t) truncation here -- esp_wifi disconnect reason codes go up
    // to ~208 (e.g. BEACON_TIMEOUT=200, NO_AP_FOUND=201), which silently
    // wrapped negative through an int8_t before `detail` was widened to
    // int32_t for the IP-logging change above.
    transport_log.log(TLOG_WIFI_STA_DISCONNECTED, _sta_disc_reason);
    _sta_disc_reason = -1;
  }
  if (_sta_got_ip) {
    IPAddress ip = WiFi.localIP();
    int32_t packed_ip = ((int32_t)ip[0] << 24) | ((int32_t)ip[1] << 16)
                       | ((int32_t)ip[2] << 8) | (int32_t)ip[3];
    transport_log.log(TLOG_WIFI_STA_GOT_IP, packed_ip);
    _sta_got_ip = false;
  }
  if (_wifi_needs_rebind) {
    _wifi_needs_rebind = false;
    wifi_interface.rebind();
  }

  // Safely attempt to reconnect every 10 seconds if flagged.
  if (_wifi_needs_reconnect && (millis() - _last_wifi_reconnect_attempt > 10000)) {
    WIFI_DEBUG_PRINTLN("Attempting manual WiFi reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    _last_wifi_reconnect_attempt = millis();
  }

  // Live WiFi provisioning: start (or re-join) WiFi without rebooting after
  // CMD_SET_WIFI_CREDS. (Define WIFI_CREDS_REBOOT at build time to reboot
  // instead.) The flag is consumed unconditionally -- it used to be read
  // behind `!_wifi_started && _role_state->prefs.tcp_enabled`, so a creds change that
  // didn't match those conditions left it stuck true forever.
  if (consumeWifiCredsPending()) {
    if (!_role_state->prefs.tcp_enabled) {
      // Nothing to do now: enabling TCP later brings WiFi up on these creds
      // via the transport-config block below.
    } else if (!_wifi_started) {
      _wifi_started = true;
      board.setInhibitSleep(true);
      WiFi.setAutoReconnect(true);
      WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info){ _onWifiStaEvent(event, info); });
      WiFi.begin(_role_state->prefs.wifi_ssid, _role_state->prefs.wifi_pwd);
      WiFi.setSleep(false);   // ble/tcp mutually exclusive → no BT coexistence → power-save off (reliable)
      wifi_interface.begin(TCP_PORT);
      serial_interface.addInterface(&wifi_interface, nullptr, nullptr, true, TLOG_XPORT_TCP);
      wifi_interface.enable();   // serial_interface already enabled; enable the new sub manually
      _wifi_up = true;
    } else if (_wifi_up) {
      // Station already running on the OLD credentials -- nothing re-ran
      // WiFi.begin(), so this used to need a reboot (or a tcp off/on toggle)
      // to take effect. Re-joining with the new ones drops the WiFi
      // association, which would tear down a TCP session still on the old
      // network before its own OK reply to this very command is guaranteed
      // delivered (the reply is queued before this runs, see
      // BEEBO_CMD_SET_WIFI_CREDS's handler, but queued isn't delivered --
      // the socket can still be reset out from under buffered bytes).
      // Deferred instead of applied immediately: see the isConnected() check
      // below, consumed once nothing is still connected over that session.
      WIFI_DEBUG_PRINTLN("New WiFi credentials queued -- rejoining once the "
                          "current session ends...");
      _wifi_creds_reconnect_pending = true;
    }
    // else: _wifi_started but torn down (tcp off) -- the re-enable path
    // calls WiFi.begin() with whatever is in _role_state->prefs, so these creds are
    // already what it will use.
  }

  // beebo: the deferred half of the branch above -- runs every tick once
  // armed, until the caller that requested new credentials (or anyone else
  // still on the old network) has actually disconnected, so the reconnect
  // never races an in-flight reply.
  if (_wifi_creds_reconnect_pending && !wifi_interface.isConnected()) {
    _wifi_creds_reconnect_pending = false;
    WIFI_DEBUG_PRINTLN("Re-joining WiFi with new credentials...");
    _wifi_needs_reconnect = false;   // the reconnect timer must not race this
    _sta_got_ip = false;
    WiFi.disconnect();
    WiFi.begin(_role_state->prefs.wifi_ssid, _role_state->prefs.wifi_pwd);
    WiFi.setSleep(false);
  }

  // Live BLE/TCP/USB toggle: apply a CMD_SET_TRANSPORT_CONFIG change without
  // rebooting. The whole switch (bring-up and tear-down together) is
  // deferred atomically until no app session is live -- see
  // _transport_switch_pending's declaration for why bring-up can't jump
  // ahead of tear-down here the way it used to. Applied right away if no
  // session is live at request time; otherwise the check below picks it up
  // once the live session ends.
  if (consumeTransportConfigPending()) {
    if (_serial->isConnected()) {
      _transport_switch_pending = true;
    } else {
      applyTransportConfig();
    }
  }
  if (_transport_switch_pending && !_serial->isConnected()) {
    _transport_switch_pending = false;
    applyTransportConfig();
  }
}

// Brings ble/tcp/usb interfaces up or down to match whatever is currently
// persisted in _role_state->prefs -- the single atomic apply step for a live
// transport switch (see _transport_switch_pending's declaration for why
// bring-up and tear-down must happen together, only once no app session is
// live). Also used by loopTransports() for the immediate-apply path when a
// switch is requested with no session live to wait for.
void Beebo::applyTransportConfig() {
  bool ble_on = _role_state->prefs.ble_enabled != 0;
  bool tcp_on = _role_state->prefs.tcp_enabled != 0 && _role_state->prefs.wifi_ssid[0] != '\0';
  bool usb_on = _role_state->prefs.usb_enabled != 0;

  bool ble_torn_down = false;
  if (ble_on && !_ble_up) {
    if (!_ble_added) {
      ble_interface.begin(BLE_NAME_PREFIX, _role_state->prefs.node_name, getBLEPin());
      serial_interface.addInterface(&ble_interface, nullptr, nullptr, true, TLOG_XPORT_BLE);
      _ble_added = true;
    } else {
      ble_interface.initRadio();   // rebuild the stack torn down by a prior deinitRadio()
    }
    ble_interface.enable();
    _ble_up = true;
  } else if (!ble_on && _ble_up) {
    ble_interface.disable();
    ble_interface.deinitRadio();   // fully power down the BLE radio, not just stop advertising
    _ble_up = false;
    ble_torn_down = true;
    // beebo: nudge the coexistence arbiter back toward WiFi now that BT is
    // fully torn down. Starting the BT controller (initRadio() above) can
    // leave esp_wifi/esp_bt coexistence arbitration favoring BT even after
    // BT is later deinitialized -- invisible to WiFi.status()/RSSI/heap
    // (all stay normal), but it silently starves WiFi's TCP data plane.
    // Found via hardware repro: a plain tcp-off/tcp-on cycle is always
    // clean, but any cycle that brought BLE's radio up first reliably
    // broke the *next* WiFi bring-up a few seconds later (BUGS.md
    // 2026-08-31). TLOG_COEX_PREFER_WIFI existed as a prepared logging
    // slot for exactly this call with no call site until now.
    esp_err_t coex_err = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    transport_log.log(TLOG_COEX_PREFER_WIFI, coex_err);
    DEBUG_LOG("ble torn down, coex_err=%d, heap=%u", (int)coex_err, (unsigned)ESP.getFreeHeap());
  }

  if (tcp_on && !_wifi_up) {
    if (ble_torn_down) {
      // beebo: BLEDevice::deinit() returns before the BT controller has
      // fully released the shared RF/PHY path -- widely reported in the
      // ESP32-Arduino community as WiFi failing (or, per our own hardware
      // repro, silently going TCP-deaf a few seconds later while every
      // driver-level status stays healthy) when it comes up immediately
      // after a live BLE deinit, with a short settle delay as the
      // reported mitigation. `esp_coex_preference_set(ESP_COEX_PREFER_
      // WIFI)` below did not fix this (confirmed via hardware repro,
      // BUGS.md 2026-08-31) -- this delay is the next, better-evidenced
      // attempt. Blocking is deliberate and scoped to only this one rare,
      // user-triggered transition (never a plain tcp-only cycle, which
      // never touches BLE) -- not a per-tick cost.
      delay(200);
      esp_bt_controller_status_t bt_status = esp_bt_controller_get_status();
      transport_log.log(TLOG_BT_CONTROLLER_STATUS, (int32_t)bt_status);
      DEBUG_LOG("wifi bring-up after ble teardown, bt_status=%d, heap=%u", (int)bt_status, (unsigned)ESP.getFreeHeap());
    }
    board.setInhibitSleep(true);   // prevent sleep when WiFi is active
    WiFi.setAutoReconnect(true);
    if (!_wifi_started) {
      // WiFi.onEvent() has no matching "off" and keeps appending a handler
      // to an internal list on every call -- register it only the first
      // time WiFi is ever brought up, not on every live re-enable.
      WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info){ _onWifiStaEvent(event, info); });
    }
    WiFi.begin(_role_state->prefs.wifi_ssid, _role_state->prefs.wifi_pwd);
    // BLE and TCP are mutually exclusive, so no BT + WiFi coexistence -- power
    // save can always be off so the station stays awake (eliminates the
    // multi-second TCP latency/loss).
    WiFi.setSleep(false);
    if (!_wifi_started) {
      wifi_interface.begin(TCP_PORT);
      serial_interface.addInterface(&wifi_interface, nullptr, nullptr, true, TLOG_XPORT_TCP);
      _wifi_started = true;
    } else {
      wifi_interface.enable();
    }
    _wifi_up = true;
  } else if (!tcp_on && _wifi_up) {
    wifi_interface.disable();
    _wifi_needs_reconnect = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);   // fully power down the WiFi radio
    board.setInhibitSleep(false);
    _wifi_up = false;
  }

  if (usb_on && !_usb_up) {
    if (!_usb_added) {
      usb_interface.begin(Serial);
      serial_interface.addInterface(&usb_interface, []() -> bool { return (bool)Serial; },
                                    nullptr, false, TLOG_XPORT_USB);
      _usb_added = true;
    }
    usb_interface.enable();
    _usb_up = true;
  } else if (!usb_on && _usb_up) {
    usb_interface.disable();   // no radio to power down, unlike BLE/WiFi
    _usb_up = false;
  }
}

mesh::Packet* Beebo::createSelfAdvertPacket() {
#if BEEBO_ENABLE_REPEATER_ROLE
  bool use_repeater_type = (_board.role == NODE_ROLE_REPEATER);
  const char* name = use_repeater_type ? getRoleName(NODE_ROLE_REPEATER) : _role_state->prefs.node_name;
  // beebo: role-aware, same rationale as the self_info reply above -- each
  // role's own advert_loc_policy/coords, not the shared companion _role_state->prefs
  // ones bleeding into a live repeater's own advert. A real 3-way branch
  // now, matching upstream simple_repeater's CommonCLI::buildAdvertData()
  // (NONE/SHARE/PREFS are genuinely different sources there, unlike
  // companion_radio's own advert() which only ever checked NONE): SHARE
  // reads the single shared/live sensors.node_lat/lon register (no GPS
  // compiled into beebo -- ENV_INCLUDE_GPS=0 -- so this stays exactly
  // 0,0 until a real GPS driver exists; deliberately NOT seeded from
  // either role's own PREFS value, so a bare "share" without GPS is
  // obviously wrong rather than silently acting like "prefs"), PREFS
  // reads the live role's own persisted coordinate (_role_state->prefs.node_lat/lon
  // for repeater, _role_state->prefs.node_lat/lon for companion -- the same field
  // CMD_SET_ADVERT_LATLON/SET_LAT/SET_LON now write).
  uint8_t loc_policy = use_repeater_type ? _role_state->prefs.advert_loc_policy : _role_state->prefs.advert_loc_policy;
  if (loc_policy == ADVERT_LOC_NONE) {
    return use_repeater_type ? createRepeaterSelfAdvert(name)
                              : createSelfAdvert(name);
  } else if (loc_policy == ADVERT_LOC_SHARE) {
    return use_repeater_type ? createRepeaterSelfAdvert(name, sensors.node_lat, sensors.node_lon)
                              : createSelfAdvert(name, sensors.node_lat, sensors.node_lon);
  }
  double lat = use_repeater_type ? _role_state->prefs.node_lat : _role_state->prefs.node_lat;
  double lon = use_repeater_type ? _role_state->prefs.node_lon : _role_state->prefs.node_lon;
  return use_repeater_type ? createRepeaterSelfAdvert(name, lat, lon)
                            : createSelfAdvert(name, lat, lon);
#else
  // beebo: STATIC_ROLE_BUILDS -- no repeater-type advert without repeater
  // support; _board.role can never be NODE_ROLE_REPEATER here (see
  // isNodeRoleBuiltIn()). Same NONE/SHARE/PREFS 3-way as above, companion-only.
  const char* name = _role_state->prefs.node_name;
  if (_role_state->prefs.advert_loc_policy == ADVERT_LOC_NONE) {
    return createSelfAdvert(name);
  } else if (_role_state->prefs.advert_loc_policy == ADVERT_LOC_SHARE) {
    return createSelfAdvert(name, sensors.node_lat, sensors.node_lon);
  }
  return createSelfAdvert(name, _role_state->prefs.node_lat, _role_state->prefs.node_lon);
#endif
}

bool Beebo::advert() {
  mesh::Packet* pkt = createSelfAdvertPacket();
  if (pkt) {
    sendZeroHop(pkt);
    return true;
  } else {
    return false;
  }
}

// beebo: requesting side of
// the "node discover" protocol. Still
// sends the optional "since" field (zeroed) for wire compatibility with a
// real simple_repeater answering it, even though this node's own responder
// (onControlDataRecv) doesn't parse it back out -- see that function's
// comment. `filter` is a bitmask of (1 << ADV_TYPE_*); `prefix_only` asks
// responders to reply with just an 8-byte pubkey prefix instead of the
// full 32 bytes.
void Beebo::sendNodeDiscoverReq(uint8_t filter, bool prefix_only) {
  uint8_t data[10];
  data[0] = CTL_TYPE_NODE_DISCOVER_REQ | (prefix_only ? 1 : 0);
  data[1] = filter;
  getRNG()->random(&data[2], 4); // tag
  memcpy(&pending_discover_tag, &data[2], 4);
  pending_discover_until = futureMillis(60000);
  uint32_t since = 0;
  memcpy(&data[6], &since, 4);

  auto pkt = createControlData(data, sizeof(data));
  if (pkt) {
    sendZeroHop(pkt);
  }
}

// To check if there is pending work
bool Beebo::hasPendingWork() const {
  return _mgr->getOutboundTotal() > 0 || dirty_contacts_expiry != 0;
}

void Beebo::sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size) {
  // beebo: only ever called from repeater-role admin-request/ACL code
  // (BeeboRepeater.cpp, Beebo.cpp's onPeerDataRecv() inside its
  // `if (isRepeater())` block) -- restored to repeater's own RegionMap
  // default region (getDefaultScope(NODE_ROLE_REPEATER, ...)), matching
  // stock simple_repeater's own default_scope more closely than the
  // earlier borrow-companion's-field stand-in did. Unlike stock's
  // sendFloodReply, still doesn't preserve the *request* packet's
  // incoming RF-region scope via recv_pkt_region -- a tracking mechanism
  // this port doesn't add. path_hash_size is unused here: the
  // TransportKey overload of sendFloodScoped always sizes hashes from the
  // live role's own path_hash_mode (_role_state->prefs's for repeater,
  // _role_state->prefs's for companion -- see its own definition).
  TransportKey default_scope;
  getDefaultScope(NODE_ROLE_REPEATER, default_scope);
  auto scope = send_scope.isNull() ? &default_scope : &send_scope;
  sendFloodScoped(*scope, packet, delay_millis);
}

int Beebo::searchPeersByHash(const uint8_t *hash) {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    int n = 0;
    for (int i = 0; i < acl.getNumClients(); i++) {
      if (acl.getClientByIdx(i)->id.isHashMatch(hash)) {
        acl_peer_indexes[n++] = i;
      }
    }
    return n;
  }
#endif
  return BaseChatMesh::searchPeersByHash(hash);
}

void Beebo::getPeerSharedSecret(uint8_t *dest_secret, int peer_idx) {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    int i = acl_peer_indexes[peer_idx];
    if (i >= 0 && i < acl.getNumClients()) {
      memcpy(dest_secret, acl.getClientByIdx(i)->shared_secret, PUB_KEY_SIZE);
    } else {
      MESH_DEBUG_PRINTLN("getPeerSharedSecret: Invalid peer idx: %d", i);
    }
    return;
  }
#endif
  BaseChatMesh::getPeerSharedSecret(dest_secret, peer_idx);
}

// beebo: never overridden
// before, so repeater-role peers fell through to BaseChatMesh's own
// onPeerPathRecv, which indexes matching_peer_indexes[sender_idx] into the
// ContactInfo table. searchPeersByHash's REPEATER branch (above) never
// populates matching_peer_indexes at all -- it fills acl_peer_indexes
// instead -- so that fallback either silently dropped the path update or,
// worse, misattributed it to a stale/unrelated companion contact. Ported
// from simple_repeater's own onPeerPathRecv (MyMesh.cpp:1019-1037): stores
// the returned path onto the matching ClientACL entry.
bool Beebo::onPeerPathRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret, uint8_t *path,
                            uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    int i = acl_peer_indexes[sender_idx];
    if (i >= 0 && i < acl.getNumClients()) {
      auto client = acl.getClientByIdx(i);
      client->out_path_len = mesh::Packet::copyPath(client->out_path, path, path_len);
      client->last_activity = getRTCClock()->getCurrentTime();
    } else {
      MESH_DEBUG_PRINTLN("onPeerPathRecv: invalid peer idx: %d", i);
    }
    // NOTE: no reciprocal path send, matching simple_repeater.
    return false;
  }
#endif
  return BaseChatMesh::onPeerPathRecv(packet, sender_idx, secret, path, path_len, extra_type, extra, extra_len);
}

void Beebo::onPeerDataRecv(mesh::Packet *packet, uint8_t type, int sender_idx, const uint8_t *secret,
                            uint8_t *data, size_t len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (isRepeater()) {
    int i = acl_peer_indexes[sender_idx];
    if (i < 0 || i >= acl.getNumClients()) {
      MESH_DEBUG_PRINTLN("onPeerDataRecv: invalid peer idx: %d", i);
      return;
    }
    ClientInfo* client = acl.getClientByIdx(i);

    if (type == PAYLOAD_TYPE_REQ) { // request (from a known admin/guest client)
      uint32_t timestamp;
      memcpy(&timestamp, data, 4);

      if (timestamp > client->last_timestamp) { // prevent replay attacks
        int reply_len = handleRequest(client, timestamp, &data[4], len - 4);
        if (reply_len == 0) return; // invalid command

        client->last_timestamp = timestamp;
        client->last_activity = getRTCClock()->getCurrentTime();

        if (packet->isRouteFlood()) {
          mesh::Packet *path = createPathReturn(client->id, secret, packet->path, packet->path_len,
                                                PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
          if (path) sendFloodReply(path, ADMIN_REQ_SERVER_RESPONSE_DELAY, packet->getPathHashSize());
        } else {
          mesh::Packet *reply =
              createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, secret, reply_data, reply_len);
          if (reply) {
            if (client->out_path_len != OUT_PATH_UNKNOWN) {
              sendDirect(reply, client->out_path, client->out_path_len, ADMIN_REQ_SERVER_RESPONSE_DELAY);
            } else {
              sendFloodReply(reply, ADMIN_REQ_SERVER_RESPONSE_DELAY, packet->getPathHashSize());
            }
          }
        }
      } else {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
      }
    } else if (type == PAYLOAD_TYPE_TXT_MSG && len > 5 && client->isAdmin()) {
      // beebo: mesh-transport text command surface.
      // One more call site into handleCommand, same as
      // simple_repeater already runs it from three places (local serial,
      // USB/companion, and here) -- not a second dispatcher. Ported
      // verbatim from examples/simple_repeater/MyMesh.cpp's own
      // onPeerDataRecv. Gated on client->isAdmin(): guests never reach
      // handleCommand over mesh at all, matching upstream's own permission
      // model (no new allowlist mechanism needed on top of what
      // handleCommand itself already exposes -- see its own declaration
      // comment in Beebo.h for why that set stays deliberately small).
      uint32_t sender_timestamp;
      memcpy(&sender_timestamp, data, 4);
      uint8_t flags = (data[4] >> 2);

      if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA)) {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: unsupported text type received: flags=%02x", (uint32_t)flags);
      } else if (sender_timestamp >= client->last_timestamp) { // prevent replay attacks
        bool is_retry = (sender_timestamp == client->last_timestamp);
        client->last_timestamp = sender_timestamp;
        client->last_activity = getRTCClock()->getCurrentTime();

        data[len] = 0; // need to make a C string again, with null terminator

        if (flags == TXT_TYPE_PLAIN) { // for legacy CLI, send Acks
          uint32_t ack_hash;
          mesh::Utils::sha256((uint8_t *)&ack_hash, 4, data, 5 + strlen((char *)&data[5]), client->id.pub_key,
                              PUB_KEY_SIZE);
          mesh::Packet *ack = createAck(ack_hash);
          if (ack) {
            if (client->out_path_len == OUT_PATH_UNKNOWN) {
              sendFloodReply(ack, TXT_ACK_DELAY, packet->getPathHashSize());
            } else {
              sendDirect(ack, client->out_path, client->out_path_len, TXT_ACK_DELAY);
            }
          }
        }

        uint8_t temp[166];
        char *command = (char *)&data[5];
        char *reply = (char *)&temp[5];
        if (is_retry) {
          *reply = 0;
        } else {
          handleCommand(sender_timestamp, command, reply);
        }
        int text_len = strlen(reply);
        if (text_len > 0) {
          uint32_t timestamp = getRTCClock()->getCurrentTimeUnique();
          if (timestamp == sender_timestamp) {
            timestamp++;  // WORKAROUND: the two timestamps need to be different, in the CLI view
          }
          memcpy(temp, &timestamp, 4);
          temp[4] = (TXT_TYPE_CLI_DATA << 2);

          mesh::Packet *txt_reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
          if (txt_reply) {
            if (client->out_path_len == OUT_PATH_UNKNOWN) {
              sendFloodReply(txt_reply, CLI_REPLY_DELAY_MILLIS, packet->getPathHashSize());
            } else {
              sendDirect(txt_reply, client->out_path, client->out_path_len, CLI_REPLY_DELAY_MILLIS);
            }
          }
        }
      } else {
        MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
      }
    }
    return;
  }
#endif
  BaseChatMesh::onPeerDataRecv(packet, type, sender_idx, secret, data, len);
}

/* ------------------------------------------------------------------------
 * beebo: text-CLI back-compat surface (MULTI_ROLE_FW.md).
 * Minimal viable set: ver, board, reboot, get/set for a small initial key
 * table. Not a port of CommonCLI.cpp's ~50-command table (see the
 * handleCommand declaration comment in Beebo.h for why) -- extend this the
 * same incremental way CMD_BEEBO grew, not all at once.
 * ------------------------------------------------------------------------ */

// beebo: matches CommonCLI.cpp's own isValidName() exactly, for identical
// meshcli-visible validation behaviour on "set name ...".
static bool isValidName(const char *n) {
  while (*n) {
    if (*n == '[' || *n == ']' || *n == '\\' || *n == ':' || *n == ',' || *n == '?' || *n == '*') return false;
    n++;
  }
  return true;
}

void Beebo::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
  while (*command == ' ') command++;  // skip leading spaces

#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: this text-CLI entry point is reachable via DualModeSerialInterface's
  // local USB console or a remote mesh admin session regardless of which role
  // is currently live (see the "USB-only" comment further down) -- unlike the
  // binary CMD_BEEBO handlers, it's not gated behind isRepeater(), so its
  // many get/set branches below that touch repeater's own resident slot
  // directly (guest.password, owner.info, loop.detect, flood.*, the
  // "password " command, ...) rely on the same guarantee loadRoleState()
  // gives every other repeater-targeting call site: repeater's slot has
  // already been loaded from /beebo_repeater at boot, unconditionally,
  // regardless of which role is live -- so there's no unloaded/default
  // state here to write into or get silently clobbered by a later load.
#endif

  // beebo: MON_COMMAND (text-CLI half -- see MonRing.h/
  // appendTextCommandRunEvent()'s comment). Same "mutating/one-shot action,
  // not a routine read" scope as the binary path's filter in
  // checkRecvFrame(): "set ..." commands (logged as just the key, after the
  // prefix, since that's the useful part) plus the handful of bare
  // one-shot actions -- "get ..."/other reads are deliberately excluded so
  // this stays infrequent.
  if (memcmp(command, "set ", 4) == 0) {
    appendTextCommandRunEvent(&command[4]);
  } else if (memcmp(command, "reboot", 6) == 0 || memcmp(command, "advert", 6) == 0) {
    appendTextCommandRunEvent(command);
  }

  if (memcmp(command, "reboot", 6) == 0) {
    board.reboot();  // doesn't return
  } else if (memcmp(command, "ver", 3) == 0) {
    sprintf(reply, "%s (Build: %s)", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);
  } else if (memcmp(command, "board", 5) == 0) {
    sprintf(reply, "%s", board.getManufacturerName());
  } else if (memcmp(command, "advert.zerohop", 14) == 0) {
    advert() ? strcpy(reply, "OK - zerohop advert sent") : strcpy(reply, "Error");
  } else if (memcmp(command, "advert", 6) == 0) {
    // beebo: bare "advert" must send a FLOOD advert, matching real
    // CommonCLI's own semantics (src/helpers/CommonCLI.cpp: bare "advert" is
    // a FLOOD advert; "advert.zerohop" -- checked above, since it's also a
    // prefix match of "advert" -- is the zero-hop-only variant). Not role-gated:
    // companion_radio's own binary CMD_SEND_SELF_ADVERT already supports
    // this exact same flood/zero-hop choice via an optional flag byte, so a
    // flood advert is an equally legitimate companion operation, not a
    // repeater-only concept.
    mesh::Packet* pkt = createSelfAdvertPacket();
    if (pkt) {
      TransportKey default_scope;
      getDefaultScope(this->_board.role, default_scope);
      sendFloodScoped(default_scope, pkt, 1500);  // longer delay, give CLI response time to be sent first (matches CommonCLI)
      strcpy(reply, "OK - Advert sent");
    } else {
      strcpy(reply, "Error");
    }
  } else if (memcmp(command, "clock.epoch", 11) == 0) {
    sprintf(reply, "%lu", (unsigned long)getRTCClock()->getCurrentTime());
  } else if (memcmp(command, "clock", 5) == 0) {
    uint32_t now = getRTCClock()->getCurrentTime();
    DateTime dt = DateTime(now);
    sprintf(reply, "%02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
  } else if (memcmp(command, "time ", 5) == 0) {
    uint32_t secs = (uint32_t)atol(&command[5]);
    uint32_t curr = getRTCClock()->getCurrentTime();
    if (secs > curr) {
      getRTCClock()->setCurrentTime(secs);
      transport_log.log(TLOG_CLOCK_SET, secs);
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      sprintf(reply, "OK - clock set: %02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else {
      strcpy(reply, "(ERR: clock cannot go backwards)");
    }
  } else if (memcmp(command, "get ", 4) == 0) {
    const char* key = &command[4];
    // beebo: back-compat get surface for
    // meshcli -r -s. Keys with a companion NodePrefs equivalent (shared,
    // role-agnostic) read/write that field directly, same as the rest of
    // this firmware; keys with none (loop.detect, flood.*, agc.reset.
    // interval, int.thresh, allow.read.only, advert.interval, txdelay,
    // direct.txdelay -- confirmed repeater-only by MULTI_ROLE_FW.md's
    // settings-tree classification pass) read directly out of /com_prefs
    // via readComPrefsField(), same mechanism as loadRepeaterAuthPrefs().
    // NOT yet covered here (separate, larger follow-ups): region, sensor
    // get/set/list, neighbors, log, gps (ENV_INCLUDE_GPS=0 on this board
    // anyway), bridge/power-mgmt (not applicable to this board at all).
    if (memcmp(key, "name", 4) == 0) {  // meshcli's own flat name, kept as-is for real back-compat
      sprintf(reply, "> %s", _role_state->prefs.node_name);
    } else if (memcmp(key, "repeater.name", 13) == 0) {
      sprintf(reply, "> %s", getRoleName(NODE_ROLE_REPEATER));
    } else if (memcmp(key, "node.role", 9) == 0) {
      sprintf(reply, "> %s", isRepeater() ? "repeater" : "companion");
    } else if (memcmp(key, "role", 4) == 0) {  // meshcli's own read-only key, same value as node.role
      sprintf(reply, "> %s", isRepeater() ? "repeater" : "companion");
    } else if (memcmp(key, "dutycycle", 9) == 0) {
      float af = _role_state->prefs.airtime_factor;
      float dc = 100.0f / (af + 1.0f);
      int dc_int = (int)dc;
      int dc_frac = (int)((dc - dc_int) * 10.0f + 0.5f);
      sprintf(reply, "> %d.%d%%", dc_int, dc_frac);
    } else if (memcmp(key, "af", 2) == 0) {
      // beebo: repeater keeps its own independent copy -- repeater's own
      // independent value, restoring simple_repeater's original single-role
      // meaning of this text-CLI key instead of aliasing the shared
      // companion value.
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.airtime_factor));
    } else if (memcmp(key, "multi.acks", 10) == 0) {
      // beebo: repeater keeps its own independent copy -- repeater's own
      // independent value, same pattern as "af"/"rxdelay" above.
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", (uint32_t)(_role_state->prefs.multi_acks));
#else
      sprintf(reply, "> %d", (uint32_t)_role_state->prefs.multi_acks);
#endif
    } else if (memcmp(key, "path.hash.mode", 14) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", (uint32_t)(_role_state->prefs.path_hash_mode));
#else
      sprintf(reply, "> %d", (uint32_t)_role_state->prefs.path_hash_mode);
#endif
    } else if (memcmp(key, "public.key", 10) == 0) {
      strcpy(reply, "> ");
      mesh::Utils::toHex(&reply[2], self_id.pub_key, PUB_KEY_SIZE);
    } else if (memcmp(key, "radio.fem.rxgain", 16) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.radio_fem_rxgain ? "on" : "off");
    } else if (memcmp(key, "radio", 5) == 0) {
      char freq[16], bw[16];
      strcpy(freq, StrHelper::ftoa(_role_state->prefs.freq));
      strcpy(bw, StrHelper::ftoa3(_role_state->prefs.bw));
      sprintf(reply, "> %s,%s,%d,%d", freq, bw, (uint32_t)_role_state->prefs.sf, (uint32_t)_role_state->prefs.cr);
    } else if (memcmp(key, "rxdelay", 7) == 0) {
      // beebo: repeater keeps its own independent copy -- see "af"'s comment above.
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.rx_delay_base));
    } else if (memcmp(key, "tx", 2) == 0 && (key[2] == 0 || key[2] == ' ')) {
      sprintf(reply, "> %d", (int32_t)_role_state->prefs.tx_power_dbm);
    } else if (memcmp(key, "freq", 4) == 0) {
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.freq));
    } else if (memcmp(key, "lat", 3) == 0) {
      // beebo: role's own PREFS-mode coordinate (_role_state->prefs.node_lat for
      // companion, _role_state->prefs.node_lat for repeater), not the shared
      // sensors.node_lat SHARE register -- see CMD_SET_ADVERT_LATLON's
      // comment.
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.node_lat));
    } else if (memcmp(key, "lon", 3) == 0) {
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.node_lon));
    } else if (memcmp(key, "wifi.ssid", 9) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.wifi_ssid);
    } else if (memcmp(key, "wifi.pwd", 8) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.wifi_pwd[0] ? "(set)" : "(unset)");
    } else if (memcmp(key, "transports", 10) == 0) {
      sprintf(reply, "> usb=%s ble=%s tcp=%s",
        _role_state->prefs.usb_enabled ? "on" : "off", _role_state->prefs.ble_enabled ? "on" : "off", _role_state->prefs.tcp_enabled ? "on" : "off");
    } else if (memcmp(key, "ble.pin", 7) == 0) {
      sprintf(reply, "> %lu", (unsigned long)_role_state->prefs.ble_pin);
    } else if (memcmp(key, "ble", 3) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.ble_enabled ? "on" : "off");
    } else if (memcmp(key, "tcp", 3) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.tcp_enabled ? "on" : "off");
    } else if (memcmp(key, "usb", 3) == 0) {
      sprintf(reply, "> %s", _role_state->prefs.usb_enabled ? "on" : "off");
    } else if (memcmp(key, "adc.multiplier", 14) == 0) {
      // beebo: report the resolved value (see tlvGetAdcMultiplier's comment),
      // not the raw persisted override which reads 0 pre-calibration.
      sprintf(reply, "> %.3f", board.getAdcMultiplier());
    } else if (memcmp(key, "adc.resolution", 14) == 0) {
      sprintf(reply, "> %u", _board.adc_resolution_bits);
    } else if (memcmp(key, "battery.sample_period", 21) == 0) {
      sprintf(reply, "> %u", _board.batt_sample_period_secs);
    } else if (memcmp(key, "battery.sample_window", 21) == 0) {
      sprintf(reply, "> %u", _board.batt_sample_window_secs);
    } else if (memcmp(key, "battery.present", 15) == 0) {
      sprintf(reply, "> %s", _board.batt_present == 2 ? "yes" : (_board.batt_present == 1 ? "no" : "unknown"));
    } else if (memcmp(key, "guest.password", 14) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %s", _role_state->prefs.guest_password);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "owner.info", 10) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t buf[128];
      int n = tlvGetOwnerInfo(this, NODE_ROLE_REPEATER, buf, sizeof(buf) - 1);
      buf[n] = 0;
      auto start = reply;
      *reply++ = '>'; *reply++ = ' ';
      const char* sp = (const char*)buf;
      while (*sp && reply - start < 159) { *reply++ = (*sp == '\n') ? '|' : *sp; sp++; }
      *reply = 0;
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "repeat", 6) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      // beebo: explicitly NODE_ROLE_REPEATER, not _role_state (whichever
      // role is live) -- this text-CLI entry point is reachable regardless
      // of live role (see this function's own top comment), so these
      // repeater-only keys must always target repeater's own slot.
      sprintf(reply, "> %s", tlvGetRepeatMode(this, NODE_ROLE_REPEATER) ? "on" : "off");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "txdelay", 7) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t txdelay_raw = tlvGetTxDelayFactor(this, NODE_ROLE_REPEATER); float txdelay_v; memcpy(&txdelay_v, &txdelay_raw, 4);
      sprintf(reply, "> %s", StrHelper::ftoa(txdelay_v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "direct.txdelay", 14) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t dtxdelay_raw = tlvGetDirectTxDelayFactor(this, NODE_ROLE_REPEATER); float dtxdelay_v; memcpy(&dtxdelay_v, &dtxdelay_raw, 4);
      sprintf(reply, "> %s", StrHelper::ftoa(dtxdelay_v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "allow.read.only", 15) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %s", tlvGetAllowReadOnly(this, NODE_ROLE_REPEATER) ? "on" : "off");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "agc.reset.interval", 18) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetAgcResetInterval(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "loop.detect", 11) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (uint8_t)tlvGetLoopDetect(this, NODE_ROLE_REPEATER);
      const char* names[] = {"off", "minimal", "moderate", "strict"};
      strcpy(reply, "> "); strcat(reply, v <= 3 ? names[v] : "strict");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.advert.interval", 21) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetFloodAdvertInterval(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "int.thresh", 10) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetInterferenceThreshold(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "advert.interval", 15) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetAdvertInterval(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max.advert", 16) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetFloodMaxAdvert(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max.unscoped", 18) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetFloodMaxUnscoped(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max", 9) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", tlvGetFloodMax(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "tune.enabled", 12) == 0) {
      // beebo: Phase A dynamic-tuning optimizer on/off switch -- RAM-only
      // (see _tune_enabled's declaration), so this always reads back "off"
      // after a reboot regardless of what was last set. Observe-only: even
      // "on" never calls a NodePrefs/ComPrefs setter (see TuneController.h).
      // This text-CLI path is USB-only (handleCommand() is only ever reached
      // via DualModeSerialInterface's local text CLI or a remote mesh admin
      // session -- never over this device's own BLE/TCP companion session);
      // BEEBO_CMD_GET/SET_TUNE_ENABLED (handleCmdFrame(), above) is the
      // binary-protocol equivalent reachable over BLE/TCP/USB, reading/
      // writing the exact same _tune_enabled field so neither path can see a
      // different value than the other.
      sprintf(reply, "> %s", _tune_enabled ? "on" : "off");
    } else if (memcmp(key, "tune.applied", 12) == 0) {
      // beebo: per-param live-actuation bitmask, same RAM-only/dual-path
      // (USB text CLI + BEEBO_CMD_GET/SET_TUNE_APPLIED_MASK) shape as
      // tune.enabled above. Plain decimal, one bit per TuneController::
      // specFor(i) -- matches every other byte-valued key in this file
      // (flood.max, int.thresh, ...), unlike monring.config's own hex
      // convention (a different key, different history).
      sprintf(reply, "> %u", (unsigned)_tune_applied_mask);
    } else {
#if BEEBO_ENABLE_REPEATER_ROLE
      if (isRepeater()) { cli.handleCommand(sender_timestamp, command, reply); return; }
#endif
      sprintf(reply, "??: %s", key);
    }
  } else if (memcmp(command, "set ", 4) == 0) {
    const char* key = &command[4];
    if (memcmp(key, "name ", 5) == 0) {
      if (isValidName(&key[5])) {
        StrHelper::strncpy(_role_state->prefs.node_name, &key[5], sizeof(_role_state->prefs.node_name));
        // beebo: the per-role dirty-routing pattern -- node_name is the
        // live role's own persisted name, reachable via this role-agnostic
        // text-CLI entry point while repeater is live -- distinct from
        // "repeater.name " below, which always targets repeater
        // explicitly regardless of which role is live (see getRoleName()).
        savePrefs();
        strcpy(reply, "OK");
      } else {
        strcpy(reply, "ERR: bad chars");
      }
    } else if (memcmp(key, "repeater.name ", 14) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      if (isValidName(&key[14])) {
        StrHelper::strncpy(_role_state->prefs.node_name, &key[14], sizeof(_role_state->prefs.node_name));
        _role_state->prefs.dirty = true;
        flushDirtyPrefs();
        strcpy(reply, "OK");
      } else {
        strcpy(reply, "ERR: bad chars");
      }
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "node.role ", 10) == 0) {
      const char* value = &key[10];
      uint8_t new_role;
      if (strcmp(value, "companion") == 0) new_role = NODE_ROLE_COMPANION;
      else if (strcmp(value, "repeater") == 0) new_role = NODE_ROLE_REPEATER;
      else { strcpy(reply, "ERR: must be companion or repeater"); return; }
      // beebo: always reboots on an actual change -- see
      // requestNodeRoleSwitch()'s own comment. No ack is sent in that case,
      // same as the "reboot"/"poweroff" text commands above.
      switch (requestNodeRoleSwitch(new_role, EVENT_SOURCE_TEXT_CLI)) {
        case NODE_ROLE_SWITCH_ERR_BUILTIN:
          // beebo: STATIC_ROLE_BUILDS -- see isNodeRoleBuiltIn()'s comment.
          strcpy(reply, "ERR: role not built into this firmware");
          break;
        case NODE_ROLE_SWITCH_ERR_BUSY:
          strcpy(reply, "ERR: busy (OTA or bulk transfer in progress)");
          break;
        case NODE_ROLE_SWITCH_NOOP:
        case NODE_ROLE_SWITCH_ERR_ARG:  // unreachable: value already validated above
          strcpy(reply, "OK");
          break;
        case NODE_ROLE_SWITCH_REBOOTING:
          board.reboot();  // doesn't return
          break;
      }
    } else if (memcmp(key, "multi.acks ", 11) == 0) {
      // beebo: repeater keeps its own independent copy -- repeater's own
      // independent value, mirroring "af"/"rxdelay"'s get-side branch above.
      uint8_t v = (uint8_t)constrain(atoi(&key[11]), 0, 1);
      _role_state->prefs.multi_acks = v;
      savePrefs();
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", (uint32_t)(_role_state->prefs.multi_acks));
#else
      sprintf(reply, "> %d", (uint32_t)_role_state->prefs.multi_acks);
#endif
    } else if (memcmp(key, "path.hash.mode ", 15) == 0) {
      int v = atoi(&key[15]);
      if (v < 0 || v > 2) { strcpy(reply, "ERR: must be 0, 1, or 2"); return; }
      _role_state->prefs.path_hash_mode = (uint8_t)v;
      savePrefs();
#if BEEBO_ENABLE_REPEATER_ROLE
      sprintf(reply, "> %d", (uint32_t)(_role_state->prefs.path_hash_mode));
#else
      sprintf(reply, "> %d", (uint32_t)_role_state->prefs.path_hash_mode);
#endif
    } else if (memcmp(key, "radio.fem.rxgain ", 17) == 0) {
      // beebo: reachable regardless of role, mirroring the existing lat/lon
      // pattern above -- including the same per-role dirty-write routing,
      // since radio_fem_rxgain is a per-role BeeboBasePrefs field like
      // advert_loc_policy/rx_delay_base/node_name, not a role-agnostic one.
      if (!board.canControlLoRaFemLna()) {
        strcpy(reply, "Error: unsupported");
      } else if (memcmp(&key[17], "on", 2) == 0) {
        if (board.setLoRaFemLnaEnabled(true)) {
          _role_state->prefs.radio_fem_rxgain = 1;
          savePrefs();
          strcpy(reply, "OK - LoRa FEM RX gain on");
        } else {
          strcpy(reply, "Error: failed to apply LoRa FEM RX gain");
        }
      } else if (memcmp(&key[17], "off", 3) == 0) {
        if (board.setLoRaFemLnaEnabled(false)) {
          _role_state->prefs.radio_fem_rxgain = 0;
          savePrefs();
          strcpy(reply, "OK - LoRa FEM RX gain off");
        } else {
          strcpy(reply, "Error: failed to apply LoRa FEM RX gain");
        }
      } else {
        strcpy(reply, "Error: state must be on or off");
      }
    } else if (memcmp(key, "adc.multiplier ", 15) == 0) {
      // beebo: fixes the same "set X has no interceptor" gap as "set lat"
      // below (BOARD_BATTERY_PREFS.md) -- adc.multiplier used to share
      // storage with upstream ComPrefs.adc_multiplier (inherited via
      // BeeboRepeaterPrefs), so falling through to real cli.handleCommand()
      // happened to land in the exact place "get adc.multiplier" read.
      // Now that adc.multiplier is beebo's own board-scoped
      // _board.adc_multiplier (BeeboBoardPrefs), that coincidence no
      // longer holds, so this needs an explicit interceptor same as "lat"/
      // "lon" got for the analogous reason. Reuses tlvSetAdcMultiplier()
      // so the clamp/live-apply/persist logic lives in exactly one place.
      // (adc.resolution/battery.* have no upstream CommonCLI.cpp
      // equivalent at all -- unlike adc.multiplier, they were never
      // reachable by "set" over a text-CLI CommonLink session before or
      // after this change, so no interceptor is needed for them here.
      // board.state.idle_margin used to be in this same no-SET-equivalent
      // group -- removed as a settings-tree leaf entirely, see
      // radioIsIdle()'s comment.)
      float v = atof(&key[15]);
      uint32_t raw; memcpy(&raw, &v, 4);
      tlvSetAdcMultiplier(this, this->_board.role, raw);
      sprintf(reply, "> %.3f", _board.adc_multiplier);
    } else if (memcmp(key, "lat ", 4) == 0) {
      // beebo: fixes the "set lat has no interceptor" gap flagged in
      // kbase/PROTOCOL_AND_SETTINGS_STORAGE.md's "critical trap" section --
      // without this, "set lat" fell through to real cli.handleCommand()
      // (repeater sessions only), writing _role_state->prefs.node_lat while every
      // read path (including "get lat" above) reads _role_state->prefs.node_lat,
      // silently diverging. Now symmetric with the get-side branch above.
      // Writes the role's own PREFS-mode coordinate, not the shared
      // sensors.node_lat SHARE register -- see CMD_SET_ADVERT_LATLON's
      // comment.
      double v = atof(&key[4]);
      _role_state->prefs.node_lat = v;
      savePrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.node_lat));
    } else if (memcmp(key, "lon ", 4) == 0) {
      double v = atof(&key[4]);
      _role_state->prefs.node_lon = v;
      savePrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(_role_state->prefs.node_lon));
    } else if (memcmp(key, "wifi.ssid ", 10) == 0) {
      StrHelper::strncpy(_role_state->prefs.wifi_ssid, &key[10], sizeof(_role_state->prefs.wifi_ssid));
      // beebo: the per-role dirty-routing pattern -- wifi_ssid/wifi_pwd
      // are per-role fields, reachable via this text-CLI admin channel
      // while repeater is live.
      savePrefs();
      strcpy(reply, "OK - reboot to apply");
    } else if (memcmp(key, "wifi.pwd ", 9) == 0) {
      StrHelper::strncpy(_role_state->prefs.wifi_pwd, &key[9], sizeof(_role_state->prefs.wifi_pwd));
      savePrefs();
      strcpy(reply, "OK - reboot to apply");
    } else if (memcmp(key, "ble ", 4) == 0) {
      // beebo: text-CLI parity for the admin-over-mesh channel -- reuses
      // tlvSetTransportConfig() for the ble/tcp mutual-exclusion and
      // usb-fallback invariants, rather than re-deriving them here.
      bool on = memcmp(&key[4], "on", 2) == 0;
      bool want_reboot = strstr(&key[4], "--reboot") != NULL;
      uint32_t raw = tlvGetTransportConfig(this, this->_board.role);
      if (on) raw |= 0xFF; else raw &= ~(uint32_t)0xFF;
      tlvSetTransportConfig(this, this->_board.role, raw);
      if (want_reboot) {
        _ota_restart_time = futureMillis(750);
        _ota_restart_ts = 0;
      } else {
        _transport_config_pending = true;   // signal main.cpp to apply live
      }
      sprintf(reply, "OK - ble is now %s%s", _role_state->prefs.ble_enabled ? "ON" : "OFF", want_reboot ? " (rebooting)" : "");
    } else if (memcmp(key, "tcp ", 4) == 0) {
      bool on = memcmp(&key[4], "on", 2) == 0;
      bool want_reboot = strstr(&key[4], "--reboot") != NULL;
      uint32_t raw = tlvGetTransportConfig(this, this->_board.role);
      if (on) raw |= (uint32_t)0xFF << 8; else raw &= ~((uint32_t)0xFF << 8);
      tlvSetTransportConfig(this, this->_board.role, raw);
      if (want_reboot) {
        _ota_restart_time = futureMillis(750);
        _ota_restart_ts = 0;
      } else {
        _transport_config_pending = true;
      }
      sprintf(reply, "OK - tcp is now %s%s", _role_state->prefs.tcp_enabled ? "ON" : "OFF", want_reboot ? " (rebooting)" : "");
    } else if (memcmp(key, "usb ", 4) == 0) {
      // usb has no live add/remove path -- a reboot is always required to
      // apply, whether or not --reboot was passed; the flag only decides
      // whether that reboot happens now (deferred here) or is left for the
      // caller to trigger later, same as the binary opcode's own comment.
      bool on = memcmp(&key[4], "on", 2) == 0;
      bool want_reboot = strstr(&key[4], "--reboot") != NULL;
      uint32_t raw = tlvGetTransportConfig(this, this->_board.role);
      if (on) raw |= (uint32_t)0xFF << 16; else raw &= ~((uint32_t)0xFF << 16);
      tlvSetTransportConfig(this, this->_board.role, raw);
      if (want_reboot) {
        _ota_restart_time = futureMillis(750);
        _ota_restart_ts = 0;
        sprintf(reply, "OK - usb is now %s (rebooting)", _role_state->prefs.usb_enabled ? "ON" : "OFF");
      } else {
        sprintf(reply, "OK - reboot to apply, usb is now %s", _role_state->prefs.usb_enabled ? "ON" : "OFF");
      }
    } else if (memcmp(key, "guest.password ", 15) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      StrHelper::strncpy(_role_state->prefs.guest_password, &key[15], sizeof(_role_state->prefs.guest_password));
      _role_state->prefs.dirty = true;
      flushDirtyPrefs();
      sprintf(reply, "guest.password now: %s", _role_state->prefs.guest_password);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "owner.info ", 11) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      char tmp[120];
      StrHelper::strncpy(tmp, &key[11], sizeof(tmp));
      for (char* p = tmp; *p; p++) if (*p == '|') *p = '\n';  // matches CommonCLI's own '|' -> '\n' convention
      tlvSetOwnerInfo(this, NODE_ROLE_REPEATER, (const uint8_t*)tmp, strlen(tmp));
      flushDirtyPrefs();
      strcpy(reply, "OK");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "repeat ", 7) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t enabled = (memcmp(&key[7], "on", 2) == 0) ? 1 : 0;
      tlvSetRepeatMode(this, NODE_ROLE_REPEATER, enabled);
      flushDirtyPrefs();
      sprintf(reply, "> %s", enabled ? "on" : "off");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "rxdelay ", 8) == 0) {
      // beebo: repeater keeps its own independent copy -- repeater-only key
      // (matches the GET side's role branch above); companion continues to
      // set the shared value via CMD_SET_TUNING_PARAMS only, unaffected.
#if BEEBO_ENABLE_REPEATER_ROLE
      float v = atof(&key[8]);
      _role_state->prefs.rx_delay_base = v;
      _role_state->prefs.dirty = true;
      flushDirtyPrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "af ", 3) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      float v = atof(&key[3]);
      _role_state->prefs.airtime_factor = v;
      _role_state->prefs.dirty = true;
      flushDirtyPrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "txdelay ", 8) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      float v = atof(&key[8]);
      uint32_t raw; memcpy(&raw, &v, 4);
      tlvSetTxDelayFactor(this, NODE_ROLE_REPEATER, raw);
      flushDirtyPrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "direct.txdelay ", 15) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      float v = atof(&key[15]);
      uint32_t raw; memcpy(&raw, &v, 4);
      tlvSetDirectTxDelayFactor(this, NODE_ROLE_REPEATER, raw);
      flushDirtyPrefs();
      sprintf(reply, "> %s", StrHelper::ftoa(v));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "allow.read.only ", 16) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (memcmp(&key[16], "on", 2) == 0) ? 1 : 0;
      tlvSetAllowReadOnly(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %s", v ? "on" : "off");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "agc.reset.interval ", 19) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t v = (uint32_t)atoi(&key[19]);
      tlvSetAgcResetInterval(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %d", tlvGetAgcResetInterval(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "loop.detect ", 12) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      const char* mode = &key[12];
      uint8_t v;
      if (strcmp(mode, "off") == 0) v = 0;
      else if (strcmp(mode, "minimal") == 0) v = 1;
      else if (strcmp(mode, "moderate") == 0) v = 2;
      else if (strcmp(mode, "strict") == 0) v = 3;
      else { strcpy(reply, "ERR: must be off, minimal, moderate, or strict"); return; }
      tlvSetLoopDetect(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      strcpy(reply, "OK");
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.advert.interval ", 22) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (uint8_t)atoi(&key[22]);
      tlvSetFloodAdvertInterval(this, NODE_ROLE_REPEATER, v);   // also rearms the flood-advert timer internally
      flushDirtyPrefs();
      sprintf(reply, "> %d", (uint32_t)v);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "int.thresh ", 11) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t v = (uint32_t)constrain(atoi(&key[11]), 0, 9);
      tlvSetInterferenceThreshold(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %d", v);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "advert.interval ", 16) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint32_t v = (uint32_t)atoi(&key[16]);
      tlvSetAdvertInterval(this, NODE_ROLE_REPEATER, v);   // also rearms the local-advert timer internally; stores raw/2 -- see tlvSetAdvertInterval's own comment
      flushDirtyPrefs();
      sprintf(reply, "> %d", tlvGetAdvertInterval(this, NODE_ROLE_REPEATER));
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max.advert ", 17) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (uint8_t)atoi(&key[17]);
      tlvSetFloodMaxAdvert(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %d", (uint32_t)v);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max.unscoped ", 19) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (uint8_t)atoi(&key[19]);
      tlvSetFloodMaxUnscoped(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %d", (uint32_t)v);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "flood.max ", 10) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
      uint8_t v = (uint8_t)atoi(&key[10]);
      tlvSetFloodMax(this, NODE_ROLE_REPEATER, v);
      flushDirtyPrefs();
      sprintf(reply, "> %d", (uint32_t)v);
#else
      strcpy(reply, "ERR: not supported");
#endif
    } else if (memcmp(key, "tune.enabled ", 13) == 0) {
      const char* v = &key[13];
      if (memcmp(v, "on", 2) == 0) {
        setTuneEnabled(true, EVENT_SOURCE_TEXT_CLI);
      } else if (memcmp(v, "off", 3) == 0) {
        setTuneEnabled(false, EVENT_SOURCE_TEXT_CLI);
      } else {
        strcpy(reply, "ERR: expected on/off");
        return;
      }
      sprintf(reply, "> %s", _tune_enabled ? "on" : "off");
    } else if (memcmp(key, "tune.applied ", 13) == 0) {
      setTuneAppliedMask((uint8_t)atoi(&key[13]), EVENT_SOURCE_TEXT_CLI);
      sprintf(reply, "> %u", (unsigned)_tune_applied_mask);
    } else {
#if BEEBO_ENABLE_REPEATER_ROLE
      if (isRepeater()) { cli.handleCommand(sender_timestamp, command, reply); return; }
#endif
      sprintf(reply, "ERR: unknown key: %s", key);
    }
  } else if (memcmp(command, "password ", 9) == 0) {
#if BEEBO_ENABLE_REPEATER_ROLE
    StrHelper::strncpy(_role_state->prefs.password, &command[9], sizeof(_role_state->prefs.password));
    _role_state->prefs.dirty = true;
    flushDirtyPrefs();
    sprintf(reply, "password now: %s", _role_state->prefs.password);
#else
    strcpy(reply, "ERR: not supported");
#endif
  // beebo: requesting side of
  // the "node discover" protocol, extended with an optional type-list
  // argument (repeater/chat/room/sensor/all) -- simple_repeater's own
  // version only ever asked for repeaters. Not
  // role-gated: unlike answering a request (which replies as whatever this
  // node's own current type is), asking "who's near me" has no such
  // constraint -- responses are logged into putNeighbour(), the same
  // shared neighbour table both roles already use.
  } else if (memcmp(command, "discover.neighbors", 19) == 0) {
    const char* sub = command + 19;
    while (*sub == ' ') sub++;
    if (*sub == 0) {
      sendNodeDiscoverReq(1 << ADV_TYPE_REPEATER);   // preserve old (repeater-only) default
      strcpy(reply, "OK - Discover sent");
    } else {
      char buf[64];
      size_t buf_len = strlen(sub);
      if (buf_len >= sizeof(buf)) buf_len = sizeof(buf) - 1;
      memcpy(buf, sub, buf_len); buf[buf_len] = 0;

      uint8_t filter = 0;
      bool bad = false;
      char* tok = strtok(buf, ", ");
      while (tok) {
        if (strcmp(tok, "all") == 0) {
          filter |= (1 << ADV_TYPE_CHAT) | (1 << ADV_TYPE_REPEATER)
                  | (1 << ADV_TYPE_ROOM) | (1 << ADV_TYPE_SENSOR);
        } else if (strcmp(tok, "repeater") == 0) {
          filter |= (1 << ADV_TYPE_REPEATER);
        } else if (strcmp(tok, "chat") == 0) {
          filter |= (1 << ADV_TYPE_CHAT);
        } else if (strcmp(tok, "room") == 0) {
          filter |= (1 << ADV_TYPE_ROOM);
        } else if (strcmp(tok, "sensor") == 0) {
          filter |= (1 << ADV_TYPE_SENSOR);
        } else {
          bad = true;
          break;
        }
        tok = strtok(NULL, ", ");
      }
      if (bad || filter == 0) {
        strcpy(reply, "Err - discover.neighbors [repeater|chat|room|sensor|all ...]");
      } else {
        sendNodeDiscoverReq(filter);
        strcpy(reply, "OK - Discover sent");
      }
    }
  } else {
#if BEEBO_ENABLE_REPEATER_ROLE
    if (isRepeater()) { cli.handleCommand(sender_timestamp, command, reply); return; }
#endif
    strcpy(reply, "ERR: unknown command");
  }
}
