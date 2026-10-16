#include "SerialBLEInterface.h"
#include "esp_mac.h"
#include <host/ble_hs.h>
#include "../DebugLog.h"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define ADVERT_RESTART_DELAY  1000   // millis

// Actual GATT link state, captured directly in the BLE-host-task callbacks
// (onConnect/onDisconnect) so the debug ring shows the REAL link up/down — not
// the derived deviceConnected flag. Counters are incremented in the BT task and
// drained into the (non-thread-safe) ring from the main loop in checkRecvFrame.
static volatile uint8_t s_ble_link_up_cnt = 0;
static volatile uint8_t s_ble_link_down_cnt = 0;

void SerialBLEInterface::begin(const char* prefix, char* name, uint32_t pin_code) {
  _pin_code = pin_code;

  if (strcmp(name, "@@MAC") == 0) {
    uint8_t addr[8];
    memset(addr, 0, sizeof(addr));
    esp_efuse_mac_get_default(addr);
    sprintf(name, "%02X%02X%02X%02X%02X%02X",    // modify (IN-OUT param)
          addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
  }
  snprintf(_dev_name, sizeof(_dev_name), "%s%s", prefix, name);

  initRadio();
}

// (Re)create the entire BLE stack and GATT server. Safe to call again after
// deinitRadio(). Does NOT start advertising — enable() does that.
//
// Recreates pServer/pService/pTxCharacteristic (heap allocations) every time.
// This is a deliberate, bounded exception to "no allocation outside setup()/
// begin()": BLEDevice::deinit() deletes the server, so the objects from a
// prior initRadio() can't be reused. Only runs on an explicit user-
// triggered BLE on/off toggle (CMD_SET_TRANSPORT_CONFIG / "set ble"), never
// in a hot loop.
void SerialBLEInterface::initRadio() {
  // Create the BLE Device
  BLEDevice::init(_dev_name);

  // beebo: cache this device's own advertised BLE address (MSB first; NimBLE
  // hands it out LSB first) -- see getLocalAddress()'s own comment for why
  // this isn't logged here directly (Beebo::_checkTransportStateChanges()
  // logs it instead, alongside the rest of the boot-time transport-state
  // dump).
  BLEAddress local = BLEDevice::getAddress();
  const uint8_t* lsb = local.getNative();
  for (int i = 0; i < 6; i++) _local_bda[i] = lsb[5 - i];

  _tx_subscribed = false;
  _notify_pending = false;
  BLEDevice::setSecurityCallbacks(this);
  BLEDevice::setMTU(MAX_FRAME_SIZE);

  // BLEDevice::init() resets the NimBLE security config, so it is set here
  // on every init. The constructor resets the I/O capability, hence the
  // order. Display-only I/O with a static passkey, MITM + SC + bonding.
  BLESecurity sec;
  sec.setPassKey(true, _pin_code);
  sec.setCapability(ESP_IO_CAP_OUT);
  // beebo: iOS's SMP implementation is stricter than Android's about the
  // bonding key set actually offered, and confirmed on real hardware to cost
  // several seconds of silent stall during the key-exchange step on a *fresh*
  // pair (never on a reconnect, which skips key distribution and reuses the
  // existing LTK) if the set is incomplete -- see RLOG_ID_BLE_HANDSHAKE's
  // own comment in DebugLog.h. Both masks are set explicitly to the same
  // value.
  sec.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec.setAuthenticationMode(true, true, true);

  //BLEDevice::setPower(ESP_PWR_LVL_N8);

  // Create the BLE Server (advertising restarts are handled by this class,
  // not by BLEServer::advertiseOnDisconnect(), which defaults to off)
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(this);

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create the BLE Characteristics. NimBLE ignores setAccessPermissions();
  // the property flags carry the security requirement (authenticated, i.e.
  // MITM, encryption) and the CCCD is added automatically for NOTIFY.
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_READ_AUTHEN | BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->setCallbacks(this);

  pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_AUTHEN
  );
  pRxCharacteristic->setCallbacks(this);

  // NimBLE builds the GATT table here and cannot stop a service afterwards,
  // so enable()/disable() only start/stop advertising and drop the link.
  pService->start();

  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
}

void SerialBLEInterface::deinitRadio() {
  _rssi_cache = BLE_RSSI_UNAVAILABLE;
  // release_memory=true never clears BLEDevice's internal "initialized" latch
  // (see BLEDevice::deinit() in the Arduino BLE lib), so a later initRadio()
  // would silently no-op and BLE would never actually come back. false still
  // fully stops the NimBLE host and the BT controller (radio powers down),
  // it just leaves the controller memory region alone so re-init works.
  // deinit() also deletes the server and advertising objects, so only our
  // pointers are cleared here.
  BLEDevice::deinit(false);
  pServer = NULL;
  pService = NULL;
  pTxCharacteristic = NULL;
  pRxCharacteristic = NULL;
  _tx_subscribed = false;
  last_conn_id = 0;
  oldDeviceConnected = deviceConnected = false;
  adv_restart_time = 0;
  clearBuffers();
}

// Change the pairing PIN without a reboot. NimBLE reads the static passkey
// from BLESecurity when pairing starts, so updating it is enough; the
// MITM/SC/bonding mode is untouched.
//
// Only affects future pairings: an already-bonded peer keeps its bond and
// never re-enters a PIN. Skipped entirely when the stack is down (pServer
// NULL) -- initRadio() will apply the new _pin_code when BLE next comes up.
void SerialBLEInterface::setPinCode(uint32_t pin_code) {
  _pin_code = pin_code;
  if (pServer == NULL) return;
  BLESecurity::setPassKey(true, _pin_code);
}

// -------- BLESecurityCallbacks methods

uint32_t SerialBLEInterface::onPassKeyRequest() {
  BLE_DEBUG_PRINTLN("onPassKeyRequest()");
  RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_PASSKEY_REQUEST);
  return _pin_code;
}

void SerialBLEInterface::onPassKeyNotify(uint32_t pass_key) {
  BLE_DEBUG_PRINTLN("onPassKeyNotify(%u)", pass_key);
}

bool SerialBLEInterface::onConfirmPIN(uint32_t pass_key) {
  BLE_DEBUG_PRINTLN("onConfirmPIN(%u)", pass_key);
  return true;
}

bool SerialBLEInterface::onSecurityRequest() {
  BLE_DEBUG_PRINTLN("onSecurityRequest()");
  RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_SECURITY_REQUEST);
  return true;  // allow
}

void SerialBLEInterface::onAuthenticationComplete(ble_gap_conn_desc* desc, int status) {
  if (status == 0) {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Success");
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_AUTH_COMPLETE | (1 << 8));
    deviceConnected = true;
  } else {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Failure*");
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_AUTH_COMPLETE | ((int32_t)(status & 0x7FFF) << 16));

    pServer->disconnect(desc->conn_handle);
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;
  }
}

// -------- BLEServerCallbacks methods

void SerialBLEInterface::onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
  BLE_DEBUG_PRINTLN("onConnect(), conn_id=%d, mtu=%d", desc->conn_handle, pServer->getPeerMTU(desc->conn_handle));
  last_conn_id = desc->conn_handle;
  _tx_subscribed = false;
  for (int i = 0; i < 6; i++) _remote_bda[i] = desc->peer_ota_addr.val[5 - i];   // NimBLE: LSB first
  s_ble_link_up_cnt++;   // actual GATT link came up (logged from main loop)
}

void SerialBLEInterface::onMtuChanged(BLEServer* pServer, ble_gap_conn_desc* desc, uint16_t mtu) {
  BLE_DEBUG_PRINTLN("onMtuChanged(), mtu=%d", mtu);
}

void SerialBLEInterface::onDisconnect(BLEServer* pServer, ble_gap_conn_desc* desc) {
  BLE_DEBUG_PRINTLN("onDisconnect()");
  s_ble_link_down_cnt++;   // actual GATT link dropped (logged from main loop)
  _rssi_cache = BLE_RSSI_UNAVAILABLE;   // stale once the central it was read from is gone
  // beebo: set directly here (event-driven), matching upstream's own fix
  // ("preserve bonded BLE reconnects") -- checkRecvFrame() used to instead
  // poll `pServer->getConnectedCount() == 0` on every call to derive this,
  // which can transiently read 0 mid-handshake on a *bonded* reconnect
  // (the stack briefly reports no connected peer while re-establishing the
  // link from a cached bond) even though the link isn't really down --
  // tearing deviceConnected down on that false reading is exactly the
  // "stuck at Connecting" symptom: the app's new connection attempt then
  // has nothing consistent to attach to.
  deviceConnected = false;
  _tx_subscribed = false;
  _notify_pending = false;   // nothing to retry on a dead link
  if (_isEnabled) {
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;
  }
}

// -------- BLECharacteristicCallbacks methods

void SerialBLEInterface::onWrite(BLECharacteristic* pCharacteristic) {
  uint8_t* rxValue = pCharacteristic->getData();
  int len = pCharacteristic->getLength();

  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("ERROR: onWrite(), frame too big, len=%d", len);
  } else {
    Frame frame = {};
    frame.len = len;
    memcpy(frame.buf, rxValue, len);

    if (xQueueSend(recv_queue, &frame, 0) != pdTRUE) {
      BLE_DEBUG_PRINTLN("ERROR: onWrite(), recv_queue is full!");
      _recv_queue_full_count++;
    }
  }
}

// beebo: RLOG_ID_BLE_HANDSHAKE's CCCD_WRITE step -- see its own comment in
// DebugLog.h. subValue bit0 = notifications enabled.
void SerialBLEInterface::onSubscribe(BLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
  if (pCharacteristic != pTxCharacteristic) return;
  bool enabled = subValue & 0x01;
  _tx_subscribed = enabled;
  RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_CCCD_WRITE | ((enabled ? 1 : 0) << 8));
}

// ---------- public methods

void SerialBLEInterface::enable() {
  if (_isEnabled) return;
  // beebo: MultiSerialInterface::release() unconditionally re-enables every
  // other exclusive transport whenever any session ends -- including BLE
  // after applyTransportConfig() has deliberately deinitRadio()'d it (pServer/
  // pService set NULL, SerialBLEInterface.cpp's deinitRadio()). _isEnabled is
  // already false by then too (disable() runs before deinitRadio() in
  // applyTransportConfig()), so the guard above doesn't catch this case --
  // pServer->getAdvertising() below would crash (LoadProhibited) on the null pointer.
  // Reproduced on real hardware as a repeatable BLE<->TCP switch reboot,
  // the mirror-image case of disable()'s own guard.
  if (pServer == NULL) return;

  _isEnabled = true;
  clearBuffers();

  // Start advertising

  //pServer->getAdvertising()->setMinInterval(500);
  //pServer->getAdvertising()->setMaxInterval(1000);

  pServer->getAdvertising()->start();
  adv_restart_time = 0;
}

void SerialBLEInterface::disable() {
  // beebo: MultiSerialInterface::lockOn() calls disable() on every other
  // exclusive transport whenever one wins the session-lock race -- if BLE
  // was already disabled moments earlier by applyTransportConfig()'s own
  // deliberate teardown (the common BLE->TCP live-switch case), this
  // re-enters an already-torn-down BLE stack (pServer is gone), which
  // crashed on the Bluedroid stack -- reproduced on real hardware as a
  // BLE->TCP switch reboot. enable() already guards the same way
  // (see its own `if (_isEnabled) return;` above); disable() needs the
  // identical guard.
  if (!_isEnabled) return;
  _isEnabled = false;

  BLE_DEBUG_PRINTLN("SerialBLEInterface::disable");

  pServer->getAdvertising()->stop();
  if (pServer->getConnectedCount() > 0) pServer->disconnect(last_conn_id);
  _tx_subscribed = false;
  oldDeviceConnected = deviceConnected = false;
  adv_restart_time = 0;
}

// Forcibly evict a central that connected while this transport isn't
// the session owner (MultiSerialInterface's non-owner-eviction pass).
// Unlike disable(), this leaves advertising/the GATT service running --
// this transport keeps listening for the *next* legitimate session, only
// the stray peer that shouldn't have been allowed to attach is dropped.
// No forcible-disconnect capability existed here before: onAuthenticationComplete()
// sets deviceConnected straight from the BLE stack, with no session-ownership
// concept of its own to guard it.
void SerialBLEInterface::resetParserState() {
  if (pServer == NULL || !_isEnabled) return;
  if (deviceConnected) {
    pServer->disconnect(last_conn_id);
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;
  }
  oldDeviceConnected = deviceConnected = false;
  clearBuffers();
}

size_t SerialBLEInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("writeFrame(), frame too big, len=%d", len);
    return 0;
  }

  if (deviceConnected && len > 0) {
    if (send_queue_len >= FRAME_QUEUE_SIZE) {
      BLE_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
      _send_queue_full_count++;
      return 0;
    }

    send_queue[send_queue_len].len = len;  // add to send queue
    memcpy(send_queue[send_queue_len].buf, src, len);
    send_queue_len++;

    return len;
  }
  return 0;
}

bool SerialBLEInterface::isWriteBusy() const {
  // beebo: two independent reasons to hold off the producer -- _notify_pending
  // (real backpressure: the last ble_gatts_notify_custom() found the
  // controller/mbuf pool full, see checkRecvFrame()) and the queue-depth
  // check (same back-pressure as SerialWifiInterface::isWriteBusy()) catching
  // the case where the real link drains slower than one frame per call, so
  // a burst (e.g. GET_CONTACTS) doesn't silently overflow the 6-slot
  // send_queue.
  return _notify_pending
    || send_queue_len >= FRAME_QUEUE_SIZE - 1;
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[], size_t max_len, RecvFrameType* type) {
  // Drain actual GATT link events captured in the BT-task callbacks into the
  // ring (main-loop context, so the ring stays single-writer).
  static uint8_t seen_up = 0, seen_down = 0;
  // beebo: RLOG_ID_APP_SESSION_START logged here, alongside the link-up event --
  // BLE has no separate app-level accept step the way TCP does (a raw socket
  // accept ahead of any frame), the GATT connection itself IS the session,
  // so this is BLE's equivalent of SerialWifiInterface's own accept-path log
  // (see MultiSerialInterface.h's lockOn() comment for why it's logged here,
  // not there).
  while (seen_up != s_ble_link_up_cnt)   {
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_CONNECT);
    int32_t addr_hi = ((int32_t)_remote_bda[0] << 8) | (int32_t)_remote_bda[1];
    int32_t addr_lo = ((int32_t)_remote_bda[2] << 24) | ((int32_t)_remote_bda[3] << 16)
                     | ((int32_t)_remote_bda[4] << 8) | (int32_t)_remote_bda[5];
    RLOGM(RLOG_ID_BLE_CLIENT_ADDR_HI, addr_hi);
    RLOGM(RLOG_ID_BLE_CLIENT_ADDR_LO, addr_lo);
    RLOGH(RLOG_ID_APP_SESSION_START, RLOG_ID_XPORT_BLE);
    seen_up++;
  }
  while (seen_down != s_ble_link_down_cnt) { RLOGH(RLOG_ID_BLE_DISCONNECT); seen_down++; }

  // beebo: hold the queue until the client has actually subscribed to the TX
  // characteristic (onSubscribe()). Right after a *fresh* pairing, iOS
  // completes bonding before it writes the CCCD, so a reply sent in that
  // window would vanish for good while the frame was already popped.
  // Confirmed on real hardware (2026-09-09): a fresh-pair BLE connect would
  // get 1-2 early command replies through, then the client's next request
  // never arrived at all (it was still waiting on the reply it never got),
  // stalling until the client's own ~30s timeout disconnected it -- a
  // reconnect using the now-established bond (CCCD already subscribed) never
  // reproduced it.
  if (send_queue_len > 0 && deviceConnected && _tx_subscribed) {
    _last_write = millis();
    // ble_gatts_notify_custom() always consumes the mbuf, success or not.
    struct os_mbuf* om = ble_hs_mbuf_from_flat(send_queue[0].buf, send_queue[0].len);
    int rc = (om != nullptr)
      ? ble_gatts_notify_custom(last_conn_id, pTxCharacteristic->getHandle(), om)
      : BLE_HS_ENOMEM;
    if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY || rc == BLE_HS_EAGAIN) {
      _notify_pending = true;   // controller/mbuf pool full: keep the frame, retry next call
    } else {
      _notify_pending = false;
      if (rc != 0) BLE_DEBUG_PRINTLN("writeBytes: notify rc=%d, frame dropped", rc);
      BLE_DEBUG_PRINTLN("writeBytes: sz=%d, hdr=%d", (uint32_t)send_queue[0].len, (uint32_t) send_queue[0].buf[0]);

      send_queue_len--;
      for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
        send_queue[i] = send_queue[i + 1];
      }
    }
  }

  {
    Frame frame;
    if (xQueueReceive(recv_queue, &frame, 0) == pdTRUE) {
      size_t len = frame.len;
      memcpy(dest, frame.buf, len);

      BLE_DEBUG_PRINTLN("readBytes: sz=%d, hdr=%d", len, (uint32_t) dest[0]);

      if (type) *type = RecvFrameType::BINARY;
      return len;
    }
  }

  if (deviceConnected != oldDeviceConnected) {
    if (!deviceConnected) {    // disconnecting
      clearBuffers();

      BLE_DEBUG_PRINTLN("SerialBLEInterface -> disconnecting...");

      //pServer->getAdvertising()->setMinInterval(500);
      //pServer->getAdvertising()->setMaxInterval(1000);

      adv_restart_time = millis() + ADVERT_RESTART_DELAY;
    } else {
      BLE_DEBUG_PRINTLN("SerialBLEInterface -> stopping advertising");
      BLE_DEBUG_PRINTLN("SerialBLEInterface -> connecting...");
      // connecting
      // do stuff here on connecting
      pServer->getAdvertising()->stop();
      adv_restart_time = 0;
      // beebo: force requestHealthSample()'s own periodic cadence to fire
      // on the very next loopTransports() tick instead of waiting for
      // whatever's left of BLE_HEALTH_SAMPLE_MS -- that cadence may
      // already have been satisfied minutes ago, while nobody was even
      // connected yet (during advertising), and a connection this
      // short-lived (a single non-interactive CLI command) can otherwise
      // end before the next periodic sample ever comes around.
      _last_health_sample_ms = millis() - BLE_HEALTH_SAMPLE_MS;
    }
    oldDeviceConnected = deviceConnected;
  }

  if (adv_restart_time && millis() >= adv_restart_time) {
    if (pServer->getConnectedCount() == 0) {
      BLE_DEBUG_PRINTLN("SerialBLEInterface -> re-starting advertising");
      pServer->getAdvertising()->start();  // re-Start advertising
    }
    adv_restart_time = 0;
  }
  return 0;
}

bool SerialBLEInterface::isConnected() const {
  return deviceConnected;  //pServer != NULL && pServer->getConnectedCount() > 0;
}

// beebo: RLOG_ID_BLE_RSSI_REQUESTED/_COMPLETE's own comment in DebugLog.h has
// the story. ble_gap_conn_rssi() is a synchronous HCI read, so unlike the
// old async read there is nothing left in flight for a teardown to race;
// both ring entries are logged back to back.
void SerialBLEInterface::_requestRssiReadIfIdle() {
  if (!deviceConnected) return;
  int8_t rssi;
  RLOGH(RLOG_ID_BLE_RSSI_REQUESTED);
  if (ble_gap_conn_rssi(last_conn_id, &rssi) == 0) {
    _rssi_cache = rssi;
    RLOGH(RLOG_ID_BLE_RSSI_COMPLETE, (int32_t)rssi);
  }
}

void SerialBLEInterface::requestHealthSample() {
  // beebo: unlike DLOG_ID_WIFI_HEALTH (gated on the radio being up, not a
  // live app session -- WiFi's heap/channel reading is meaningful the
  // moment the STA associates, before any companion app connects), BLE's
  // own heap number never actually moves between samples while
  // advertising with nobody connected, and rssi is only ever
  // BLE_RSSI_UNAVAILABLE until a central shows up to read it from -- so a
  // sample logged in that state is pure noise. Gated on deviceConnected,
  // not just _isEnabled/advertising.
  if (!_isEnabled || !deviceConnected) return;
  if (millis() - _last_health_sample_ms < BLE_HEALTH_SAMPLE_MS) return;
  _last_health_sample_ms = millis();
  uint16_t heap_kb = (uint16_t)(ESP.getFreeHeap() / 1024);
  // beebo: rssi isn't logged here -- the read below logs it as
  // RLOG_ID_BLE_RSSI_COMPLETE.
  DLOGL(DLOG_ID_BLE_HEALTH, "heap=%uKB", heap_kb);
  _requestRssiReadIfIdle();
}
