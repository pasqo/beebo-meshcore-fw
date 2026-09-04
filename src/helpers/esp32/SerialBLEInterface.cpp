#include "SerialBLEInterface.h"
#include "esp_mac.h"
#include "../TransportLog.h"

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
// begin()": createService()/createCharacteristic() register synchronously
// with the Bluedroid GATTS app that BLEDevice::init() just (re)created, so
// the old C++ objects from a prior initRadio() are bound to a now-torn-down
// registration and can't be reused as-is. Only runs on an explicit user-
// triggered BLE on/off toggle (CMD_SET_TRANSPORT_CONFIG / "set ble"), never
// in a hot loop.
void SerialBLEInterface::initRadio() {
  // Create the BLE Device
  BLEDevice::init(_dev_name);
  BLEDevice::setSecurityCallbacks(this);
  BLEDevice::setMTU(MAX_FRAME_SIZE);

  BLESecurity  sec;
  sec.setStaticPIN(_pin_code);
  sec.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);

  //BLEDevice::setPower(ESP_PWR_LVL_N8);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(this);

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  pTxDescriptor = new BLE2902();
  pTxCharacteristic->addDescriptor(pTxDescriptor);

  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  pRxCharacteristic->setCallbacks(this);

  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
}

void SerialBLEInterface::deinitRadio() {
  // release_memory=true never clears BLEDevice's internal "initialized" latch
  // (see BLEDevice::deinit() in the Arduino BLE lib), so a later initRadio()
  // would silently no-op and BLE would never actually come back. false still
  // fully disables/deinits Bluedroid + the BT controller (radio powers down),
  // it just leaves the controller memory region alone so re-init works.
  BLEDevice::deinit(false);
  // BLEDevice::deinit() only tears down the underlying Bluedroid/BT-controller
  // stack -- it never frees the C++ wrapper objects initRadio() allocated
  // (BLEServer/BLEService/BLECharacteristic/BLE2902 have no cascading
  // destructors in the Arduino BLE lib), so free them explicitly here or
  // every ble on/off cycle leaks them.
  delete pTxDescriptor;
  delete pTxCharacteristic;
  delete pRxCharacteristic;
  delete pService;
  delete pServer;
  pServer = NULL;
  pService = NULL;
  pTxCharacteristic = NULL;
  pRxCharacteristic = NULL;
  pTxDescriptor = NULL;
  last_conn_id = 0;
  oldDeviceConnected = deviceConnected = false;
  adv_restart_time = 0;
  clearBuffers();
}

// Change the pairing PIN without a reboot. The pin reaches the stack two ways
// and both are covered here: the GAP static-passkey security param (what
// initRadio() sets, re-applied live below via the same calls in the same
// order) and onPassKeyRequest()'s return value (just _pin_code, so updating
// the field is enough).
//
// setStaticPIN() ends with setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY) of
// its own, which is NOT what initRadio() leaves configured -- so the
// SC_MITM_BOND mode is re-asserted right after, exactly as initRadio() does,
// or a live PIN change would quietly drop the MITM requirement.
//
// Only affects future pairings: an already-bonded peer keeps its bond and
// never re-enters a PIN. Skipped entirely when the stack is down (pServer
// NULL) -- initRadio() will apply the new _pin_code when BLE next comes up.
void SerialBLEInterface::setPinCode(uint32_t pin_code) {
  _pin_code = pin_code;
  if (pServer == NULL) return;
  BLESecurity sec;
  sec.setStaticPIN(_pin_code);
  sec.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
}

// -------- BLESecurityCallbacks methods

uint32_t SerialBLEInterface::onPassKeyRequest() {
  BLE_DEBUG_PRINTLN("onPassKeyRequest()");
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
  return true;  // allow
}

void SerialBLEInterface::onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
  if (cmpl.success) {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Success");
    deviceConnected = true;
  } else {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Failure*");

    //pServer->removePeerDevice(pServer->getConnId(), true);
    pServer->disconnect(pServer->getConnId());
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;
  }
}

// -------- BLEServerCallbacks methods

void SerialBLEInterface::onConnect(BLEServer* pServer) {
}

void SerialBLEInterface::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
  BLE_DEBUG_PRINTLN("onConnect(), conn_id=%d, mtu=%d", param->connect.conn_id, pServer->getPeerMTU(param->connect.conn_id));
  last_conn_id = param->connect.conn_id;
  memcpy(_remote_bda, param->connect.remote_bda, sizeof(_remote_bda));
  s_ble_link_up_cnt++;   // actual GATT link came up (logged from main loop)
}

void SerialBLEInterface::onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
  BLE_DEBUG_PRINTLN("onMtuChanged(), mtu=%d", pServer->getPeerMTU(param->mtu.conn_id));
}

void SerialBLEInterface::onDisconnect(BLEServer* pServer) {
  BLE_DEBUG_PRINTLN("onDisconnect()");
  s_ble_link_down_cnt++;   // actual GATT link dropped (logged from main loop)
  if (_isEnabled) {
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;

    // loop() will detect this on next loop, and set deviceConnected to false
  }
}

// -------- BLECharacteristicCallbacks methods

void SerialBLEInterface::onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
  uint8_t* rxValue = pCharacteristic->getData();
  int len = pCharacteristic->getLength();

  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("ERROR: onWrite(), frame too big, len=%d", len);
  } else if (recv_queue_len >= FRAME_QUEUE_SIZE) {
    BLE_DEBUG_PRINTLN("ERROR: onWrite(), recv_queue is full!");
    _recv_queue_full_count++;
  } else {
    recv_queue[recv_queue_len].len = len;
    memcpy(recv_queue[recv_queue_len].buf, rxValue, len);
    recv_queue_len++;
  }
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
  // pService->start() below would crash (LoadProhibited) on the null pointer.
  // Reproduced on real hardware as a repeatable BLE<->TCP switch reboot
  // (BUGS.md 2026-09-02), the mirror-image case of disable()'s own guard.
  if (pServer == NULL) return;

  _isEnabled = true;
  clearBuffers();

  // Start the service
  pService->start();

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
  // re-enters an already-torn-down BLE stack. pServer->getAdvertising()
  // ->stop() then fails (esp_ble_gap_stop_advertising: rc=259) and
  // pServer->disconnect(last_conn_id) crashes inside esp_ble_gatts_close()
  // (LoadProhibited) -- reproduced on real hardware as a BLE->TCP switch
  // reboot (BUGS.md 2026-09-02). enable() already guards the same way
  // (see its own `if (_isEnabled) return;` above); disable() needs the
  // identical guard.
  if (!_isEnabled) return;
  _isEnabled = false;

  BLE_DEBUG_PRINTLN("SerialBLEInterface::disable");

  pServer->getAdvertising()->stop();
  pServer->disconnect(last_conn_id);
  pService->stop();
  oldDeviceConnected = deviceConnected = false;
  adv_restart_time = 0;
}

// beebo: forcibly evict a central that connected while this transport isn't
// the session owner (MultiSerialInterface's non-owner-eviction pass, see
// plans/TRANSPORT_STATE_MACHINE.md's "Session arbitration state machine").
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

#define  BLE_WRITE_MIN_INTERVAL   60

bool SerialBLEInterface::isWriteBusy() const {
  // beebo: two independent reasons to hold off the producer -- the
  // BLE_WRITE_MIN_INTERVAL timer paces notify() calls to what the BLE stack
  // can issue, and the queue-depth check (same back-pressure as
  // SerialWifiInterface::isWriteBusy()) catches the case where the real
  // link is draining slower than that timer assumes, so a burst (e.g.
  // GET_CONTACTS) doesn't silently overflow the 6-slot send_queue.
  return millis() < _last_write + BLE_WRITE_MIN_INTERVAL   // still too soon to start another write?
    || send_queue_len >= FRAME_QUEUE_SIZE - 1;
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[], size_t max_len) {
  // Drain actual GATT link events captured in the BT-task callbacks into the
  // ring (main-loop context, so the ring stays single-writer).
  static uint8_t seen_up = 0, seen_down = 0;
  // beebo: TLOG_APP_SESSION_START logged here, alongside the link-up event --
  // BLE has no separate app-level accept step the way TCP does (a raw socket
  // accept ahead of any frame), the GATT connection itself IS the session,
  // so this is BLE's equivalent of SerialWifiInterface's own accept-path log
  // (see MultiSerialInterface.h's lockOn() comment for why it's logged here,
  // not there).
  while (seen_up != s_ble_link_up_cnt)   {
    transport_log.log(TLOG_BLE_CONNECT);
    int32_t addr_hi = ((int32_t)_remote_bda[0] << 8) | (int32_t)_remote_bda[1];
    int32_t addr_lo = ((int32_t)_remote_bda[2] << 24) | ((int32_t)_remote_bda[3] << 16)
                     | ((int32_t)_remote_bda[4] << 8) | (int32_t)_remote_bda[5];
    transport_log.log(TLOG_BLE_CLIENT_ADDR_HI, addr_hi);
    transport_log.log(TLOG_BLE_CLIENT_ADDR_LO, addr_lo);
    transport_log.log(TLOG_APP_SESSION_START, TLOG_XPORT_BLE);
    seen_up++;
  }
  while (seen_down != s_ble_link_down_cnt) { transport_log.log(TLOG_BLE_DISCONNECT); seen_down++; }

  if (send_queue_len > 0   // first, check send queue
    && millis() >= _last_write + BLE_WRITE_MIN_INTERVAL    // space the writes apart
  ) {
    _last_write = millis();
    pTxCharacteristic->setValue(send_queue[0].buf, send_queue[0].len);
    pTxCharacteristic->notify();

    BLE_DEBUG_PRINTLN("writeBytes: sz=%d, hdr=%d", (uint32_t)send_queue[0].len, (uint32_t) send_queue[0].buf[0]);

    send_queue_len--;
    for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
      send_queue[i] = send_queue[i + 1];
    }
  }

  if (recv_queue_len > 0) {   // check recv queue
    size_t len = recv_queue[0].len;   // take from top of queue
    memcpy(dest, recv_queue[0].buf, len);

    BLE_DEBUG_PRINTLN("readBytes: sz=%d, hdr=%d", len, (uint32_t) dest[0]);

    recv_queue_len--;
    for (int i = 0; i < recv_queue_len; i++) {   // delete top item from queue
      recv_queue[i] = recv_queue[i + 1];
    }
    return len;
  }

  if (pServer->getConnectedCount() == 0)  deviceConnected = false;

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

void SerialBLEInterface::requestHealthSample() {
  // beebo: gated on the radio being up, not on a central being connected --
  // mirrors TLOG_WIFI_HEALTH's own gating (_wifi_up, not a live app
  // session), so a heap reading is available the moment BLE is turned on,
  // same as WiFi's.
  //
  // RSSI is deliberately NOT read here. esp_ble_gap_read_rssi() is async
  // (result lands on the BT task via a GAP event, no synchronous getter
  // exists on this stack the way WiFi.RSSI() does) -- an earlier version of
  // this function issued that read whenever a central was connected, which
  // raced applyTransportConfig()'s BLE teardown: loopTransports() calls
  // this, then can call ble_interface.disable()+deinitRadio() later in the
  // very same tick when a live BLE->TCP switch is being applied, tearing
  // down the Bluedroid stack while an RSSI read could still be outstanding
  // -- reproduced on real hardware as a hang + watchdog reboot on switching
  // back to TCP after BLE (BUGS.md). Logging heap-only here removes the
  // outstanding HCI command entirely, so there's nothing left to race.
  if (!_isEnabled) return;
  if (millis() - _last_health_sample_ms < BLE_HEALTH_SAMPLE_MS) return;
  _last_health_sample_ms = millis();
  uint16_t heap_kb = (uint16_t)(ESP.getFreeHeap() / 1024);
  int32_t detail = (int32_t)heap_kb | ((int32_t)(uint8_t)BLE_RSSI_UNAVAILABLE << 16);
  transport_log.log(TLOG_BLE_HEALTH, detail);
}
