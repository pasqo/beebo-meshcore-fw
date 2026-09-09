#include "SerialBLEInterface.h"
#include "esp_mac.h"
#include "../DebugRing.h"

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

SerialBLEInterface* SerialBLEInterface::s_instance = nullptr;

// beebo: BLEDevice's custom-gap-handler extension point (see initRadio()'s
// own comment) -- runs on the BT host task, same as onConnect/onDisconnect
// above. Currently only cares about the RSSI-read tracing this exists for;
// every other event is ignored.
void SerialBLEInterface::_gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (event != ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT) return;
  if (s_instance == nullptr) return;
  s_instance->_rssi_read_inflight = false;
  s_instance->_rssi_cache = param->read_rssi_cmpl.rssi;
  RLOGH(RLOG_ID_BLE_RSSI_COMPLETE, (int32_t)param->read_rssi_cmpl.rssi);
}

// beebo: see the header's own comment on _notify_pending -- clears it as
// soon as the BT controller confirms the last notify() was actually
// handed off, so checkRecvFrame() can issue the next queued frame
// immediately instead of waiting out a fixed guessed interval.
void SerialBLEInterface::_gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
  if (s_instance == nullptr) return;

  if (event == ESP_GATTS_CONF_EVT) {
    if (s_instance->pTxCharacteristic == nullptr) return;
    if (param->conf.handle != s_instance->pTxCharacteristic->getHandle()) return;
    s_instance->_notify_pending = false;
    return;
  }

  // beebo: RLOG_ID_BLE_HANDSHAKE's CCCD_WRITE step -- see its own comment
  // in DebugRing.h. BLE2902's value is 2 raw bytes (bit0 = notifications
  // enabled), same layout esp_ble_gatts_send_indicate()'s peer checks
  // against (BLECharacteristic::notify()).
  if (event == ESP_GATTS_WRITE_EVT && !param->write.is_prep) {
    if (s_instance->pTxDescriptor == nullptr) return;
    if (param->write.handle != s_instance->pTxDescriptor->getHandle()) return;
    bool enabled = param->write.len >= 1 && (param->write.value[0] & 0x01);
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_CCCD_WRITE | ((enabled ? 1 : 0) << 8));
  }
}

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

  // beebo: cache this device's own advertised BLE address -- see
  // getLocalAddress()'s own comment for why this isn't logged here
  // directly (Beebo::_checkTransportStateChanges() logs it instead,
  // alongside the rest of the boot-time transport-state dump).
  memcpy(_local_bda, *BLEDevice::getAddress().getNative(), sizeof(_local_bda));

  // beebo: (re)register every initRadio() -- deinitRadio() doesn't reset
  // BLEDevice::m_customGapHandler itself but there's no harm re-arming it.
  // BLEDevice::init() above already registered ITS OWN gap callback
  // (BLEDevice::gapEventHandler) with the IDF via
  // esp_ble_gap_register_callback() -- that's the single global slot the
  // IDF exposes, already owned by BLEDevice for advertising/security/
  // scan/client handling, so calling esp_ble_gap_register_callback()
  // again here directly would silently replace it and break all of that.
  // setCustomGapHandler() is BLEDevice's own supported extension point
  // instead: BLEDevice::gapEventHandler() forwards every event to it
  // after its own switch, so this coexists with security/advertising
  // rather than replacing them. See _gapEventHandler()'s own comment for
  // what this exists for.
  BLEDevice::setCustomGapHandler(_gapEventHandler);
  // beebo: see _gattsEventHandler()'s own comment -- same extension-point
  // pattern as setCustomGapHandler() above, for the GATTS event stream
  // instead of GAP.
  BLEDevice::setCustomGattsHandler(_gattsEventHandler);
  _notify_pending = false;
  BLEDevice::setSecurityCallbacks(this);
  BLEDevice::setMTU(MAX_FRAME_SIZE);

  BLESecurity  sec;
  sec.setStaticPIN(_pin_code);
  // beebo: setStaticPIN() above only sets the INIT key mask
  // (BLESecurity::setInitEncryptionKey(), see its own source) -- the RESP
  // key mask (what key types *we*, the GATT server/responder, offer during
  // bonding key distribution) is left completely unconfigured otherwise.
  // Every official ESP-IDF BLE security example sets both explicitly to
  // the same mask; iOS's SMP implementation is known to be stricter than
  // Android's about the bonding key set actually offered, and confirmed on
  // real hardware (2026-09-09) to cost several seconds of silent stall
  // during the key-exchange step on a *fresh* pair (never on a reconnect,
  // which skips key distribution and reuses the existing LTK) -- see
  // BUGS.md and RLOG_ID_BLE_HANDSHAKE's own comment in DebugRing.h.
  sec.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
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
  // beebo: pure tracing -- see RLOG_ID_BLE_RSSI_TEARDOWN_WHILE_INFLIGHT's
  // own comment in DebugRing.h. Logged, not waited/cancelled on: this is
  // exactly the scenario that used to hang the BT stack (see
  // requestHealthSample()'s own comment) -- capturing it in the ring lets
  // a real hang be correlated after the fact with "yes, a read was
  // actually outstanding right here", without changing teardown behavior
  // at all yet.
  if (_rssi_read_inflight) {
    RLOGH(RLOG_ID_BLE_RSSI_TEARDOWN_WHILE_INFLIGHT);
    // The stack that would have delivered ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT
    // is being torn down below -- that completion is never coming, so clear
    // the flag here rather than leave it stuck true and misreport every
    // later teardown as another mid-read anomaly.
    _rssi_read_inflight = false;
  }
  _rssi_cache = BLE_RSSI_UNAVAILABLE;
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

void SerialBLEInterface::onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
  if (cmpl.success) {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Success");
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_AUTH_COMPLETE | (1 << 8));
    deviceConnected = true;
  } else {
    BLE_DEBUG_PRINTLN(" - SecurityCallback - Authentication Failure*");
    RLOGH(RLOG_ID_BLE_HANDSHAKE, RLOG_ID_BLE_HANDSHAKE_AUTH_COMPLETE | ((int32_t)cmpl.fail_reason << 16));

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
  _notify_pending = false;   // no confirmation is coming for a dead link
  if (_isEnabled) {
    adv_restart_time = millis() + ADVERT_RESTART_DELAY;
  }
}

// -------- BLECharacteristicCallbacks methods

void SerialBLEInterface::onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
  uint8_t* rxValue = pCharacteristic->getData();
  int len = pCharacteristic->getLength();

  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("ERROR: onWrite(), frame too big, len=%d", len);
  } else {
    Frame frame;
    frame.len = len;
    memcpy(frame.buf, rxValue, len);

    if (xQueueSend(recv_queue, &frame, 0) != pdTRUE) {
      BLE_DEBUG_PRINTLN("ERROR: onWrite(), recv_queue is full!");
      _recv_queue_full_count++;
    }
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

bool SerialBLEInterface::isWriteBusy() const {
  // beebo: two independent reasons to hold off the producer -- _notify_pending
  // (real backpressure: a notify() is still in flight, gated on the BT
  // controller's own ESP_GATTS_CONF_EVT rather than a guessed interval --
  // see _gattsEventHandler()'s comment) and the queue-depth check (same
  // back-pressure as SerialWifiInterface::isWriteBusy()) catching the case
  // where the real link drains slower than one frame per confirmation, so
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

  // beebo: safety net only -- a dropped/never-delivered ESP_GATTS_CONF_EVT
  // (e.g. the link died between notify() and confirmation) must not wedge
  // the send queue forever. Real pacing is _notify_pending, cleared by
  // _gattsEventHandler() the moment the controller actually confirms.
  if (_notify_pending && millis() >= _notify_sent_at + _NOTIFY_CONF_TIMEOUT_MS) {
    _notify_pending = false;
  }

  // beebo: BLECharacteristic::notify() (esp32-BLE-Arduino) silently drops
  // the notification with no error we ever see (only an unchecked
  // BLECharacteristicCallbacks::onStatus(ERROR_NOTIFY_DISABLED) callback)
  // whenever the client hasn't yet written the CCCD (0x2902, "enable
  // notifications") descriptor -- see that function's own p2902 check.
  // Right after a *fresh* pairing, iOS completes bonding before it writes
  // that descriptor, so a reply notified out in that window vanishes for
  // good: send_queue_len-- below still fires unconditionally, discarding
  // the frame with nothing to retry it. Confirmed on real hardware
  // (2026-09-09): a fresh-pair BLE connect would get 1-2 early command
  // replies through, then the client's next request never arrived at all
  // (it was still waiting on the reply it never got), stalling until the
  // client's own ~30s timeout disconnected it -- a reconnect using the
  // now-established bond (CCCD already subscribed) never reproduced it.
  // Holding the queue here until the CCCD is actually enabled -- same
  // backpressure shape as _notify_pending -- fixes this without touching
  // notify()'s own silent-drop behavior.
  bool notify_ready = pTxDescriptor != nullptr && pTxDescriptor->getNotifications();
  if (send_queue_len > 0   // first, check send queue
    && !_notify_pending    // wait for the BT controller to confirm the prior notify
    && notify_ready        // wait for the client to actually subscribe (see above)
  ) {
    _last_write = millis();
    _notify_pending = true;
    _notify_sent_at = _last_write;
    pTxCharacteristic->setValue(send_queue[0].buf, send_queue[0].len);
    pTxCharacteristic->notify();

    BLE_DEBUG_PRINTLN("writeBytes: sz=%d, hdr=%d", (uint32_t)send_queue[0].len, (uint32_t) send_queue[0].buf[0]);

    send_queue_len--;
    for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
      send_queue[i] = send_queue[i + 1];
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

// beebo: RSSI-read tracing (RLOG_ID_BLE_RSSI_REQUESTED/_COMPLETE/
// _TEARDOWN_WHILE_INFLIGHT's own comment in DebugRing.h has the full
// story). esp_ble_gap_read_rssi() is async -- an earlier version of
// requestHealthSample() issued it unconditionally whenever a central was
// connected, which raced applyTransportConfig()'s BLE teardown
// (loopTransports() calls requestHealthSample(), then can call
// ble_interface.disable()+deinitRadio() later in the very same tick on a
// live BLE->TCP switch) and reproduced on real hardware as a hang +
// watchdog reboot (BUGS.md). This does NOT fix that race -- it's not
// gated any differently, still fires the read whenever connected -- it
// only adds the bookkeeping (_rssi_read_inflight) needed to observe, via
// the ring, how often a teardown actually lands mid-read in practice,
// before deciding whether/how to build a real fix. Skips issuing a new
// read while one is already outstanding, so at most one is ever in flight.
void SerialBLEInterface::_requestRssiReadIfIdle() {
  if (!deviceConnected || _rssi_read_inflight) return;
  esp_err_t rc = esp_ble_gap_read_rssi(_remote_bda);
  if (rc == ESP_OK) {
    _rssi_read_inflight = true;
    RLOGH(RLOG_ID_BLE_RSSI_REQUESTED);
  }
}

void SerialBLEInterface::requestHealthSample() {
  // beebo: unlike RLOG_ID_WIFI_HEALTH (gated on the radio being up, not a
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
  int32_t detail = (int32_t)heap_kb | ((int32_t)(uint8_t)BLE_RSSI_UNAVAILABLE << 16);
  RLOGL(RLOG_ID_BLE_HEALTH, detail);
  _requestRssiReadIfIdle();
}
