#pragma once

#include "../BaseSerialInterface.h"
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class SerialBLEInterface : public BaseSerialInterface, BLESecurityCallbacks, BLEServerCallbacks, BLECharacteristicCallbacks {
  BLEServer *pServer;
  BLEService *pService;
  BLECharacteristic * pTxCharacteristic;
  BLECharacteristic * pRxCharacteristic;
  BLE2902 * pTxDescriptor;
  bool deviceConnected;
  bool oldDeviceConnected;
  bool _isEnabled;
  uint16_t last_conn_id;
  uint32_t _pin_code;
  unsigned long _last_write;
  unsigned long adv_restart_time;
  char _dev_name[48];   // saved so the radio can be torn down and re-inited
  esp_bd_addr_t _remote_bda;   // connected central's link-layer address (onConnect); valid while deviceConnected
  esp_bd_addr_t _local_bda;    // this device's own advertised address, cached in initRadio() -- see getLocalAddress()
  unsigned long _last_health_sample_ms;
  static const uint32_t BLE_HEALTH_SAMPLE_MS = 60000;   // same cadence as SerialWifiInterface's WIFI_HEALTH_SAMPLE_MS
  // BLE_RSSI_UNAVAILABLE: esp_ble_gap_read_rssi()'s own "couldn't read"
  // value (see ble_read_rssi_cmpl_evt_param's doc comment in
  // esp_gap_ble_api.h) -- reused as DLOG_ID_BLE_HEALTH's logged RSSI
  // whenever no read is currently outstanding (no central connected, or the
  // last request hasn't completed yet).
  static const int8_t BLE_RSSI_UNAVAILABLE = 127;
  // beebo: tracing-only, see RLOG_ID_BLE_RSSI_REQUESTED/_COMPLETE/
  // _TEARDOWN_WHILE_INFLIGHT's own comment in DebugLog.h -- set when
  // requestHealthSample() issues esp_ble_gap_read_rssi(), cleared by
  // _gapEventHandler() on ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT. deinitRadio()
  // checks it purely to log an anomaly if a teardown lands mid-read; it
  // does NOT wait or cancel on it.
  volatile bool _rssi_read_inflight;
  // beebo: last completed RSSI reading, cached for GET_PREFS_TLV readback
  // (PREFS_TLV_BLE_RSSI) -- set by _gapEventHandler() on
  // ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT, cleared to BLE_RSSI_UNAVAILABLE on
  // disconnect/teardown (no central connected, or a read never completed).
  volatile int8_t _rssi_cache;
  // beebo: only one SerialBLEInterface instance ever exists (ble_interface,
  // main.cpp) -- needed so the static GAP callback (esp_ble_gap_register_callback()
  // takes a plain C function pointer, no user-data param) can reach back into
  // instance state.
  static SerialBLEInterface* s_instance;
  static void _gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
  // beebo: real send-queue backpressure -- replaces the fixed
  // BLE_WRITE_MIN_INTERVAL guess with the BT controller's own
  // ESP_GATTS_CONF_EVT (fires once a queued notify()/indicate() has
  // actually been handed off/drained by the controller, not on a timer).
  // Registered via BLEDevice::setCustomGattsHandler(), the same
  // already-established extension-point pattern _gapEventHandler() uses
  // for setCustomGapHandler() -- runs on the BT host task, alongside
  // onConnect/onDisconnect. See checkRecvFrame()'s send-queue drain.
  static void _gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);
  // beebo: true from the moment a notify() is issued until either its
  // ESP_GATTS_CONF_EVT arrives or _NOTIFY_CONF_TIMEOUT_MS elapses (a
  // dropped/never-delivered confirmation must not wedge the send queue
  // forever) -- see checkRecvFrame()/_gattsEventHandler().
  volatile bool _notify_pending;
  unsigned long _notify_sent_at;
  static const uint32_t _NOTIFY_CONF_TIMEOUT_MS = 500;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  // beebo: bumped 4->6 -- see SerialWifiInterface.h's matching comment.
  #define FRAME_QUEUE_SIZE  6
  // beebo: recv_queue is pushed to from onWrite() (BT host task) and popped
  // from checkRecvFrame() (main loop task) -- a plain array + int length
  // shared across two FreeRTOS tasks with no lock is a real race (torn
  // writes/lost increments on concurrent push+pop), fixed with a proper
  // FreeRTOS queue instead of the raw array. xQueueCreateStatic avoids heap allocation
  // (storage is a plain member array). send_queue stays a plain array:
  // upstream never made it thread-safe either -- only checkRecvFrame()
  // (main loop task) ever touches it, no concurrent-task access exists.
  StaticQueue_t recv_queue_state;
  uint8_t recv_queue_storage[FRAME_QUEUE_SIZE * sizeof(Frame)];
  QueueHandle_t recv_queue;
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];
  uint32_t _send_queue_full_count = 0;
  uint32_t _recv_queue_full_count = 0;

  void clearBuffers() { xQueueReset(recv_queue); send_queue_len = 0; _notify_pending = false; }

  // beebo: shared by requestHealthSample()'s periodic cadence and
  // checkRecvFrame()'s connect-transition -- see requestHealthSample()'s
  // own comment for why the latter needs its own immediate call: a BLE
  // connection this short-lived (a single non-interactive CLI command)
  // can end before the periodic cadence -- last satisfied while nobody
  // was even connected yet, during advertising -- comes around again.
  void _requestRssiReadIfIdle();

protected:
  // BLESecurityCallbacks methods
  uint32_t onPassKeyRequest() override;
  void onPassKeyNotify(uint32_t pass_key) override;
  bool onConfirmPIN(uint32_t pass_key) override;
  bool onSecurityRequest() override;
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override;

  // BLEServerCallbacks methods
  void onConnect(BLEServer* pServer) override;
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override;
  void onMtuChanged(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override;
  void onDisconnect(BLEServer* pServer) override;

  // BLECharacteristicCallbacks methods
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) override;

public:
  SerialBLEInterface() {
    pServer = NULL;
    pService = NULL;
    pTxCharacteristic = NULL;
    pRxCharacteristic = NULL;
    pTxDescriptor = NULL;
    deviceConnected = false;
    oldDeviceConnected = false;
    adv_restart_time = 0;
    _isEnabled = false;
    _last_write = 0;
    last_conn_id = 0;
    recv_queue = xQueueCreateStatic(
      FRAME_QUEUE_SIZE, sizeof(Frame), recv_queue_storage, &recv_queue_state
    );
    send_queue_len = 0;
    _last_health_sample_ms = 0;
    memset(_remote_bda, 0, sizeof(_remote_bda));
    memset(_local_bda, 0, sizeof(_local_bda));
    _rssi_read_inflight = false;
    _rssi_cache = BLE_RSSI_UNAVAILABLE;
    _notify_pending = false;
    _notify_sent_at = 0;
    s_instance = this;
  }

  /**
   * init the BLE interface.
   * @param prefix   a prefix for the device name
   * @param name  IN/OUT - a name for the device (combined with prefix). If "@@MAC", is modified and returned
   * @param pin_code   the BLE security pin
   */
  void begin(const char* prefix, char* name, uint32_t pin_code);

  // Change the pairing PIN of a running (or not-yet-started) stack, so a PIN
  // change doesn't need a reboot. Applies to future pairings only -- existing
  // bonds are unaffected. See the definition for the auth-mode caveat.
  void setPinCode(uint32_t pin_code);

  // Builds the BLE stack + GATT server. Factored out of begin() (begin() is
  // just device-name setup followed by initRadio()). Safe to call again after
  // deinitRadio() (re-uses the device name saved in begin()).
  void initRadio();

  // Tears down the BLE stack (NimBLE host + controller) built by initRadio(),
  // fully powering down the BLE radio and freeing its RAM. Call disable()
  // first to stop advertising/drop any link. Call initRadio() again to bring
  // BLE back up.
  void deinitRadio();

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  // beebo: this device's own advertised BLE address, cached by initRadio()
  // -- see RLOG_ID_BLE_LOCAL_ADDR_HI/LO's own comment in DebugLog.h.
  // Beebo::_checkTransportStateChanges() calls this (not initRadio()
  // itself) right when RLOG_ID_XPORT_LINK_BLE_IFACE_ENABLED transitions
  // on, so the address lands in the ring alongside the rest of the
  // boot-time transport-state dump instead of its own separate,
  // out-of-order timestamp. Only valid once initRadio() has actually run
  // (i.e. once isEnabled() is true) -- zeroed otherwise.
  const uint8_t* getLocalAddress() const { return _local_bda; }
  void resetParserState() override;

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len, RecvFrameType* type) override;

  // beebo: lifetime counts of writeFrame()/onWrite() silently dropping a
  // frame because send_queue/recv_queue (FRAME_QUEUE_SIZE=6) was full.
  uint32_t getSendQueueFullCount() const { return _send_queue_full_count; }
  uint32_t getRecvQueueFullCount() const { return _recv_queue_full_count; }

  // beebo: called from Beebo::loopTransports() every tick; no-ops unless a
  // central is actually connected (see DLOG_ID_BLE_HEALTH's own comment in
  // DebugLog.h), then at most once per BLE_HEALTH_SAMPLE_MS -- logs
  // DLOG_ID_BLE_HEALTH itself and kicks off the async RSSI read via
  // _requestRssiReadIfIdle(); the read's own result is logged separately,
  // later, once it actually arrives (_gapEventHandler(), RLOG_ID_BLE_RSSI_COMPLETE).
  void requestHealthSample();

  // beebo: last completed RSSI reading (BLE_RSSI_UNAVAILABLE if none --
  // no central connected, or a read is still outstanding/never
  // completed). Backs node.ble.rssi (PREFS_TLV_BLE_RSSI, read-only).
  int8_t getLastRssi() const { return _rssi_cache; }
};

#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define BLE_DEBUG_PRINT(F, ...) Serial.printf("BLE: " F, ##__VA_ARGS__)
  #define BLE_DEBUG_PRINTLN(F, ...) Serial.printf("BLE: " F "\n", ##__VA_ARGS__)
#else
  #define BLE_DEBUG_PRINT(...) {}
  #define BLE_DEBUG_PRINTLN(...) {}
#endif
