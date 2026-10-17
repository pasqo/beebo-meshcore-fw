#pragma once

#include "../BaseSerialInterface.h"
#include <string.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class SerialBLEInterface : public BaseSerialInterface, BLESecurityCallbacks, BLEServerCallbacks, BLECharacteristicCallbacks {
  BLEServer *pServer;
  BLEService *pService;
  BLECharacteristic * pTxCharacteristic;
  BLECharacteristic * pRxCharacteristic;
  volatile bool _tx_subscribed;   // client wrote the TX CCCD (notifications on); set by onSubscribe()
  bool deviceConnected;
  bool oldDeviceConnected;
  bool _isEnabled;
  uint16_t last_conn_id;
  uint32_t _pin_code;
  unsigned long _last_write;
  unsigned long adv_restart_time;
  char _dev_name[48];   // saved so the radio can be torn down and re-inited
  uint8_t _remote_bda[6];   // connected central's link-layer address, MSB first (onConnect); valid while deviceConnected
  uint8_t _local_bda[6];    // this device's own advertised address, MSB first, cached in initRadio() -- see getLocalAddress()
  unsigned long _last_health_sample_ms;
  static const uint32_t BLE_HEALTH_SAMPLE_MS = 60000;   // same cadence as SerialWifiInterface's WIFI_HEALTH_SAMPLE_MS
  // BLE_RSSI_UNAVAILABLE: reported by getLastRssi() and logged as
  // DLOG_ID_BLE_HEALTH's RSSI whenever no reading exists (no central
  // connected, or the last ble_gap_conn_rssi() failed).
  static const int8_t BLE_RSSI_UNAVAILABLE = 127;
  // beebo: last RSSI reading, cached for GET_PREFS_TLV readback
  // (PREFS_TLV_BLE_RSSI) -- set by _requestRssiReadIfIdle(), cleared to
  // BLE_RSSI_UNAVAILABLE on disconnect/teardown.
  volatile int8_t _rssi_cache;
  // beebo: send-queue backpressure. Frames go out via
  // ble_gatts_notify_custom() directly (not BLECharacteristic::notify(),
  // which discards the return code). NimBLE reports a full controller/mbuf
  // pool as an error return, not as a later event: _notify_pending is true
  // while the last send failed that way, and checkRecvFrame() retries the
  // same frame on its next call.
  volatile bool _notify_pending;

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

  // beebo: reads the connected central's RSSI (synchronous HCI read).
  // Shared by requestHealthSample()'s periodic cadence and
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
  void onAuthenticationComplete(ble_gap_conn_desc* desc, int status) override;

  // BLEServerCallbacks methods
  void onConnect(BLEServer* pServer, ble_gap_conn_desc* desc) override;
  void onMtuChanged(BLEServer* pServer, ble_gap_conn_desc* desc, uint16_t mtu) override;
  void onDisconnect(BLEServer* pServer, ble_gap_conn_desc* desc) override;

  // BLECharacteristicCallbacks methods
  void onWrite(BLECharacteristic* pCharacteristic) override;
  void onSubscribe(BLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override;

public:
  SerialBLEInterface() {
    pServer = NULL;
    pService = NULL;
    pTxCharacteristic = NULL;
    pRxCharacteristic = NULL;
    _tx_subscribed = false;
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
    _rssi_cache = BLE_RSSI_UNAVAILABLE;
    _notify_pending = false;
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
  bool txDrained() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len, RecvFrameType* type) override;

  // beebo: lifetime counts of writeFrame()/onWrite() silently dropping a
  // frame because send_queue/recv_queue (FRAME_QUEUE_SIZE=6) was full.
  uint32_t getSendQueueFullCount() const { return _send_queue_full_count; }
  uint32_t getRecvQueueFullCount() const { return _recv_queue_full_count; }

  // beebo: called from Beebo::loopTransports() every tick; no-ops unless a
  // central is actually connected (see DLOG_ID_BLE_HEALTH's own comment in
  // DebugLog.h), then at most once per BLE_HEALTH_SAMPLE_MS -- logs
  // DLOG_ID_BLE_HEALTH itself and reads the RSSI via _requestRssiReadIfIdle()
  // (result logged separately as RLOG_ID_BLE_RSSI_COMPLETE).
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
