#pragma once

#include "../BaseSerialInterface.h"
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
  unsigned long _last_health_sample_ms;
  static const uint32_t BLE_HEALTH_SAMPLE_MS = 3000;   // same cadence as SerialWifiInterface's WIFI_HEALTH_SAMPLE_MS
  // BLE RSSI is never actually read (see requestHealthSample()'s own
  // comment for why) -- RLOG_ID_BLE_HEALTH always logs this sentinel for it,
  // esp_ble_gap_read_rssi()'s own "couldn't read" value (see
  // ble_read_rssi_cmpl_evt_param's doc comment in esp_gap_ble_api.h),
  // reused here for "not read at all".
  static const int8_t BLE_RSSI_UNAVAILABLE = 127;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  // beebo: bumped 4->6 -- see SerialWifiInterface.h's matching comment.
  #define FRAME_QUEUE_SIZE  6
  int recv_queue_len;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];
  uint32_t _send_queue_full_count = 0;
  uint32_t _recv_queue_full_count = 0;

  void clearBuffers() { recv_queue_len = 0; send_queue_len = 0; }

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
    send_queue_len = recv_queue_len = 0;
    _last_health_sample_ms = 0;
    memset(_remote_bda, 0, sizeof(_remote_bda));
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
  void resetParserState() override;

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[], size_t max_len) override;

  // beebo: lifetime counts of writeFrame()/onWrite() silently dropping a
  // frame because send_queue/recv_queue (FRAME_QUEUE_SIZE=6) was full.
  uint32_t getSendQueueFullCount() const { return _send_queue_full_count; }
  uint32_t getRecvQueueFullCount() const { return _recv_queue_full_count; }

  // beebo: called from Beebo::loopTransports() every BLE_HEALTH_SAMPLE_MS
  // while connected -- kicks off the async RSSI read; RLOG_ID_BLE_HEALTH is
  // logged later from _gapEventHandler() once the result actually arrives.
  void requestHealthSample();
};

#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define BLE_DEBUG_PRINT(F, ...) Serial.printf("BLE: " F, ##__VA_ARGS__)
  #define BLE_DEBUG_PRINTLN(F, ...) Serial.printf("BLE: " F "\n", ##__VA_ARGS__)
#else
  #define BLE_DEBUG_PRINT(...) {}
  #define BLE_DEBUG_PRINTLN(...) {}
#endif
