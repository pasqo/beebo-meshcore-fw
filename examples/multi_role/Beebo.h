#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <helpers/MonRing.h>
#include <helpers/TuneController.h>
#include <esp_ota_ops.h>

/*------------ Frame Protocol --------------*/
#define FIRMWARE_VER_CODE 14

#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE "6 Jun 2026"
#endif

#if __has_include("BeeboVersion.h")
  #include "BeeboVersion.h"  // beebo: generated from BEEBO_FW_VERSION, see beebo/gen_version_header.py
#endif
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v1.16.0"
#endif

#include <SPIFFS.h>

#include "DataStore.h"
#include "../companion_radio/NodePrefs.h"
#include "BeeboCompanionPrefs.h"
#include "BeeboRepeaterPrefs.h"

// beebo: CommonCLI.h's real struct is named NodePrefs upstream (persisted to
// /com_prefs -- a naming/filename mismatch predating beebo), which collides
// with the NodePrefs already included above (companion's own,
// /beebo_companion). Rename it to ComPrefs for the scope of this one
// #include only -- CommonCLI.h itself is never modified, so every other
// consumer (simple_repeater, simple_room_server, simple_sensor,
// BridgeBase.h) is completely unaffected regardless of build flags. See
// beebo/plans/COMMONCLI_TEXT_DISPATCH.md's "ComPrefs rename mechanism" for
// why the rename lives here and not behind a build flag inside CommonCLI.h
// (a first attempt keyed off -D BEEBO_BUILD broke BridgeBase.h, which also
// includes CommonCLI.h in every multi_role-env translation unit).
#define NodePrefs ComPrefs
#include <helpers/CommonCLI.h>
#undef NodePrefs

#include <RTClib.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/BaseSerialInterface.h>
#include <helpers/esp32/SerialBLEInterface.h>
#include <helpers/esp32/SerialWifiInterface.h>
#include <helpers/MultiSerialInterface.h>
#include <helpers/DualModeSerialInterface.h> // beebo: multi_role Phase 5 -- dual-mode, not plain ArduinoSerialInterface (see usb_interface's own comment)
#include <helpers/ClientACL.h> // beebo: multi_role Phase 3 -- repeater-role admin login table
#include <helpers/IdentityStore.h>
#include <helpers/RegionMap.h> // beebo: multi_role Phase 3 -- repeater-role named-region registry, queried by handleAnonRegionsReq
#include "RateLimiter.h" // beebo: multi_role Phase 3 -- repeater-role anon-request rate limiting (copy of examples/simple_repeater's, not shared infra elsewhere)
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <target.h>

/* ---------------------------------- CONFIGURATION ------------------------------------- */

// beebo: STATIC_ROLE_BUILDS Phase 0 -- gate companion/repeater role support at compile
// time. Both default on (matching today's hot-switchable multi_role behavior); a static
// role build defines the unwanted one to 0 in its platformio.ini env.
#ifndef BEEBO_ENABLE_COMPANION_ROLE
#define BEEBO_ENABLE_COMPANION_ROLE 1
#endif
#ifndef BEEBO_ENABLE_REPEATER_ROLE
#define BEEBO_ENABLE_REPEATER_ROLE 1
#endif

#ifndef LORA_FREQ
#define LORA_FREQ 915.0
#endif
#ifndef LORA_BW
#define LORA_BW 250
#endif
#ifndef LORA_SF
#define LORA_SF 10
#endif
#ifndef LORA_CR
#define LORA_CR 5
#endif
#ifndef LORA_TX_POWER
#define LORA_TX_POWER 20
#endif
#ifndef MAX_LORA_TX_POWER
#define MAX_LORA_TX_POWER LORA_TX_POWER
#endif

#ifndef MAX_CONTACTS
#define MAX_CONTACTS 100
#endif

#ifndef OFFLINE_QUEUE_SIZE
#define OFFLINE_QUEUE_SIZE 16
#endif

#ifndef BLE_NAME_PREFIX
#define BLE_NAME_PREFIX "MeshCore-"
#endif

#include <helpers/BaseChatMesh.h>
#include <helpers/TransportKeyStore.h>

/* -------------------------------------------------------------------------------------- */

#define REQ_TYPE_GET_STATUS             0x01 // same as _GET_STATS
#define REQ_TYPE_KEEP_ALIVE             0x02
#define REQ_TYPE_GET_TELEMETRY_DATA     0x03

// beebo: CMD_BEEBO wire-protocol constants, hoisted here (out of Beebo.cpp)
// so both Beebo.cpp (shared dispatch/plumbing) and BeeboCompanion.cpp (the
// BaseChatMesh-contract overrides) can see them -- each .cpp is its own
// translation unit, so a #define local to one is invisible to the other.
#define CMD_APP_START                 1
#define CMD_SEND_TXT_MSG              2
#define CMD_SEND_CHANNEL_TXT_MSG      3
#define CMD_GET_CONTACTS              4 // with optional 'since' (for efficient sync)
#define CMD_GET_DEVICE_TIME           5
#define CMD_SET_DEVICE_TIME           6
#define CMD_SEND_SELF_ADVERT          7
#define CMD_SET_ADVERT_NAME           8
#define CMD_ADD_UPDATE_CONTACT        9
#define CMD_SYNC_NEXT_MESSAGE         10
#define CMD_SET_RADIO_PARAMS          11
#define CMD_SET_RADIO_TX_POWER        12
#define CMD_RESET_PATH                13
#define CMD_SET_ADVERT_LATLON         14
#define CMD_REMOVE_CONTACT            15
#define CMD_SHARE_CONTACT             16
#define CMD_EXPORT_CONTACT            17
#define CMD_IMPORT_CONTACT            18
#define CMD_REBOOT                    19
#define CMD_GET_BATT_AND_STORAGE      20   // was CMD_GET_BATTERY_VOLTAGE
#define CMD_SET_TUNING_PARAMS         21
#define CMD_DEVICE_QUERY              22
#define CMD_EXPORT_PRIVATE_KEY        23
#define CMD_IMPORT_PRIVATE_KEY        24
#define CMD_SEND_RAW_DATA             25
#define CMD_SEND_LOGIN                26
#define CMD_SEND_STATUS_REQ           27
#define CMD_HAS_CONNECTION            28
#define CMD_LOGOUT                    29 // 'Disconnect'
#define CMD_GET_CONTACT_BY_KEY        30
#define CMD_GET_CHANNEL               31
#define CMD_SET_CHANNEL               32
#define CMD_SIGN_START                33
#define CMD_SIGN_DATA                 34
#define CMD_SIGN_FINISH               35
#define CMD_SEND_TRACE_PATH           36
#define CMD_SET_DEVICE_PIN            37
#define CMD_SET_OTHER_PARAMS          38
#define CMD_SEND_TELEMETRY_REQ        39  // can deprecate this
#define CMD_GET_CUSTOM_VARS           40
#define CMD_SET_CUSTOM_VAR            41
#define CMD_GET_ADVERT_PATH           42
#define CMD_GET_TUNING_PARAMS         43
#define CMD_SEND_BINARY_REQ           50
#define CMD_FACTORY_RESET             51
#define CMD_SEND_PATH_DISCOVERY_REQ   52
#define CMD_SET_FLOOD_SCOPE_KEY       54   // v8+
#define CMD_SEND_CONTROL_DATA         55   // v8+
#define CMD_GET_STATS                 56   // v8+, second byte is stats type
#define CMD_SEND_ANON_REQ             57
#define CMD_SET_AUTOADD_CONFIG        58
#define CMD_GET_AUTOADD_CONFIG        59
#define CMD_GET_ALLOWED_REPEAT_FREQ   60
#define CMD_SET_PATH_HASH_MODE        61
#define CMD_SEND_CHANNEL_DATA         62
#define CMD_SET_DEFAULT_FLOOD_SCOPE   63
#define CMD_GET_DEFAULT_FLOOD_SCOPE   64
#define CMD_SEND_RAW_PACKET           65

// Stats sub-types for CMD_GET_STATS
#define STATS_TYPE_CORE               0
#define STATS_TYPE_RADIO              1
#define STATS_TYPE_PACKETS             2
#define STATS_TYPE_SYSTEM             3   // beebo: free heap, free PSRAM, flash size, MCU temp
#define STATS_TYPE_TRANSPORT          4   // beebo: transport event ring buffer
#define STATS_TYPE_PROFILE            5   // beebo: command-latency profiling ring buffer

#define RESP_CODE_OK                  0
#define RESP_CODE_ERR                 1
#define RESP_CODE_CONTACTS_START      2  // first reply to CMD_GET_CONTACTS
#define RESP_CODE_CONTACT             3  // multiple of these (after CMD_GET_CONTACTS)
#define RESP_CODE_END_OF_CONTACTS     4  // last reply to CMD_GET_CONTACTS
#define RESP_CODE_SELF_INFO           5  // reply to CMD_APP_START
#define RESP_CODE_SENT                6  // reply to CMD_SEND_TXT_MSG
#define RESP_CODE_CONTACT_MSG_RECV    7  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CHANNEL_MSG_RECV    8  // a reply to CMD_SYNC_NEXT_MESSAGE (ver < 3)
#define RESP_CODE_CURR_TIME           9  // a reply to CMD_GET_DEVICE_TIME
#define RESP_CODE_NO_MORE_MESSAGES    10 // a reply to CMD_SYNC_NEXT_MESSAGE
#define RESP_CODE_EXPORT_CONTACT      11
#define RESP_CODE_BATT_AND_STORAGE    12 // a reply to a CMD_GET_BATT_AND_STORAGE
#define RESP_CODE_DEVICE_INFO         13 // a reply to CMD_DEVICE_QUERY
#define RESP_CODE_PRIVATE_KEY         14 // a reply to CMD_EXPORT_PRIVATE_KEY
#define RESP_CODE_DISABLED            15
#define RESP_CODE_CONTACT_MSG_RECV_V3 16 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_MSG_RECV_V3 17 // a reply to CMD_SYNC_NEXT_MESSAGE (ver >= 3)
#define RESP_CODE_CHANNEL_INFO        18 // a reply to CMD_GET_CHANNEL
#define RESP_CODE_SIGN_START          19
#define RESP_CODE_SIGNATURE           20
#define RESP_CODE_CUSTOM_VARS         21
#define RESP_CODE_ADVERT_PATH         22
#define RESP_CODE_TUNING_PARAMS       23
#define RESP_CODE_STATS               24   // v8+, second byte is stats type
#define RESP_CODE_AUTOADD_CONFIG      25
#define RESP_ALLOWED_REPEAT_FREQ      26
#define RESP_CODE_CHANNEL_DATA_RECV   27
#define RESP_CODE_DEFAULT_FLOOD_SCOPE 28

#define MAX_CHANNEL_DATA_LENGTH       (MAX_FRAME_SIZE - 9)

#define SEND_TIMEOUT_BASE_MILLIS        500
#define FLOOD_SEND_TIMEOUT_FACTOR       16.0f
#define DIRECT_SEND_PERHOP_FACTOR       6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS 250
#define LAZY_CONTACTS_WRITE_DELAY       5000

// beebo: multi_role Phase 3 -- repeater-role remote-admin-over-mesh protocol
// constants, ported unchanged from examples/simple_repeater/MyMesh.cpp so
// existing remote-admin clients (meshcli, phone app remote-admin) see
// identical wire behaviour from a repeater-role multi_role node.
// REQ_TYPE_GET_STATUS/KEEP_ALIVE/GET_TELEMETRY_DATA already declared above
// (companion's own client-side "query a remote repeater" code already uses
// these exact values) -- reused as-is, not redefined.
#define FIRMWARE_VER_LEVEL              2

#define REQ_TYPE_GET_ACCESS_LIST        0x05
#define REQ_TYPE_GET_NEIGHBOURS         0x06
#define REQ_TYPE_GET_OWNER_INFO         0x07     // FIRMWARE_VER_LEVEL >= 2

#define RESP_SERVER_LOGIN_OK            0 // response to ANON_REQ

#define ANON_REQ_TYPE_REGIONS           0x01
#define ANON_REQ_TYPE_OWNER             0x02
#define ANON_REQ_TYPE_BASIC             0x03   // just remote clock

#define ADMIN_REQ_SERVER_RESPONSE_DELAY 300
// beebo: multi_role Phase 6 -- mesh-transport text command reply delays,
// same values as examples/simple_repeater/MyMesh.cpp.
#define TXT_ACK_DELAY                   200
#define CLI_REPLY_DELAY_MILLIS          600

// beebo: multi_role repeater-role forwarding gate (allowPacketForward) --
// same values as src/helpers/CommonCLI.h's LOOP_DETECT_* (not included
// directly here, same ODR-avoidance rationale as readComPrefsField's own
// comment: CommonCLI.h's NodePrefs struct clashes with companion's own).
#define LOOP_DETECT_OFF                 0
#define LOOP_DETECT_MINIMAL             1
#define LOOP_DETECT_MODERATE            2
#define LOOP_DETECT_STRICT              3

// beebo: found in the multi_role event-loop review -- repeater-role "node
// discover" protocol (same values as examples/simple_repeater/MyMesh.cpp),
// entirely absent before. See onControlDataRecv/sendNodeDiscoverReq.
#define CTL_TYPE_NODE_DISCOVER_REQ      0x80
#define CTL_TYPE_NODE_DISCOVER_RESP     0x90

#define PUBLIC_GROUP_PSK                "izOH6cXN6mrJ5e26oRXNcg=="

// these are _pushed_ to client app at any time
#define PUSH_CODE_ADVERT                0x80
#define PUSH_CODE_PATH_UPDATED          0x81
#define PUSH_CODE_SEND_CONFIRMED        0x82
#define PUSH_CODE_MSG_WAITING           0x83
#define PUSH_CODE_RAW_DATA              0x84
#define PUSH_CODE_LOGIN_SUCCESS         0x85
#define PUSH_CODE_LOGIN_FAIL            0x86
#define PUSH_CODE_STATUS_RESPONSE       0x87
#define PUSH_CODE_LOG_RX_DATA           0x88
#define PUSH_CODE_TRACE_DATA            0x89
#define PUSH_CODE_NEW_ADVERT            0x8A
#define PUSH_CODE_TELEMETRY_RESPONSE    0x8B
#define PUSH_CODE_BINARY_RESPONSE       0x8C
#define PUSH_CODE_PATH_DISCOVERY_RESPONSE 0x8D
#define PUSH_CODE_CONTROL_DATA          0x8E   // v8+
#define PUSH_CODE_CONTACT_DELETED       0x8F // used to notify client app of deleted contact when overwriting oldest
#define PUSH_CODE_CONTACTS_FULL         0x90 // used to notify client app that contacts storage is full
#define PUSH_CODE_POKE_REPLY            0x91 // beebo: reply to CMD_SEND_POKE, correlated by tag

#define ERR_CODE_UNSUPPORTED_CMD        1
#define ERR_CODE_NOT_FOUND              2
#define ERR_CODE_TABLE_FULL             3
#define ERR_CODE_BAD_STATE              4
#define ERR_CODE_FILE_IO_ERROR          5
#define ERR_CODE_ILLEGAL_ARG            6

#define MAX_SIGN_DATA_LEN               (8 * 1024) // 8K

// Auto-add config bitmask
// Bit 0: If set, overwrite oldest non-favourite contact when contacts file is full
// Bits 1-4: these indicate which contact types to auto-add when manual_contact_mode = 0x01
#define AUTO_ADD_OVERWRITE_OLDEST (1 << 0)  // 0x01 - overwrite oldest non-favourite when full
#define AUTO_ADD_CHAT             (1 << 1)  // 0x02 - auto-add Chat (Companion) (ADV_TYPE_CHAT)
#define AUTO_ADD_REPEATER         (1 << 2)  // 0x04 - auto-add Repeater (ADV_TYPE_REPEATER)
#define AUTO_ADD_ROOM_SERVER      (1 << 3)  // 0x08 - auto-add Room Server (ADV_TYPE_ROOM)
#define AUTO_ADD_SENSOR           (1 << 4)  // 0x10 - auto-add Sensor (ADV_TYPE_SENSOR)

struct AdvertPath {
  uint8_t pubkey_prefix[7];
  uint8_t path_len;
  char    name[32];
  uint32_t recv_timestamp;
  uint8_t path[MAX_PATH_SIZE];
};

// beebo: direct (zero-hop) neighbours heard via ANY zero-hop RX that carries a
// resolvable sender prefix — an advert (full 32-byte pubkey) or a control/
// discover response (1-32 byte pubkey prefix, per the requester's --full
// flag). Packet types that carry no identity at zero-hop (ACK: none at all;
// TXT/PATH/REQ/RESPONSE: always just a 1-byte src_hash, no better even with
// --full) are NOT recorded — there is nothing to key a neighbour on. Auto-add
// is usually off (too many nodes), so this RAM-only table is how an immediate
// neighbour that isn't a saved contact can still be identified and, if
// wanted, promoted (promotion needs the full 32-byte key, so pubkey_len < 32
// entries can't be promoted directly). Carries name + location when known
// (from an advert only); SNR is stored x4 (divide by 4 for dB), matching the
// repeater convention.
//
// Prefix matching: since a control/discover response may carry as little as
// 1 byte of pubkey, two distinct nodes can share a stored prefix. We treat a
// new sighting as the SAME node only if the prefix bytes match over the
// shorter of the two lengths AND the SNR hasn't drifted more than
// NEIGHBOUR_SNR_DRIFT (independent radio links land at different SNRs), else
// it's treated as a different node and evicts the oldest slot instead. A
// longer prefix seen later (e.g. an advert following a 1-byte discover hit)
// upgrades the stored prefix in place.
#ifndef MAX_NEIGHBOURS
#define MAX_NEIGHBOURS 16
#endif

#define NEIGHBOUR_SNR_DRIFT  24   // x4 => 6dB; beyond this, same prefix is treated as a different node

struct NeighbourInfo {
  uint8_t  pubkey[PUB_KEY_SIZE];   // prefix, right-padded with zeroes past pubkey_len
  uint8_t  pubkey_len;             // valid prefix length in bytes (0 = empty slot)
  uint32_t advert_timestamp;   // by THEIR clock (from the advert, 0 if never adverted)
  uint32_t heard_timestamp;    // by OUR clock (0 = empty slot)
  int8_t   snr;                // x4
  uint8_t  type;               // ADV_TYPE_* (chat/repeater/room/sensor), 0xFF if unknown
  int32_t  lat, lon;           // 1e6 fixed-point, 0 if never adverted / no location
  char     name[32];           // empty if never adverted
};

// beebo: multi_role Phase 3 -- wire-compatible with examples/simple_repeater's
// REQ_TYPE_GET_STATUS reply (RepeaterStats, Beebo.h:49-65 there), so existing
// remote-admin clients (meshcli, the phone app's remote-admin view) see the
// same reply shape from a repeater-role multi_role node. Values are filled
// from the same underlying getters companion's own CMD_BEEBO stats handlers
// already use (radio_driver, _mgr, _radio, getTotalAirTime(), etc.) -- not a
// separate/duplicated stats-gathering path.
struct RepeaterStats {
  uint16_t batt_milli_volts;
  uint16_t curr_tx_queue_len;
  int16_t  noise_floor;
  int16_t  last_rssi;
  uint32_t n_packets_recv;
  uint32_t n_packets_sent;
  uint32_t total_air_time_secs;
  uint32_t total_up_time_secs;
  uint32_t n_sent_flood, n_sent_direct;
  uint32_t n_recv_flood, n_recv_direct;
  uint16_t err_events;
  int16_t  last_snr;   // x 4
  uint16_t n_direct_dups, n_flood_dups;
  uint32_t total_rx_air_time_secs;
  uint32_t n_recv_errors;
};

class Beebo : public BaseChatMesh, public DataStoreHost
#if BEEBO_ENABLE_REPEATER_ROLE
    , public CommonCLICallbacks   // beebo: CommonCLI (repeater-only) needs this; companion-only static builds skip it entirely
#endif
{
public:
  Beebo(mesh::Radio &radio, mesh::RNG &rng, mesh::RTCClock &rtc, SimpleMeshTables &tables, DataStore& store);

  void begin();
  void initMonRing();  // beebo: allocate the monitoring capture ring (call last in setup())
  void startInterface(BaseSerialInterface &serial);

  // beebo: build one RESP_CODE_MONRING frame (status header + up to max_len of
  // records, oldest-first from after_seq) into `out`. Returns frame length and
  // sets *returned to the record count (0 = nothing at/above after_seq). Shared
  // by the paged handler and the BULK_XFER streaming pump. `first_page` gates
  // splicing in the start references (emitStartRefs()) ahead of the real
  // records. `reset` is written into the header as-is (see
  // BEEBO_CMD_GET_MONRING) -- this function doesn't decide it, only reports
  // whatever the caller already determined.
  int fillMonRingFrame(uint8_t *out, uint32_t after_seq, size_t max_len, uint32_t *returned, bool first_page, bool reset);

  const char *getNodeName();
  NodePrefs *getNodePrefs();
  uint32_t getBLEPin();

  void loop();
  void handleCmdFrame(size_t len);
  bool advert();

  // beebo: multi_role Phase 5 text-CLI back-compat dispatcher, now called
  // from two places: locally over USB (DualModeSerialInterface, see
  // main.cpp) with sender_timestamp=0, and (Phase 6) from the mesh RX path
  // in onPeerDataRecv's PAYLOAD_TYPE_TXT_MSG branch with a real timestamp
  // from an authenticated admin ACL client -- same pattern
  // examples/simple_repeater/MyMesh.cpp already uses this exact function
  // for (three call sites funnelling into one dispatcher, not a second one
  // per transport). NOT a port of src/helpers/CommonCLI.cpp's full
  // ~50-command table: that class declares its own incompatible global
  // NodePrefs (ODR clash with companion's), and MULTI_ROLE_FW.md's Phase 5
  // section calls for a fresh, minimal dispatcher using beebo's own dotted
  // key names (node.role, ...) as the canonical key space, not
  // CommonCLI's flat legacy names -- except where a real external tool
  // (meshcli) already expects a specific flat name (e.g. "name"), kept
  // as-is for genuine back-compat. Reads/writes companion's own NodePrefs
  // fields directly. Every command here becomes mesh-reachable to a
  // logged-in admin client too (Phase 6 gates on client->isAdmin(), not a
  // separate allowlist) -- keep that in mind before adding anything with a
  // large payload or destructive blast radius (OTA, prefs bulk-write) to
  // this dispatcher; mesh bandwidth and multi-hop reliability don't suit
  // them (see MULTI_ROLE_FW.md Phase 6).
  void handleCommand(uint32_t sender_timestamp, char* command, char* reply);

  // beebo: multi_role Phase 4 -- repeater-role advert, mirroring
  // BaseChatMesh::createSelfAdvert (src/helpers/BaseChatMesh.cpp:19-39,
  // shared library code, not editable in place) but with ADV_TYPE_REPEATER
  // instead of the hardcoded ADV_TYPE_CHAT. advert() branches on
  // _beebo_companion.node_role between the two -- live/hot, no reboot needed (see
  // MULTI_ROLE_FW.md Phase 4).
#if BEEBO_ENABLE_REPEATER_ROLE
  mesh::Packet* createRepeaterSelfAdvert(const char* name);
  mesh::Packet* createRepeaterSelfAdvert(const char* name, double lat, double lon);
#endif
  // beebo: role + advert_loc_policy branch shared by advert() and loop()'s
  // periodic flood-advert scheduling (see that method's own comment) --
  // factored out so both build the packet identically.
  mesh::Packet* createSelfAdvertPacket();

  int  getRecentlyHeard(AdvertPath dest[], int max_num);

#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: multi_role Phase 3 item 2 -- repeater-role inbound login/ACL
  // request handler, ported from examples/simple_repeater/MyMesh.cpp.
  // Reached only when _beebo_companion.node_role == NODE_ROLE_REPEATER (see the
  // onAnonDataRecv/onPeerDataRecv overrides below); a companion-role node
  // never calls any of these. Text-CLI-over-mesh (TXT_TYPE_CLI_DATA) is
  // deliberately excluded -- that's Phase 5/6, once the text dispatcher
  // exists at all in multi_role.
  uint8_t handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data, bool is_flood);
  uint8_t handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
  uint8_t handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
  uint8_t handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data);
  int handleRequest(ClientInfo* sender, uint32_t sender_timestamp, uint8_t* payload, size_t payload_len);
#endif
  void sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size);

protected:
  float getAirtimeBudgetFactor() const override;
  int getInterferenceThreshold() const override;
  // beebo: found in the multi_role event-loop review -- Dispatcher's default
  // (0, disabled) was never overridden here at all, for either role, so
  // repeater.agc.reset.interval was stored/read-back correctly but never
  // actually armed the periodic AGC-reset in Dispatcher::loop(). Ported from
  // simple_repeater's own override (MyMesh.h:231-233): repeater role uses
  // the RAM-cached com_prefs.agc_reset_interval; companion keeps the prior
  // implicit default (0, disabled -- companion never had this pref at all).
  int getAGCResetInterval() const override;
  int calcRxDelay(float score, uint32_t air_time) const override;
  uint32_t getRetransmitDelay(const mesh::Packet *packet) override;
  uint32_t getDirectRetransmitDelay(const mesh::Packet *packet) override;
  uint8_t getExtraAckTransmitCount() const override;
  bool filterRecvFloodPacket(mesh::Packet* packet) override;
  bool allowPacketForward(const mesh::Packet* packet) override;

  // beebo: multi_role Phase 3 -- re-overridden (companion gets these three
  // from BaseChatMesh, keyed to its ContactInfo table) to dispatch on
  // _beebo_companion.node_role: REPEATER routes to the ACL-keyed body (ported from
  // simple_repeater), falling through to BaseChatMesh::<method>(...) for
  // companion's existing contact/chat path otherwise. The two never run at
  // once -- a node is either role, never both -- so this is dispatch, not a
  // conflict (see MULTI_ROLE_FW.md Phase 3's implementation note).
  int searchPeersByHash(const uint8_t* hash) override;
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override;
  void onPeerDataRecv(mesh::Packet* packet, uint8_t type, int sender_idx, const uint8_t* secret, uint8_t* data, size_t len) override;
  // beebo: found in the multi_role event-loop review -- missing entirely
  // before, not just unbranched. searchPeersByHash's REPEATER branch fills
  // acl_peer_indexes, a completely separate array from BaseChatMesh's own
  // matching_peer_indexes (keyed to the ContactInfo table) -- so relying on
  // the inherited BaseChatMesh::onPeerPathRecv for repeater-role peers reads
  // an index space that was never populated for them. Ported from
  // simple_repeater's own onPeerPathRecv (MyMesh.cpp:1019-1037): stores the
  // returned path onto the matching ClientACL entry instead.
  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path,
                      uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: net-new -- BaseChatMesh never overrides this, so no fallthrough
  // body is needed; gated if (_beebo_companion.node_role != NODE_ROLE_REPEATER) return;
  // Declaration (and BeeboRepeater.cpp's definition) both compiled out
  // entirely when repeater support isn't built in -- Mesh::onAnonDataRecv's
  // own empty default (Mesh.h:130) stands in for a companion-only build.
  void onAnonDataRecv(mesh::Packet* packet, const uint8_t* secret, const mesh::Identity& sender, uint8_t* data, size_t len) override;
#endif

  void sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis);
  void sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delay_millis=0) override;
  void sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delay_millis=0) override;

  void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override;
  void onPacketCaptured(mesh::Packet* pkt) override;                   // beebo: stage distilled fields onto pkt
  void logRxDisposition(const mesh::Packet* pkt, uint8_t reason) override;  // beebo: accumulate accept/reject reason
  void onPacketDisposed(mesh::Packet* pkt) override;  // beebo: commit the finished monitor-ring record
  void logTx(mesh::Packet* pkt, int len) override;      // beebo: capture our own TX into the monitor ring
  void logTxFail(mesh::Packet* pkt, int len) override;  // beebo: ditto, send timed out
  bool isAutoAddEnabled() const override;
  bool shouldAutoAddContactType(uint8_t type) const override;
  bool shouldOverwriteWhenFull() const override;
  uint8_t getAutoAddMaxHops() const override;
  void onContactsFull() override;
  void onContactOverwrite(const uint8_t* pub_key) override;
  bool onContactPathRecv(ContactInfo& from, uint8_t* in_path, uint8_t in_path_len, uint8_t* out_path, uint8_t out_path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override;
  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) override;
  void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override;  // beebo: track direct neighbours
  void onContactPathUpdated(const ContactInfo &contact) override;
  ContactInfo* processAck(const uint8_t *data) override;
  void queueMessage(const ContactInfo &from, uint8_t txt_type, mesh::Packet *pkt, uint32_t sender_timestamp,
                    const uint8_t *extra, int extra_len, const char *text);

  void onMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                     const char *text) override;
  void onCommandDataRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                         const char *text) override;
  void onSignedMessageRecv(const ContactInfo &from, mesh::Packet *pkt, uint32_t sender_timestamp,
                           const uint8_t *sender_prefix, const char *text) override;
  void onChannelMessageRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint32_t timestamp,
                            const char *text) override;
  void onChannelDataRecv(const mesh::GroupChannel &channel, mesh::Packet *pkt, uint16_t data_type,
                         const uint8_t *data, size_t data_len) override;

  uint8_t onContactRequest(const ContactInfo &contact, uint32_t sender_timestamp, const uint8_t *data,
                           uint8_t len, uint8_t *reply) override;
  void onContactResponse(const ContactInfo &contact, const uint8_t *data, uint8_t len) override;
  void onControlDataRecv(mesh::Packet *packet) override;
  void onRawDataRecv(mesh::Packet *packet) override;
  void onTraceRecv(mesh::Packet *packet, uint32_t tag, uint32_t auth_code, uint8_t flags,
                   const uint8_t *path_snrs, const uint8_t *path_hashes, uint8_t path_len) override;
  void onPokeReply(mesh::Packet *packet, uint32_t tag, int16_t rssi, int16_t snr, int16_t noise_floor) override;

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override;
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override;
  void onSendTimeout() override;

  // DataStoreHost methods
  bool onContactLoaded(const ContactInfo& contact) override { return addContact(contact); }
  bool getContactForSave(uint32_t idx, ContactInfo& contact) override { return getContactByIdx(idx, contact); }
  bool onChannelLoaded(uint8_t channel_idx, const ChannelDetails& ch) override { return setChannel(channel_idx, ch); }
  bool getChannelForSave(uint8_t channel_idx, ChannelDetails& ch) override { return getChannel(channel_idx, ch); }

  void clearPendingReqs() {
    pending_login = pending_status = pending_telemetry = pending_discovery = pending_req = 0;
  }

public:
  // beebo: one persistence policy for BOTH pref stores (NodePrefs /node_prefs
  // and CommonCLI's /com_prefs). Every setting already lives in a RAM cache --
  // _prefs for the companion fields, _com_prefs_cache for the repeater ones --
  // so no setter needs to reach flash itself: it mutates its cache, marks that
  // store dirty, and a single flush point decides when (and whether) the bytes
  // land. That makes autosave one rule instead of two, and makes batching a
  // property of the flush point rather than something each command open-codes.
  //
  // When _save_prefs is false, changes to either store stay in RAM (avoids
  // flash thrash during measurement sweeps) until 'commit' or a 'restore'
  // discards them. The flag is RAM-only, never persisted, defaults to true at
  // every boot. Before this was unified, _com_prefs writes ignored the flag
  // entirely, so a repeater.* change hit flash even with autosave off.
  // beebo: marks NodePrefs (/node_prefs) AND BeeboCompanionPrefs
  // (/beebo_companion) dirty together rather than requiring every call site
  // to know which of the two structs its own field actually lives in --
  // writeDirtyPrefs() below still only writes whichever file's bit is set,
  // so this just means an unrelated store's flush is occasionally a no-op
  // rewrite of unchanged bytes, never a correctness issue.
  void savePrefs() { _prefs_dirty = _beebo_companion_dirty = true; flushDirtyPrefs(); }
  void markComPrefsDirty() { _com_prefs_dirty = true; flushDirtyPrefs(); }

  // Write whatever is dirty, if policy allows it right now. A no-op inside a
  // batch (see beginPrefsBatch) or while autosave is off -- the dirty bits
  // survive either way, so the bytes go out at the batch end / on commit.
  void flushDirtyPrefs() {
    if (!_save_prefs || _batch_depth > 0) return;
    writeDirtyPrefs();
  }

  // beebo: defer flushes until the matching endPrefsBatch(), so a command
  // touching N fields across both stores costs one write per store instead of
  // one per field (SET_PREFS_TLV's whole point). Nests: only the outermost
  // end flushes.
  void beginPrefsBatch() { _batch_depth++; }
  void endPrefsBatch() {
    if (_batch_depth > 0) _batch_depth--;
    flushDirtyPrefs();
  }

  bool getSavePrefs() const { return _save_prefs; }
  void setSavePrefs(bool on) { _save_prefs = on; }   // does not commit
  // 'commit': force-write current RAM state once, whatever the policy says.
  // Deliberately unconditional rather than dirty-gated -- an explicit commit
  // is a user asking for the files to match RAM, including after a direct
  // _prefs mutation that never called savePrefs().
  void commitPrefs() {
    _prefs_dirty = _com_prefs_dirty = _beebo_companion_dirty = _beebo_repeater_dirty = true;
    writeDirtyPrefs();
  }
  void reloadPrefs();       // 'restore': re-read persisted prefs from flash (discard RAM)

#if ENV_INCLUDE_GPS == 1
  void applyGpsPrefs() {
    sensors.setSettingValue("gps", _prefs.gps_enabled ? "1" : "0");
    if (_prefs.gps_interval > 0) {
      char interval_str[12];  // Max: 24 hours = 86400 seconds (5 digits + null)
      sprintf(interval_str, "%u", _prefs.gps_interval);
      sensors.setSettingValue("gps_interval", interval_str);
    }
  }
#endif

  // To check if there is pending work
  bool hasPendingWork() const;

  // beebo: true while an OTA session is open (between CMD_OTA_BEGIN and reboot);
  // main.cpp loop() uses this to drive the rapid OTA flash on the status LED.
  bool isOTAActive() const { return ota_partition != NULL; }
  // beebo: true if CMD_OTA_BEGIN requested priority mode (see loop()).
  bool isOTAPriority() const { return ota_priority; }

#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: CommonCLICallbacks implementation -- see
  // beebo/plans/COMMONCLI_TEXT_DISPATCH.md's "CommonCLICallbacks
  // implementation surface" table for what each maps onto and why. `cli`
  // (the real CommonCLI instance) only ever serves repeater role, so this
  // whole block is repeater-only, same as `cli`/`com_prefs` themselves.
  const char* getFirmwareVer() override { return FIRMWARE_VERSION; }
  const char* getBuildDate() override { return FIRMWARE_BUILD_DATE; }
  const char* getRole() override { return "Repeater"; }
  bool formatFileSystem() override { return _store->formatFileSystem(); }
  void sendSelfAdvertisement(int delay_millis, bool flood) override;
  void setLoggingOn(bool enable) override { /* beebo: placeholder -- no log-file concept to wrap yet */ }
  void eraseLogFile() override { /* beebo: placeholder, see setLoggingOn() */ }
  void dumpLogFile() override { /* beebo: placeholder, see setLoggingOn() */ }
  void setTxPower(int8_t power_dbm) override { radio_driver.setTxPower(power_dbm); }
  void formatNeighborsReply(char* reply) override;
  void removeNeighbor(const uint8_t* pubkey, int key_len) override { removeNeighborByPrefix(pubkey, key_len); }
  void formatStatsReply(char* reply) override { strcpy(reply, "not supported"); }
  void formatRadioStatsReply(char* reply) override { strcpy(reply, "not supported"); }
  void formatPacketStatsReply(char* reply) override { strcpy(reply, "not supported"); }
  mesh::LocalIdentity& getSelfId() override { return self_id; }
  void saveIdentity(const mesh::LocalIdentity& new_id) override;
  void clearStats() override;
  void applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) override;
  void startRegionsLoad() override { /* beebo: placeholder -- the stateful multi-line 'region load' session isn't wired into handleCommand() here; 'region def'/'region put' cover one-shot bulk definition instead */ }
  bool saveRegions() override { return region_map.save(_store->getPrimaryFS()); }
  void onDefaultRegionChanged(const RegionEntry* r) override { /* beebo: region_map's own default-region flag is persisted by saveRegions() above; no separate live-scoping consumer wired yet, unlike _beebo_companion.default_scope_key's own periodic-advert path */ }
  void setRxBoostedGain(bool enable) override { radio_driver.setRxBoostedGainMode(enable); }
#endif

private:
  // Returns true (once) when CMD_SET_WIFI_CREDS was received; loop()'s own
  // transport-management block (below) uses this to start WiFi live, without
  // rebooting. Resets the flag on read.
  bool consumeWifiCredsPending() {
    bool v = _wifi_creds_pending;
    _wifi_creds_pending = false;
    return v;
  }
  // Returns true (once) when CMD_SET_TRANSPORT_CONFIG was received; loop()'s
  // transport-management block uses this to bring BLE/TCP/USB up or down
  // live, without rebooting. Resets the flag on read.
  bool consumeTransportConfigPending() {
    bool v = _transport_config_pending;
    _transport_config_pending = false;
    return v;
  }

  // beebo: transport ownership -- moved in from main.cpp so Beebo is a
  // genuinely self-contained multi-role/multi-transport mesh class (its own
  // NodePrefs already hardcode ble_enabled/tcp_enabled/usb_enabled, so it was
  // never really transport-agnostic; now it owns the objects that realize
  // those prefs too, not just the decisions). main.cpp keeps only the truly
  // platform-specific wiring: Serial/board/radio_init and the status LED.
  DualModeSerialInterface usb_interface;
  SerialWifiInterface wifi_interface;
  SerialBLEInterface ble_interface;
  MultiSerialInterface serial_interface;   // aggregates BLE + WiFi + USB (base build)

  bool _wifi_started = false;     // true once wifi_interface has been added to serial_interface
  bool _wifi_up = false;          // true while the WiFi radio + interface are currently live
  bool _ble_added = false;        // true once ble_interface has been added to serial_interface
  bool _ble_up = false;           // true while the BLE radio + interface are currently live
  bool _usb_added = false;        // true once usb_interface has been added to serial_interface
  bool _usb_up = false;           // true while the USB interface is currently live

  bool _wifi_needs_reconnect = false;
  unsigned long _last_wifi_reconnect_attempt = 0;
  bool _wifi_suspended = false;   // true while the WiFi radio is intentionally powered down (MULTI_TRANSPORT)
  // WiFi STA events fire in the event-loop task; stash them here and log to the
  // (non-thread-safe) debug ring from loop() to keep all ring writes single-context.
  volatile int  _sta_disc_reason = -1;   // >=0 when a STA disconnect needs logging
  volatile bool _sta_got_ip = false;     // true when a STA got-IP needs logging

  // A live ble<->tcp switch queues its OK reply on whichever transport carried
  // the command, but that transport's send_queue is only drained by a *later*
  // checkRecvFrame() call -- if we tear it down in the same loop() iteration
  // that queued the reply, the reply (and the caller's confirmation) is lost
  // and they just see a hard disconnect. Defer the teardown a short beat so
  // at least one more loop() pass gets a chance to flush it first.
  bool _ble_teardown_pending = false;
  unsigned long _ble_teardown_time = 0;
  bool _wifi_teardown_pending = false;
  unsigned long _wifi_teardown_time = 0;
  bool _usb_teardown_pending = false;
  unsigned long _usb_teardown_time = 0;

  void beginTransports();       // called from begin(): bring up transports per persisted prefs
  void loopTransports();        // called from loop(): STA-event drain, reconnect, live provisioning/toggle, teardown timers

#ifdef P_LORA_TX_LED
  // beebo: status LED (PWM via analogWrite), level chosen each loop by
  // priority high -> low: OTA active (rapid flash) > radio TX (long high
  // blink) > radio RX (short high blink) > transport activity (low-intensity
  // flash) > baseline heartbeat (low, every 5s). Moved in from main.cpp --
  // it only ever reads Beebo's own counters (sent/recv/transport-activity/
  // OTA-active), so it belongs here with the rest of the class's state, same
  // rationale as the transport-ownership move above.
  int led_level = -1;                      // last level written to the LED (-1 = unset)
  uint32_t led_last_sent = 0;              // last observed radio TX packet count
  uint32_t led_last_recv = 0;              // last observed radio RX packet count
  unsigned long led_tx_flash_until = 0;    // deadline for TX blink
  unsigned long led_rx_flash_until = 0;    // deadline for RX blink
  uint32_t led_last_activity = 0;          // last observed transport frame count
  unsigned long led_xport_flash_until = 0; // deadline for transport-activity flash
  void updateStatusLed();
#endif

  // beebo: refreshes _cached_batt_mv/_batt_state once the sample window opens
  // and radioIsIdle() (or the hard deadline passes, radio idle or not -- see
  // updateBattTrend()). force_read forces a live board.getBattMilliVolts()
  // regardless of scheduling (the status commands always need a fresh reading
  // to reply with); otherwise an ADC read only happens once due, so idle
  // loop() ticks don't pay for it. Returns the live reading if one was taken,
  // else the last cached one. Called from loop() (so the trend still advances
  // on idle nodes) and opportunistically from status handlers, so a status
  // query can trigger reclassification the instant it's due instead of
  // waiting for the next loop() tick -- but reclassification itself never
  // happens more often than batt_sample_period_secs, since the classifier's
  // hysteresis assumes deltas measured over that period, not over however
  // often the app happens to poll.
  uint16_t updateBattTrend(bool force_read = false);
  void clampRadioPrefs();  // beebo: sanitise radio-affecting prefs (shared by begin/reloadPrefs)
  void applyRadioPrefs();  // beebo: push radio-affecting prefs into radio driver + board FEM
  void writeOKFrame();
  void writeErrFrame(uint8_t err_code);
  void writeDisabledFrame();
  void writeContactRespFrame(uint8_t code, const ContactInfo &contact);
  void updateContactFromFrame(ContactInfo &contact, uint32_t& last_mod, const uint8_t *frame, int len);
  void addToOfflineQueue(const uint8_t frame[], int len);
  int getFromOfflineQueue(uint8_t frame[]);
  int getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]) override { 
    return _store->getBlobByKey(key, key_len, dest_buf);
  }
  bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], int len) override {
    return _store->putBlobByKey(key, key_len, src_buf, len);
  }

  void checkSerialInterface();
  bool isValidClientRepeatFreq(uint32_t f) const;

  // helpers, short-cuts
  void saveChannels() { _store->saveChannels(this); }
  void saveContacts();

  DataStore* _store;
  NodePrefs _prefs;
  BeeboCompanionPrefs _beebo_companion;
  // beebo: cached (_beebo_companion.node_role == NODE_ROLE_REPEATER), kept in sync by
  // setNodeRole() -- avoids re-deriving this comparison at every one of its
  // call sites. Only setNodeRole() and the post-load sync in begin()/
  // reloadPrefs() may write _beebo_companion.node_role directly; everything else
  // (including the CMD_SET_NODE_ROLE handlers) must go through setNodeRole()
  // so the cache can't drift from the persisted value.
  bool _is_repeater = false;
  // beebo: mutually exclusive with _is_repeater -- node_role is a strict
  // either/or (companion or repeater), see setNodeRole()/isNodeRoleBuiltIn().
  bool _is_companion = false;
  // beebo: PER_ROLE_IDENTITY -- defined in Beebo.cpp (needs radio_new_identity(),
  // declared per-board in target.h, not necessarily visible from this header).
  // Loads self_id for `role`, migrating the legacy shared identity in or
  // generating a fresh one exactly like begin() does on first use of a role.
  void reloadIdentityForRole(uint8_t role);
  void setNodeRole(uint8_t role, uint8_t source) {
    if (role != _beebo_companion.node_role) {
      appendSettingChangedEvent(SETTING_NODE_ROLE, _beebo_companion.node_role, role, source);
      reloadIdentityForRole(role);
    }
    _beebo_companion.node_role = role;
    _is_repeater = (role == NODE_ROLE_REPEATER);
    _is_companion = (role == NODE_ROLE_COMPANION);
  }
  // beebo: STATIC_ROLE_BUILDS Phase 1 -- true if `role` is actually compiled
  // into this binary. Callers of setNodeRole() that accept a role from the
  // outside (SET_NODE_ROLE, `node.role` text command) must check this
  // first: a static-role build has no acl/region_map/anon_limiter
  // (repeater) or no contact/channel/message store (companion), so ending
  // up with _is_repeater/_is_companion true for an uncompiled half would
  // dereference members that don't exist. Also false for any value other
  // than NODE_ROLE_COMPANION/NODE_ROLE_REPEATER (e.g. a stale persisted
  // COPEATER=2 from older firmware), which syncNodeRoleCache() below falls
  // back away from.
  static bool isNodeRoleBuiltIn(uint8_t role) {
#if !BEEBO_ENABLE_COMPANION_ROLE
    if (role == NODE_ROLE_COMPANION) return false;
#endif
#if !BEEBO_ENABLE_REPEATER_ROLE
    if (role == NODE_ROLE_REPEATER) return false;
#endif
    return role == NODE_ROLE_COMPANION || role == NODE_ROLE_REPEATER;
  }
  // beebo: STATIC_ROLE_BUILDS Phase 1 -- called right after
  // DataStore::loadPrefs() writes _beebo_companion.node_role directly from persisted
  // storage (begin() and reloadPrefs()), which bypasses setNodeRole()'s
  // isNodeRoleBuiltIn() check. A device previously flashed with multi_role
  // (or the other static-role build, or older firmware with a persisted
  // COPEATER=2) can have a persisted node_role this binary doesn't support
  // -- fall back to whichever half IS built in (preferring companion)
  // rather than leaving _is_repeater/_is_companion true for uncompiled state.
  void syncNodeRoleCache() {
    if (!isNodeRoleBuiltIn(_beebo_companion.node_role)) {
#if BEEBO_ENABLE_COMPANION_ROLE
      _beebo_companion.node_role = NODE_ROLE_COMPANION;
#else
      _beebo_companion.node_role = NODE_ROLE_REPEATER;
#endif
    }
    _is_repeater = (_beebo_companion.node_role == NODE_ROLE_REPEATER);
    _is_companion = (_beebo_companion.node_role == NODE_ROLE_COMPANION);
  }
  bool _save_prefs = true;       // beebo: RAM-only flag gating both stores' writes (see savePrefs)
  bool _prefs_dirty = false;     // beebo: NodePrefs RAM cache differs from /node_prefs
  bool _com_prefs_dirty = false; // beebo: _com_prefs_cache differs from /com_prefs
  bool _beebo_companion_dirty = false; // beebo: BeeboCompanionPrefs RAM cache differs from /beebo_companion
  bool _beebo_repeater_dirty = false;  // beebo: BeeboRepeaterPrefs RAM cache differs from /beebo_repeater
  int _batch_depth = 0;          // beebo: >0 while a multi-field command defers its flush
  // beebo: multi_role Phase 3 -- repeater-role admin login/permission table.
  // Loaded unconditionally at boot regardless of node.role (cheap, harmless
  // when role == companion, matching MULTI_ROLE_FW.md's Phase 4 guidance
  // pulled forward rather than gating storage init on a runtime value).
#if BEEBO_ENABLE_REPEATER_ROLE
  ClientACL acl;
  // beebo: Phase 3 item 2 -- named-region registry, queried by
  // handleAnonRegionsReq (an admin discovery feature, distinct from and
  // unrelated to companion's own existing TransportKeyStore/send_scope
  // send-scoping mechanism above -- that one is untouched/reused as-is,
  // this is new, separate infrastructure simple_repeater has and companion
  // never did). Loaded (read-only for now) by ensureRepeaterStateLoaded()
  // (BeeboRepeater.cpp) -- boot-time if node.role is repeater, lazily on
  // first switch into repeater otherwise (see that method's own comment);
  // nothing populates it yet since region editing is a text-CLI command
  // (Phase 5/6) -- until then it always reports zero regions.
  TransportKeyStore key_store;
  RegionMap region_map{key_store};
  // beebo: the real upstream CommonCLI class, wired to `this` as its
  // CommonCLICallbacks* -- see Beebo.h's ComPrefs rename mechanism (top of
  // this file) and beebo/plans/COMMONCLI_TEXT_DISPATCH.md's "Wiring" steps
  // 6-11. `com_prefs` is a normal fixed-address member, bound once via
  // CommonCLI's constructor initializer list (CommonCLI.h has no setter),
  // exactly like companion's own `_prefs`.
  ComPrefs com_prefs;
  CommonCLI cli{board, *getRTCClock(), sensors, region_map, acl, &com_prefs, this};
  // beebo: repeater's own dedup_window_ms, the one field with no home in
  // ComPrefs at all -- see BeeboRepeaterPrefs.h's own comment.
  BeeboRepeaterPrefs _beebo_repeater;
  // beebo: Phase 3 item 2 -- ACL peer-match scratch space, filled by
  // searchPeersByHash's REPEATER branch, read back by getPeerSharedSecret/
  // onPeerDataRecv. Deliberately separate from BaseChatMesh's own
  // matching_peer_indexes[MAX_SEARCH_RESULTS] (only 8 slots -- too small
  // for MAX_CLIENTS=32 ACL entries that could share a hash prefix).
  int acl_peer_indexes[MAX_CLIENTS];
  // beebo: rate-limits anon (pre-login) requests -- login attempts and the
  // unauthenticated regions/owner/clock queries. Same construction params
  // as simple_repeater's (max 4 every 3 minutes).
  RateLimiter anon_limiter{4, 180};
#endif
  // beebo: repeater-role's own persisted name, distinct from the companion
  // name (NodePrefs.node_name) -- now the real com_prefs.node_name field
  // (ComPrefs, /com_prefs), loaded/saved via cli.loadPrefs()/cli.savePrefs()
  // like every other ComPrefs field. Empty (all-zero, e.g. a device that
  // never set it) means "not customized" -- getRepeaterName() falls back to
  // the companion name in that case.
  const char* getRepeaterName() const {
#if BEEBO_ENABLE_REPEATER_ROLE
    if (com_prefs.node_name[0]) return com_prefs.node_name;
#endif
    return _prefs.node_name;
  }
  void writeDirtyPrefs();   // write every dirty store now, ignoring policy (see savePrefs)

  // beebo: GET_PREFS_TLV/SET_PREFS_TLV (BEEBO_CMD_*, see protocol.yaml) --
  // one round trip for however many of these fields the caller wants,
  // instead of one round trip per field (`beebo prefs-tlv-bench` measured
  // 759.7ms -> 6.1ms, ~124x, for these 12 fields over real hardware).
  // Every existing individual BEEBO_CMD_GET_X/SET_X opcode keeps working
  // completely unchanged -- other apps depend on them -- because
  // PREFS_TLV_FIELDS below and each individual handler in Beebo.cpp both
  // call the SAME tlvGet*/tlvSet* function per field (defined in
  // BeeboRepeater.cpp next to the ComPrefs cache they read/write). One
  // implementation per field, called from two places, so the two paths
  // can't drift apart.
  //
  // Currently covers every /com_prefs (repeater-only "ComPrefs") field
  // `beebo settings repeater.*` exposes. NodePrefs-backed fields (radio/fem/
  // adc/battery/etc.) are a natural follow-up extension of this same table
  // -- not yet added.
  //
  // Deliberately NOT in this table, even among ComPrefs fields (both GET
  // and SET stay individual-command-only): none currently -- every ComPrefs
  // field this firmware exposes is a plain stored value or string, safe to
  // batch. (node.role, wifi credentials, board.state.quiet, node.transport.*,
  // and node.role -- are still excluded, each for its own reason (see the
  // plan doc): quiet's SET has a live radio/FEM sleep-or-wake side effect,
  // not a plain stored value, and node.role is a full role transition, not a
  // stored value. WIFI_SSID/TRANSPORT_CONFIG/MONRING_CONFIG WERE excluded
  // for the same "might reboot mid-batch" worry, but that turned out to be
  // based on stale docs (SET_WIFI_CREDS's reboot is opt-in at build time via
  // WIFI_CREDS_REBOOT, off by default -- see its individual handler) and an
  // over-cautious read of SET_TRANSPORT_CONFIG (its reboot is *caller-
  // requested* via an explicit flag that only the individual command
  // exposes -- the TLV path below simply never requests one, the same way
  // "the user will explicitly change transport and reboot" separately). All
  // three are now in the table, using the exact same tlvGet*/tlvSet*
  // registration pattern as every ComPrefs field above -- that pattern IS
  // the general mechanism for adding a side-effect-bearing field safely:
  // write set_raw so it does precisely what the individual handler does,
  // minus anything (like a reboot) that only makes sense as a single
  // deliberate action, and register it here.
  enum PrefsTlvKey : uint8_t {
    PREFS_TLV_REPEAT_MODE = 1,
    PREFS_TLV_TXDELAY_FACTOR = 2,
    PREFS_TLV_DIRECT_TXDELAY_FACTOR = 3,
    PREFS_TLV_ALLOW_READ_ONLY = 4,
    PREFS_TLV_AGC_RESET_INTERVAL = 5,
    PREFS_TLV_LOOP_DETECT = 6,
    PREFS_TLV_FLOOD_ADVERT_INTERVAL = 7,
    PREFS_TLV_INTERFERENCE_THRESHOLD = 8,
    PREFS_TLV_ADVERT_INTERVAL = 9,
    PREFS_TLV_FLOOD_MAX = 10,
    PREFS_TLV_FLOOD_MAX_UNSCOPED = 11,
    PREFS_TLV_FLOOD_MAX_ADVERT = 12,
    PREFS_TLV_OWNER_INFO = 13,        // string
    PREFS_TLV_REPEATER_NAME = 14,     // string
    PREFS_TLV_WIFI_SSID = 15,         // string
    PREFS_TLV_TRANSPORT_CONFIG = 16,  // packed byte0=ble byte1=tcp byte2=usb, never reboots
    PREFS_TLV_MONRING_CONFIG = 17,
    PREFS_TLV_REPEATER_RXDELAY = 18,   // repeater's own rx_delay_base (offset 80) -- see Beebo.h's com_prefs.rx_delay_base comment
    PREFS_TLV_REPEATER_AIRTIME = 19,   // repeater's own airtime_factor (offset 0) -- see Beebo.h's com_prefs.airtime_factor comment
    PREFS_TLV_REPEATER_DEDUP_WINDOW = 20,  // repeater's own dedup_window_ms -- see BeeboRepeaterPrefs.h
    // beebo: SETTINGS_TREE_CLEANUP.md Decision A pattern -- repeater's own
    // independent copy of a field stock upstream only ever gave one shared
    // NodePrefs-shaped copy of. com_prefs already has these bytes (same
    // struct as _prefs, see PROTOCOL_AND_SETTINGS_STORAGE.md's "critical
    // trap" section) -- no new storage, just a repeater-scoped accessor.
    // Companion's own copy (_prefs.multi_acks/path_hash_mode, sensors.
    // node_lat/node_lon) stays reachable exactly as before via the stock
    // CMD_SET_OTHER_PARAMS/CMD_SET_PATH_HASH_MODE/CMD_SET_ADVERT_LATLON
    // opcodes and stock CommonCLI text keys -- unrelated to these.
    PREFS_TLV_REPEATER_MULTI_ACKS = 21,
    PREFS_TLV_REPEATER_PATH_HASH_MODE = 22,
    PREFS_TLV_REPEATER_LAT = 23,   // fixed-point *1e6 in an int32
    PREFS_TLV_REPEATER_LON = 24,   // fixed-point *1e6 in an int32
  };
  enum PrefsTlvType : uint8_t { TLV_U32 = 0, TLV_FLOAT = 1, TLV_STRING = 2 };
  struct PrefsTlvField {
    uint8_t key;      // PrefsTlvKey
    uint8_t type;      // PrefsTlvType
    uint32_t (*get_raw)(Beebo* self);                            // TLV_U32/TLV_FLOAT (float bit-cast into the u32)
    bool (*set_raw)(Beebo* self, uint32_t raw);                   // TLV_U32/TLV_FLOAT
    int (*get_str)(Beebo* self, uint8_t* out, size_t max_len);     // TLV_STRING: returns length written
    bool (*set_str)(Beebo* self, const uint8_t* in, size_t len);   // TLV_STRING
  };
  static const PrefsTlvField PREFS_TLV_FIELDS[];
  static const size_t PREFS_TLV_FIELD_COUNT;

  // Every field above needs more than a plain store: a value transform
  // (AGC_RESET_INTERVAL's stored-quarter-seconds, ADVERT_INTERVAL's
  // asymmetric *2-on-read/raw-store-on-write -- see its individual
  // SET_ADVERT_INTERVAL comment in Beebo.cpp for why), validation
  // (LOOP_DETECT rejects out-of-range, INTERFERENCE_THRESHOLD clamps,
  // ALLOW_READ_ONLY normalizes to 0/1), a RAM-shadow sync
  // (allowPacketForward()'s hot-path _fwd_* copies -- see Beebo.h's field
  // comment above them -- or com_prefs.owner_info/com_prefs.node_name for the
  // string fields), or a timer rearm (FLOOD_ADVERT_INTERVAL/
  // ADVERT_INTERVAL). No setter writes flash itself: the ComPrefs-backed ones
  // touch _com_prefs_cache (via _writeComPrefsCacheOnly, not
  // writeComPrefsField) and the NodePrefs-backed ones (WIFI_SSID/
  // TRANSPORT_CONFIG/MONRING_CONFIG) mutate _prefs and call savePrefs() --
  // both of which just mark their store dirty. The flush comes from the
  // caller: deferred to the batch end for a SET_PREFS_TLV payload, immediate
  // for a single individual SET_* call (see BeeboRepeater.cpp's definitions
  // for the exact per-field logic).
  static uint32_t tlvGetRepeatMode(Beebo* self);
  static bool tlvSetRepeatMode(Beebo* self, uint32_t raw);
  static uint32_t tlvGetTxDelayFactor(Beebo* self);
  static bool tlvSetTxDelayFactor(Beebo* self, uint32_t raw);
  static uint32_t tlvGetDirectTxDelayFactor(Beebo* self);
  static bool tlvSetDirectTxDelayFactor(Beebo* self, uint32_t raw);
  static uint32_t tlvGetAllowReadOnly(Beebo* self);
  static bool tlvSetAllowReadOnly(Beebo* self, uint32_t raw);
  static uint32_t tlvGetAgcResetInterval(Beebo* self);
  static bool tlvSetAgcResetInterval(Beebo* self, uint32_t raw);
  static uint32_t tlvGetLoopDetect(Beebo* self);
  static bool tlvSetLoopDetect(Beebo* self, uint32_t raw);
  static uint32_t tlvGetFloodAdvertInterval(Beebo* self);
  static bool tlvSetFloodAdvertInterval(Beebo* self, uint32_t raw);
  static uint32_t tlvGetInterferenceThreshold(Beebo* self);
  static bool tlvSetInterferenceThreshold(Beebo* self, uint32_t raw);
  static uint32_t tlvGetAdvertInterval(Beebo* self);
  static bool tlvSetAdvertInterval(Beebo* self, uint32_t raw);
  static uint32_t tlvGetFloodMax(Beebo* self);
  static bool tlvSetFloodMax(Beebo* self, uint32_t raw);
  static uint32_t tlvGetFloodMaxUnscoped(Beebo* self);
  static bool tlvSetFloodMaxUnscoped(Beebo* self, uint32_t raw);
  static uint32_t tlvGetFloodMaxAdvert(Beebo* self);
  static bool tlvSetFloodMaxAdvert(Beebo* self, uint32_t raw);
  static int tlvGetOwnerInfo(Beebo* self, uint8_t* out, size_t max_len);
  static bool tlvSetOwnerInfo(Beebo* self, const uint8_t* in, size_t len);
  static int tlvGetRepeaterName(Beebo* self, uint8_t* out, size_t max_len);
  static bool tlvSetRepeaterName(Beebo* self, const uint8_t* in, size_t len);
  static int tlvGetWifiSsid(Beebo* self, uint8_t* out, size_t max_len);
  static bool tlvSetWifiSsid(Beebo* self, const uint8_t* in, size_t len);
  static uint32_t tlvGetTransportConfig(Beebo* self);
  static bool tlvSetTransportConfig(Beebo* self, uint32_t raw);
  static uint32_t tlvGetMonringConfig(Beebo* self);
  static bool tlvSetMonringConfig(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterRxDelayBase(Beebo* self);
  static bool tlvSetRepeaterRxDelayBase(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterAirtimeFactor(Beebo* self);
  static bool tlvSetRepeaterAirtimeFactor(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterDedupWindow(Beebo* self);
  static bool tlvSetRepeaterDedupWindow(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterMultiAcks(Beebo* self);
  static bool tlvSetRepeaterMultiAcks(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterPathHashMode(Beebo* self);
  static bool tlvSetRepeaterPathHashMode(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterLat(Beebo* self);
  static bool tlvSetRepeaterLat(Beebo* self, uint32_t raw);
  static uint32_t tlvGetRepeaterLon(Beebo* self);
  static bool tlvSetRepeaterLon(Beebo* self, uint32_t raw);

  // Encodes every field above into out (caller-sized) as [key][len][value]
  // triplets, returns bytes written. Decodes one triplet from in[pos..],
  // looks its key up in PREFS_TLV_FIELDS and applies it via that field's
  // set_raw/set_str -- same function the matching individual SET_* handler
  // calls, so behavior is identical by construction, not by convention.
  // set_raw/set_str only touch the RAM cache; the caller (CMD_BEEBO's
  // SET_PREFS_TLV dispatch, Beebo.cpp) flushes it to flash ONCE after every
  // triplet in the payload has been applied, however many fields changed --
  // one flash write, not one per field.
  int encodePrefsTlv(uint8_t* out, size_t max_len);
  bool applyPrefsTlvTriplet(const uint8_t* in, size_t len, size_t& pos);

  // beebo: CommonCLI fallback's 'tempradio <freq> <bw> <sf> <cr> <mins>' --
  // ported from simple_repeater's own MyMesh.h/.cpp (same field names,
  // same two-stage set-then-revert timer), ticked from loopRepeater() since
  // this command is only reachable via the repeater-only CommonCLI fallback.
  uint32_t _temp_set_radio_at = 0, _temp_revert_radio_at = 0;
  float _temp_pending_freq = 0.0f, _temp_pending_bw = 0.0f;
  uint8_t _temp_pending_sf = 0, _temp_pending_cr = 0;
  bool removeNeighborByPrefix(const uint8_t* pubkey, int key_len);
  // beebo: pushes whichever of _beebo_companion.dedup_window_ms (companion) /
  // _beebo_repeater.dedup_window_ms (repeater) is role-appropriate into the shared
  // SimpleMeshTables instance -- called after boot, a prefs reload, either
  // dedup-window SET handler, and every node.role hot switch (binary and
  // text CLI), since SimpleMeshTables caches the value rather than reading
  // _prefs/_is_repeater live like rx_delay_base/airtime_factor's call sites do.
  void pushActiveDedupWindow();

  // beebo: found while fixing the event-loop review's regressions --
  // begin()/loop() are split by role into their own files
  // (BeeboCompanion.cpp/BeeboRepeater.cpp), dispatched from the shared
  // Beebo::begin()/Beebo::loop() (Beebo.cpp). This is what actually
  // fixes the boot-time regression: beginRepeater()'s ACL/region/ComPrefs
  // loading (~13 separate file opens) only runs for a node that's actually
  // booting as (or switching into) repeater role, instead of unconditionally
  // regardless of role like it did before -- companion boots now do exactly
  // what companion_radio's own begin() does, no more, no less.
  //
  // ensureRepeaterStateLoaded() is the one piece that's genuinely shared
  // between the two entry points: beginRepeater() (boot-as-repeater) and
  // the SET_NODE_ROLE handlers (switching into repeater mid-session, Phase
  // 4 hot-switch) both call it, guarded by _repeater_state_loaded so the
  // actual load only ever happens once per boot regardless of how many
  // times the role toggles afterward.
#if BEEBO_ENABLE_COMPANION_ROLE
  void beginCompanion();
  void loopCompanion(bool skip_radio);
#endif
#if BEEBO_ENABLE_REPEATER_ROLE
  void beginRepeater();
  void loopRepeater(bool skip_radio);
  void ensureRepeaterStateLoaded();
  bool _repeater_state_loaded = false;
#endif
  // beebo: restores stock simple_repeater's own default-scope mechanism
  // (MyMesh.cpp's begin()/onDefaultRegionChanged(), RegionMap's independent
  // default_id, distinct from home_id/getHomeRegion()) instead of borrowing
  // companion's _prefs.default_scope_key, the ad-hoc Phase 3 stand-in used
  // before this. No cached TransportKey member (unlike stock's own
  // default_scope field) -- computed fresh from region_map.getDefaultRegion()
  // each call, only ever needed on a periodic flood-advert timer or an
  // explicit advert action, not a per-packet hot path, so a live cache
  // (and the onDefaultRegionChanged() invalidation it would need) isn't
  // worth the complexity. Declared unconditionally (like region_map/
  // com_prefs themselves) since sendFloodReply() -- always compiled,
  // though only ever repeater-reachable at runtime -- calls it; body is
  // internally #if-guarded, same pattern as the tlvGetRepeater* accessors.
  void getRepeaterDefaultScope(TransportKey& out);

  // beebo: region of the flood packet currently being evaluated by
  // allowPacketForward(), set by filterRecvFloodPacket() just before Mesh's
  // base-class routing calls it -- ported from simple_repeater's own
  // recv_pkt_region/filterRecvFloodPacket (Beebo.cpp:818-833), reusing
  // companion's existing region_map instance rather than a second one.
  RegionEntry* recv_pkt_region = NULL;
  bool isLooped(const mesh::Packet* packet, const uint8_t max_counters[]);

  // beebo: allowPacketForward()'s three flood-decline causes, split into
  // their own cumulative counters/MON_EVENT trail (EVENT_MAX_HOP_NO_FWD/
  // EVENT_REGION_NO_FWD/EVENT_LOOP_NO_FWD, MonRing.h) instead of all three
  // collapsing into the one generic RX_DISP_NO_FORWARD routing-axis bit
  // routeRecvPacket() logs -- that bit is shared with `repeater.repeat
  // off` too and the routing axis has no free value left to split it
  // further there, same constraint that pushed EVENT_RX_DEDUP_TABLE_FULL out
  // to its own event type rather than a disp bit. Deliberately NOT fed
  // into MonRing::SohStats/computeSoh() -- these are configured policy
  // drops (region scoping, hop/loop limits), not internal faults.
  uint32_t _max_hop_no_fwd_count = 0;
  uint32_t _region_no_fwd_count = 0;
  uint32_t _loop_no_fwd_count = 0;
  void logForwardDenyEvent(uint8_t event_type, const mesh::Packet* packet);

  // beebo: found in the multi_role event-loop review -- entirely absent
  // before, for either role. simple_repeater's own periodic local/flood
  // self-advert scheduling (Beebo.h:97, Beebo.cpp:1319-1331), gated to
  // repeater role only in loop() (see that method) -- companion has no
  // periodic self-advert concept at all (its "advert" is app/CLI-triggered
  // only), so these sit inert (0) whenever node.role is companion.
  unsigned long next_local_advert = 0, next_flood_advert = 0;
  void updateAdvertTimer();
  void updateFloodAdvertTimer();
  // beebo: "who's near me" discovery protocol (CTL_TYPE_NODE_DISCOVER_REQ/
  // RESP), ported from simple_repeater's own onControlDataRecv/
  // sendNodeDiscoverReq (Beebo.cpp:1039-1112). Neither side is role-gated:
  // answering in onControlDataRecv() replies with whatever type this node
  // currently is (repeater or chat/companion), and sending a request works
  // from either role (handleCommand's "discover.neighbors" text command, or
  // BEEBO_CMD_NODE_DISCOVER over the companion binary link).
  RateLimiter discover_limiter{4, 120};  // max 4 every 2 minutes, same as simple_repeater
  uint32_t pending_discover_tag = 0;
  unsigned long pending_discover_until = 0;
  void sendNodeDiscoverReq(uint8_t filter, bool prefix_only = false);

  uint8_t reply_data[MAX_PACKET_PAYLOAD];
  uint8_t reply_path[MAX_PATH_SIZE];
  int8_t  reply_path_len;
  uint8_t reply_path_hash_size;
  uint32_t pending_login;
  uint32_t pending_status;
  uint32_t pending_telemetry, pending_discovery;   // pending _TELEMETRY_REQ
  uint32_t pending_req;   // pending _BINARY_REQ
  BaseSerialInterface *_serial;

  ContactsIterator _iter;
  uint32_t _iter_filter_since;
  uint32_t _most_recent_lastmod;
  uint32_t _active_ble_pin;
  bool _iter_started;
  bool _pending_disconnect;
  bool send_unscoped;   // force un-scoped flood (instead of using send_scope)
  uint8_t app_target_ver;
  uint8_t *sign_data;
  uint32_t sign_data_len;
  esp_ota_handle_t ota_handle;
  const esp_partition_t *ota_partition;
  bool ota_priority = false;   // beebo: CMD_OTA_BEGIN requested priority mode
  uint32_t _ota_restart_time = 0;   // millis deadline to reboot after OTA (0 = none)
  // beebo: host's clock as of OTA_END, carried through to the deferred
  // restart so it can persist that instead of this device's own (possibly
  // wrong) current time -- see BEEBO_CMD_REBOOT_WITH_TIME's own comment.
  // 0 means no timestamp was sent (older CLI), falls back to a plain reboot.
  uint32_t _ota_restart_ts = 0;
  bool _wifi_creds_pending = false;
  // beebo: re-joining WiFi on new creds while the CMD_SET_WIFI_CREDS caller
  // is still connected over that same TCP session tears down the socket
  // before its OK reply is guaranteed delivered -- see loopTransports()'s
  // own comment. Set instead of reconnecting immediately when a session is
  // still up; consumed once wifi_interface.isConnected() goes false (the
  // caller disconnected, from having gotten its reply or otherwise), so the
  // reconnect only happens after nothing is depending on the old session.
  bool _wifi_creds_reconnect_pending = false;
  bool _transport_config_pending = false;
  unsigned long dirty_contacts_expiry;

  RadioRecord buildRadioRecord();  // beebo: snapshot current radio config, shared by initMonRing()/logRxRaw()/logTx()/logTxFail()/monring.clear() sites
  EnvRecord   buildEnvRecord();    // beebo: snapshot current env sample, shared by initMonRing()/logRxRaw()/logTx()/logTxFail()/monring.clear() sites
  // beebo: true once the radio has been quiet for IDLE_MARGIN ms (see
  // Beebo.cpp) -- general-purpose idle probe (not RX/TX-specific), used to
  // gate the periodic Vbat sample away from RX/TX-induced IR drop.
  bool radioIsIdle() const;
  MonRing monring;   // beebo: continuous monitoring capture ring (PSRAM, ESP only)
  // beebo: Phase A dynamic-tuning optimizer (DYNAMIC_OPTIMIZER_PLAN.md) --
  // repeater role only, off by default (must be turned on explicitly via
  // "set tune.enabled on"). RAM-only, like _monring_config's simple_repeater
  // counterpart: resets to off/all-observe-only on reboot, acceptable for
  // this experimental feature.
  TuneController tune_controller;
  bool _tune_enabled = false;
  unsigned long _next_tune_tick = 0;
  // beebo: per-param live-actuation promotion, bit i = TuneController::
  // specFor(i)'s param_id. 0 (default) = every param stays observe-only
  // (TuneController::tick()'s should_apply is only ever true for a param
  // whose bit is set here) -- see applyTuneDecision() for what "apply"
  // means per param.
  uint8_t _tune_applied_mask = 0;
  // beebo: setters (not plain field writes) so every call site -- binary
  // CMD_BEEBO opcode and USB/mesh-admin text CLI alike -- logs an
  // SettingRecord (MON_SETTING) through the same path, same reasoning as
  // setNodeRole() above.
  void setTuneEnabled(bool on, uint8_t source) {
    if (on != _tune_enabled) {
      appendSettingChangedEvent(SETTING_TUNE_ENABLED, _tune_enabled ? 1 : 0, on ? 1 : 0, source);
    }
    _tune_enabled = on;
  }
  void setTuneAppliedMask(uint8_t mask, uint8_t source) {
    if (mask != _tune_applied_mask) {
      appendSettingChangedEvent(SETTING_TUNE_APPLIED_MASK, _tune_applied_mask, mask, source);
    }
    _tune_applied_mask = mask;
  }
  // beebo: perform one TuneController::Decision -- writes the given TUNE_*
  // param's live value via the same tlvSet*/direct-NodePrefs path its own
  // individual GET/SET_* command uses, so a live-applied change can't drift
  // from what a human explicitly setting that same value would produce.
  void applyTuneDecision(uint8_t param_id, int16_t value);
  // beebo: Dispatcher::_err_flags (ERR_EVENT_*) is sticky -- once a bit sets
  // it never clears until reboot, and by itself carries no timestamp, so a
  // `status`/CMD_GET_STATS query long after the fact gives no idea when or
  // how many times a fault actually happened. logFaultEvent() (Dispatcher.h
  // hook override) fires immediately, every single occurrence -- not just
  // the first -- logging one EVENT_TX_POOL_FULL/EVENT_TX_CAD_TIMEOUT/
  // EVENT_RX_START_TIMEOUT record each time (one dedicated type per bit, see
  // MonRing.h), independent of _err_flags' own sticky/one-shot bit semantics.
  uint32_t _tx_pool_full_count = 0;
  uint32_t _cad_timeout_count = 0;
  uint32_t _rx_start_timeout_count = 0;
  void logFaultEvent(uint16_t bit) override;
  // beebo: DoS/QoS audit -- Dispatcher.h hook overrides, called immediately
  // (not polled) at the exact point a queueOutbound()/queueInbound() call
  // fails. is_relay=true (a received packet's own forward attempt) is
  // intentionally a no-op here -- see MonRing.h's EVENT_TX_QUEUE_FULL
  // comment for why (richer trace already lives on that packet's own
  // MON_RX record via RXREC_QUEUE_FULL).
  void logTxQueueFull(bool is_relay) override;
  void logRxQueueFull() override;
  // beebo: last-seen values of the node link (BLE/WiFi) queue-full
  // counters -- these live in ble_interface/wifi_interface with no
  // Dispatcher-level hook available (unlike the mesh-side ones above), so
  // appendLinkQueueDropEvents() polls once per tick and logs a record for
  // whichever direction actually increased since last checked.
  uint32_t _last_link_tx_queue_full = 0;
  uint32_t _last_link_rx_queue_full = 0;
  void appendLinkQueueDropEvents();
  // beebo: log one MON_SETTING record. Called from setNodeRole()/
  // setTuneEnabled()/setTuneAppliedMask() below so every path that mutates
  // one of these (binary CMD_BEEBO opcode, USB/mesh-admin text CLI) logs
  // consistently through one place, rather than instrumenting each call
  // site separately. `source` is EVENT_SOURCE_* (MonRing.h) -- the caller
  // knows which dispatch path it's on, this function doesn't guess.
  void appendSettingChangedEvent(uint8_t setting_key, uint32_t old_raw, uint32_t new_raw, uint8_t source);
  // beebo: log one MON_COMMAND record for a binary-protocol command
  // (command_id = the same (outer_cmd<<8)|sub_id encoding checkRecvFrame()'s
  // PROFILE_SCOPE already computes) -- see MonRing.h's MON_COMMAND
  // comment for the inclusion filter.
  void appendCommandRunEvent(uint16_t command_id);
  // beebo: log one MON_COMMAND record for a text-CLI command --
  // `label` is truncated to 12 bytes (CommandRecord.command), matching
  // MonRing.h's MON_COMMAND/EVENT_SOURCE_TEXT_CLI layout. Called from
  // handleCommand() (Beebo.cpp) for the same "mutating/one-shot action, not
  // a routine read" subset the binary path filters for.
  void appendTextCommandRunEvent(const char* label);
  // beebo: scratch for the fields logRxRaw distils from the raw frame, before a
  // Packet exists to hold them. logRxRaw() and onPacketCaptured() are adjacent,
  // uninterrupted calls in Dispatcher::checkRecv(), so a single scratch (not an
  // array) is safe: onPacketCaptured() copies it onto the new Packet immediately.
  RxRecord _rx_stage;
  bool     _rx_staged = false;
  // beebo: bench IR-drop test -- "CMD_SET_QUIET(1)" sleeps the radio + FEM
  // and suspends mesh/radio work in loop(), leaving only the VBAT ADC path
  // live (getBattMilliVolts(), polled via CMD_GET_STATS/STATS_TYPE_CORE) so
  // a bench meter can read board current in isolation. "CMD_SET_QUIET(0)"
  // wakes both live, no reboot needed (RadioLibWrapper::wake() +
  // LoRaFEMControl::setRxModeEnable()). Mirrors simple_repeater's
  // "board.quiet" CLI command.
  bool _bench_quiet = false;
  // beebo: cached battery reading for EnvRecord — voltage moves slowly, and
  // the ADC read (HeltecV4Board::getBattMilliVolts()) blocks for ~10-12ms
  // (settle delay + 8x analogRead), so it stays off the RX/TX hot path and is
  // refreshed only on this slow timer; noise_floor/free_heap are cheap and are
  // sampled fresh at every RX/TX capture instead.
  uint16_t _cached_batt_mv = 0;
  // beebo: true once the first post-boot VBAT sample has been taken and
  // deliberately discarded as the trend anchor -- it lands right after
  // boot's heavy load, before the battery has settled, so it's charted
  // like any other sample but not used to seed _cached_batt_mv.
  bool _batt_boot_settled = false;
  // beebo: two-stage sample scheduling (see updateBattTrend()) -- _next_batt_trigger
  // opens the radioIsIdle()-gated window, _next_batt_deadline is the unconditional
  // fallback if the radio never goes idle in time.
  unsigned long _next_batt_trigger = 0;
  unsigned long _next_batt_deadline = 0;
  // beebo: millis() timestamp of the last RX/TX event, feeds radioIsIdle().
  uint32_t _last_radio_active_ms = 0;
  // beebo: battery charge-trend state machine (BATT_STATE_*, Beebo.cpp) —
  // updated alongside _cached_batt_mv on the same slow resample tick.
  uint8_t  _batt_state = 0;  // BATT_STATE_INIT
  // beebo: cached "slow stats" for the status commands — both the flash-storage
  // usage (usedBytes() is a live block-scan) and the MCU temperature (4 averaged
  // sensor conversions, ~300 ms). loop() refreshes them on a slow cadence off the
  // hot path so CMD_GET_BATT_AND_STORAGE / STATS_TYPE_SYSTEM return the cache
  // instantly. Storage total is the partition size — constant, read once.
  uint32_t _fs_used_kb = 0;
  uint32_t _fs_total_kb = 0;
  int16_t  _mcu_temp_scaled = 0;   // °C * 10
  unsigned long _next_slowstat_refresh = 0;

  TransportKey send_scope;

  uint8_t cmd_frame[OTA_CHUNK_SIZE + 1];  // large enough for OTA chunks on WiFi/USB
  uint8_t out_frame[MAX_SEND_FRAME_SIZE + 1];  // beebo: grows under BULK_XFER; else == MAX_FRAME_SIZE+1
  CayenneLPP telemetry;

  // beebo: per-session bulk-transfer state (negotiated via CMD_SET_XFER_CAPS).
  // Reset to legacy defaults on every connect (CMD_DEVICE_QUERY) so a plain app
  // that never negotiates can never inherit a large-frame / streaming grant.
  uint16_t _app_max_tx;   // max frame size we may send this app (>= MAX_FRAME_SIZE)
  bool     _app_stream;   // app accepts back-to-back streamed bulk reads
  struct { bool active; uint32_t after_seq; uint32_t next; bool first_page; bool reset; } _monread;  // in-flight monitor-ring stream

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];

    bool isChannelMsg() const;
  };
  int offline_queue_len;
  Frame offline_queue[OFFLINE_QUEUE_SIZE];

  struct AckTableEntry {
    unsigned long msg_sent;
    uint32_t ack;
    uint32_t timeout_ms;  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- this
                          // entry's own est_timeout, so checkAckTableTimeouts()
                          // can judge each in-flight send against its own
                          // deadline (flood/direct/hop-count all differ),
                          // not one shared scalar.
    uint32_t tx_pkt_hash; // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- Packet::
                          // calculateMonRingHash() of the original DM packet
                          // (BaseChatMesh::sendMessage()'s new out-param) --
                          // the correlation key EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT needs to
                          // reference the origin MON_TX record.
    ContactInfo* contact;
  };
  #define EXPECTED_ACK_TABLE_SIZE 8
  AckTableEntry expected_ack_table[EXPECTED_ACK_TABLE_SIZE]; // circular table
  int next_ack_idx;
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- incremented at the insert site
  // when the slot about to be reused still has ack != 0 (a real send that
  // hasn't been resolved success or failure yet) -- the table wrapped faster
  // than EXPECTED_ACK_TABLE_SIZE sends could resolve, silently losing track
  // of that earlier send's eventual outcome. A starvation event, not a
  // no-op; distinct from ack_timeout_count (which only counts sends that DID
  // get a chance to time out properly).
  uint32_t ack_overflow_count;

  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- see MonRing.h's
  // EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT comment for the data[] layout. Called from the two
  // places a normal verdict is reached: processAck() match
  // (BeeboCompanion.cpp), checkAckTableTimeouts()'s sweep. The table's own
  // overflow-detection point at its insert site uses emitAckOverflowEvent()
  // instead (see MonRing.h's EVENT_ACK_OVERFLOW comment for why).
  void emitAckResultEvent(uint8_t verdict, uint32_t pkt_hash, uint32_t age_ms);
  void emitAckOverflowEvent(uint32_t pkt_hash, uint32_t age_ms);

  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- sweeps every expected_ack_table
  // slot (not just the single most-recent send BaseChatMesh's own
  // txt_send_timeout tracks) and counts+evicts any whose own timeout_ms has
  // elapsed with no ACK. Called once per loopCompanion() tick. This is also
  // the natural hook for a future "flag failed, decide whether to retransmit"
  // step -- the entry's contact/ack are still readable here, right before
  // the slot is cleared.
  void checkAckTableTimeouts();

public:
  uint32_t getAckOverflowCount() const { return ack_overflow_count; }
private:

  #define ADVERT_PATH_TABLE_SIZE   16
  AdvertPath advert_paths[ADVERT_PATH_TABLE_SIZE]; // circular table

  // beebo: direct-neighbour table (see NeighbourInfo). Evict-oldest on overflow.
  NeighbourInfo neighbours[MAX_NEIGHBOURS];
  // `pubkey`/`pubkey_len`: 1-32 bytes of the sender's prefix, as available at
  // the RX site (full for adverts, partial for control/discover responses).
  // `name`==NULL means "no advert info" — an existing slot's name/type/
  // location/advert_timestamp are left untouched (a discover hit doesn't
  // regress an already-adverted neighbour back to unknown).
  void putNeighbour(const uint8_t* pubkey, uint8_t pubkey_len, uint32_t advert_timestamp,
                    int8_t snr, uint8_t type, const char* name, int32_t lat, int32_t lon);
};

extern Beebo beebo;
