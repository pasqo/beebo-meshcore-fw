// beebo: repeater-role begin()/loop() path, split out of Beebo.cpp so a
// companion boot never touches this file's code at all -- companion_radio's
// own begin()/loop() never had any of this, and running it unconditionally
// regardless of role would push companion boots past Dispatcher::begin()'s
// 8-second RX-timeout budget.

#include "Beebo.h"

#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>
#include <helpers/ProfileLog.h>

// beebo: entirely absent
// before, for either role. simple_repeater's own periodic local/flood
// self-advert scheduling, gated to repeater role only in Beebo::loop()'s
// dispatch -- companion has no periodic self-advert concept at all (its
// "advert" is app/CLI-triggered only), so this file's code never runs when
// node.role is companion. The flood branch reuses companion's own
// default_scope/sendFloodScoped infra (shared with the flood-scoping path), same
// as every other flood-send path in Beebo.cpp.
//
// skip_radio: Beebo::loop()'s OTA-priority/bench-quiet gate on whether to run
// the radio dispatch pass this tick. Repeater calls Mesh::loop() directly
// here rather than BaseChatMesh::loop() -- that base class's own loop() also
// bundles in companion-shaped ACK-timeout/self-message-loopback bookkeeping
// (txt_send_timeout/_pendingLoopback), both private to BaseChatMesh, so
// there's no way to selectively keep just the shared Mesh::loop() part from
// outside it without editing that shared header. Tradeoff accepted: a
// self-addressed CMD_SEND_TXT_MSG over a repeater's binary connection won't
// loop back to itself. The advert-timer logic below is intentionally NOT
// gated by skip_radio -- it already wasn't, before this split. It does,
// however, need to run AFTER checkSerialInterface(), same as it always has
// (radio-loop, then checkSerialInterface(), then advert-timer check) -- so
// checkSerialInterface() is called from here now too, not from Beebo::loop().
#if BEEBO_ENABLE_REPEATER_ROLE
void Beebo::loopRepeater(bool skip_radio) {
  if (!skip_radio) {
    Mesh::loop();
  }
  checkSerialInterface();
  if (next_flood_advert && millisHasNowPassed(next_flood_advert)) {
    mesh::Packet* pkt = createSelfAdvertPacket();
    if (pkt) {
      TransportKey default_scope;
      getDefaultScope(NODE_ROLE_REPEATER, default_scope);
      sendFloodScoped(default_scope, pkt, 0);
    }
    updateFloodAdvertTimer(); // schedule next flood advert
    updateAdvertTimer();      // also schedule local advert (so they don't overlap)
  } else if (next_local_advert && millisHasNowPassed(next_local_advert)) {
    advert();
    updateAdvertTimer(); // schedule next local advert
  }

  // beebo: CommonCLI fallback's 'tempradio' two-stage timer -- see
  // applyTempRadioParams()'s own comment.
  if (_temp_set_radio_at && millisHasNowPassed(_temp_set_radio_at)) {
    _temp_set_radio_at = 0;
    radio_driver.setParams(_temp_pending_freq, _temp_pending_bw, _temp_pending_sf, _temp_pending_cr);
  }
  if (_temp_revert_radio_at && millisHasNowPassed(_temp_revert_radio_at)) {
    _temp_revert_radio_at = 0;
    radio_driver.setParams(_role_state->prefs.freq, _role_state->prefs.bw, _role_state->prefs.sf, _role_state->prefs.cr);
  }
}

// beebo: the per-role state store -- repeater's full persisted state
// (ACL, region map, /beebo_repeater prefs) is loaded eagerly at boot now,
// for every compiled-in role regardless of which is live (Beebo::begin()'s
// loadRoleState(NODE_ROLE_REPEATER) call, Beebo.cpp), replacing the old
// lazy load-on-first-repeater-entry model this function used to implement
// itself. beginRepeater() (called from begin() only `if (_is_repeater)`,
// and from the SET_NODE_ROLE handlers on every switch into repeater) is
// now just the live-session part: (re-)arm the advert timers, unconditional
// on every entry into repeater so a stale deadline from a much earlier
// load/switch never fires immediately.
void Beebo::beginRepeater() {
  updateAdvertTimer();
  updateFloodAdvertTimer();
}

#endif // BEEBO_ENABLE_REPEATER_ROLE

// beebo: mirrors stock simple_repeater's begin()/onDefaultRegionChanged()
// combined -- compute fresh each call instead of caching, see this
// function's own declaration in Beebo.h for why. Declared/defined
// unconditionally (region_map itself always exists, like _role_state->prefs)
// since sendFloodReply() (Beebo.cpp, always compiled) calls it even though
// the NODE_ROLE_REPEATER branch is only ever reachable at runtime from
// repeater-role code.
void Beebo::getDefaultScope(uint8_t role, TransportKey& out) {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (role == NODE_ROLE_REPEATER) {
    RegionEntry* r = region_map.getDefaultRegion();
    if (r) {
      region_map.getTransportKeysFor(*r, &out, 1);
      return;
    }
    memset(out.key, 0, sizeof(out.key));
    return;
  }
#endif
  memcpy(out.key, role_state_store[role].prefs.default_scope_key, sizeof(out.key));
}

// beebo: unlike allowPacketForward()'s per-packet
// fields, these are only consulted here (loop()'s once-per-interval check,
// minutes/hours apart), so reading straight off _role_state->prefs is fine. Not
// themselves gated: PREFS_TLV_FIELDS' tlvSetAdvertInterval/
// tlvSetFloodAdvertInterval (below, also ungated per that table's own
// convention) call these unconditionally, so they must always link even
// though _role_state->prefs itself only exists when BEEBO_ENABLE_REPEATER_ROLE is set.
void Beebo::updateAdvertTimer() {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (_role_state->prefs.advert_interval > 0) { // schedule local advert timer
    next_local_advert = futureMillis(((uint32_t)_role_state->prefs.advert_interval) * 2 * 60 * 1000);
    return;
  }
#endif
  next_local_advert = 0; // stop the timer
}

void Beebo::updateFloodAdvertTimer() {
#if BEEBO_ENABLE_REPEATER_ROLE
  if (_role_state->prefs.flood_advert_interval > 0) { // schedule flood advert timer
    next_flood_advert = futureMillis(((uint32_t)_role_state->prefs.flood_advert_interval) * 60 * 60 * 1000);
    return;
  }
#endif
  next_flood_advert = 0; // stop the timer
}

// beebo: table-driven PREFS_TLV_FIELDS -- CUSTOM get_raw/set_raw for every
// ComPrefs-backed ("repeater.*") field, since every one of them needs more
// than a plain store: either a value transform (AGC_RESET_INTERVAL's
// stored-quarter-seconds, ADVERT_INTERVAL's asymmetric */`raw store on
// write -- see its individual SET_ADVERT_INTERVAL comment in Beebo.cpp),
// validation (LOOP_DETECT rejects out-of-range), a timer rearm
// (FLOOD_ADVERT_INTERVAL/ADVERT_INTERVAL), or (REPEAT_MODE) an inverted
// on-disk sense (disable_fwd vs. the TLV's repeat=on/off sense). Each of
// these is called by BOTH this table's SET_PREFS_TLV path AND the matching
// individual BEEBO_CMD_SET_X handler in Beebo.cpp -- one implementation, so
// they can't drift apart. set_raw/set_str just mark _role_state->prefs.dirty (not
// flush themselves) -- the caller flushes once after every triplet in a
// SET_PREFS_TLV payload is applied, or once immediately for a single
// individual SET_* call, same batching contract _role_state->prefs's own dirty bit
// already has everywhere else.
// beebo: the role-targeted path -- ALL of the ComPrefs-backed
// getters/setters below target repeater's own slot explicitly
// (role_state_store[NODE_ROLE_REPEATER] + persistRoleSlot(), see that
// function's comment in Beebo.cpp), NOT self->_role_state->prefs (whichever
// role is currently live) -- fixed 2026-08-10, same bug class as
// tlvSetName/tlvGet+SetOwnerInfo below and the rxdelay/dedup_window
// pair above: a `beebo settings repeater.*` write issued while companion was
// the live role used to silently land in companion's own NodePrefs slot
// instead (see BUGS.md, kbase/SETTINGS_REFACTOR.md's Known gaps).
uint32_t Beebo::tlvGetRepeatMode(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.disable_fwd ? 0 : 1;
#else
  return 0;
#endif
}
bool Beebo::tlvSetRepeatMode(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.disable_fwd = raw ? 0 : 1;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

static uint32_t _floatBits(float v) { uint32_t b; memcpy(&b, &v, 4); return b; }
static float _bitsFloat(uint32_t b) { float v; memcpy(&v, &b, 4); return v; }

uint32_t Beebo::tlvGetAgcResetInterval(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return ((uint32_t)self->role_state_store[role].prefs.agc_reset_interval) * 4;
#else
  return 0;
#endif
}
bool Beebo::tlvSetAgcResetInterval(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.agc_reset_interval = (uint8_t)(raw / 4);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetAdvertInterval(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return ((uint32_t)self->role_state_store[role].prefs.advert_interval) * 2;
#else
  return 0;
#endif
}
bool Beebo::tlvSetAdvertInterval(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: stores raw sub[1] directly, NOT raw/2 -- see the individual
  // SET_ADVERT_INTERVAL handler's comment (a prior double-division bug).
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.advert_interval = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  // beebo: updateAdvertTimer() reads _role_state->prefs (the LIVE role) --
  // a genuine re-arm when repeater is live, a harmless no-op re-apply of
  // the live (non-repeater) role's own unrelated advert_interval otherwise,
  // same reasoning as tlvSetDedupWindow's pushActiveDedupWindow()
  // call above.
  self->updateAdvertTimer();
#endif
  return true;
}

uint32_t Beebo::tlvGetTxDelayFactor(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return _floatBits(self->role_state_store[role].prefs.tx_delay_factor);
#else
  return 0;
#endif
}
bool Beebo::tlvSetTxDelayFactor(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.tx_delay_factor = _bitsFloat(raw);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetDirectTxDelayFactor(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return _floatBits(self->role_state_store[role].prefs.direct_tx_delay_factor);
#else
  return 0;
#endif
}
bool Beebo::tlvSetDirectTxDelayFactor(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.direct_tx_delay_factor = _bitsFloat(raw);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetAllowReadOnly(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.allow_read_only;
#else
  return 0;
#endif
}
bool Beebo::tlvSetAllowReadOnly(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.allow_read_only = raw ? 1 : 0;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetLoopDetect(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.loop_detect;
#else
  return 0;
#endif
}
bool Beebo::tlvSetLoopDetect(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw > 3) return false;
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.loop_detect = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetFloodAdvertInterval(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.flood_advert_interval;
#else
  return 0;
#endif
}
bool Beebo::tlvSetFloodAdvertInterval(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.flood_advert_interval = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
  // beebo: see tlvSetAdvertInterval's comment above -- same live-vs-target
  // reasoning applies to this timer rearm.
  self->updateFloodAdvertTimer();
#endif
  return true;
}

uint32_t Beebo::tlvGetInterferenceThreshold(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.interference_threshold;
#else
  return 0;
#endif
}
bool Beebo::tlvSetInterferenceThreshold(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.interference_threshold = (uint8_t)(raw > 9 ? 9 : raw);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetFloodMax(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.flood_max;
#else
  return 0;
#endif
}
bool Beebo::tlvSetFloodMax(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.flood_max = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetFloodMaxUnscoped(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.flood_max_unscoped;
#else
  return 0;
#endif
}
bool Beebo::tlvSetFloodMaxUnscoped(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.flood_max_unscoped = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

// beebo: repeater keeps its own independent copy of rx_delay_base/
// airtime_factor (seeded from companion's shared value only for a
// genuinely brand-new /com_prefs, see loadRoleState()'s repeater branch
// in Beebo.cpp). Deliberately separate opcodes from CMD_GET/SET_TUNING_PARAMS
// (the stock upstream companion pair, which stays byte-for-byte unchanged
// and keeps reading/writing the shared companion
// _role_state->prefs.rx_delay_base/airtime_factor) -- these are only
// reachable via `beebo settings repeater.routing.*`/BEEBO_CMD_*, never the
// standard MeshCore app protocol. Targets the given role's own slot
// explicitly (see persistRoleSlot()'s comment, Beebo.cpp), NOT
// self->_role_state->prefs (whichever role is currently live).
uint32_t Beebo::tlvGetRxDelayBase(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return _floatBits(self->role_state_store[role].prefs.rx_delay_base);
#else
  return 0;
#endif
}
bool Beebo::tlvSetRxDelayBase(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.rx_delay_base = _bitsFloat(raw);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

uint32_t Beebo::tlvGetAirtimeFactor(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return _floatBits(self->role_state_store[role].prefs.airtime_factor);
#else
  return 0;
#endif
}
bool Beebo::tlvSetAirtimeFactor(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.airtime_factor = _bitsFloat(raw);
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

// beebo: repeater's own independent dedup live-eviction window, ms, own
// file (see BeeboRepeaterPrefs.h). 0 is a real, literal value -- disables
// live-eviction counting entirely (SimpleMeshTables::hasSeen()) -- not a
// "reset to default" sentinel; valid range is [0, DEDUP_WINDOW_MAX_MS],
// rejected (false, caller writes ERR) outside that. Targets the given
// role's own slot explicitly (see persistRoleSlot()'s comment, Beebo.cpp),
// NOT self->_role_state->prefs (whichever role is currently live).
// pushActiveDedupWindow() still reads _role_state->prefs (the live role)
// unconditionally, so it's a harmless no-op re-apply of the live role's
// already-current value when this write targets a non-live repeater slot,
// and a genuine re-apply when repeater is the one that's live.
uint32_t Beebo::tlvGetDedupWindow(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.dedup_window_ms;
#else
  return 0;
#endif
}
bool Beebo::tlvSetDedupWindow(Beebo* self, uint8_t role, uint32_t raw) {
  if (raw > DEDUP_WINDOW_MAX_MS) return false;
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: MON_SETTING, reusing this field's own PrefsTlvKey (20) as the
  // SETTING_* key -- see MonRing.h's enum comment. Placed here (inside the
  // tlv setter) rather than the command handler so it fires from every
  // entry point uniformly, same as node_role's own appendSettingChangedEvent
  // call living inside setNodeRole() rather than each of its callers.
  BeeboRoleState& slot = self->role_state_store[role];
  self->appendSettingChangedEvent(PREFS_TLV_DEDUP_WINDOW,
                                   slot.prefs.dedup_window_ms, raw, EVENT_SOURCE_BINARY);
  slot.prefs.dedup_window_ms = raw;
  persistRoleSlot(self, role, slot);
  self->pushActiveDedupWindow();
#endif
  return true;
}

uint32_t Beebo::tlvGetFloodMaxAdvert(Beebo* self, uint8_t role) {
#if BEEBO_ENABLE_REPEATER_ROLE
  return self->role_state_store[role].prefs.flood_max_advert;
#else
  return 0;
#endif
}
bool Beebo::tlvSetFloodMaxAdvert(Beebo* self, uint8_t role, uint32_t raw) {
#if BEEBO_ENABLE_REPEATER_ROLE
  BeeboRoleState& slot = self->role_state_store[role];
  slot.prefs.flood_max_advert = (uint8_t)raw;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

// beebo: string fields -- owner_info and repeater_name
// (role_state_store[NODE_ROLE_REPEATER].prefs.node_name, same struct field
// companion's own "name" upstream key would use if CommonCLI ever served
// companion role, which it never does here). Both always target repeater's
// own slot explicitly, regardless of which role is currently live -- fixed
// 2026-08-10 (previously used self->_role_state->prefs, see this file's own
// top-of-block comment).
int Beebo::tlvGetOwnerInfo(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  const char* info = self->role_state_store[role].prefs.owner_info;
  size_t n = strnlen(info, sizeof(self->role_state_store[role].prefs.owner_info) - 1);
  if (n > max_len) n = max_len;
  memcpy(out, info, n);
  return (int)n;
#else
  return 0;
#endif
}
bool Beebo::tlvSetOwnerInfo(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
#if BEEBO_ENABLE_REPEATER_ROLE
  // beebo: silently truncates (never rejects) an overlong value.
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.owner_info) - 1) len = sizeof(slot.prefs.owner_info) - 1;
  memcpy(slot.prefs.owner_info, in, len);
  slot.prefs.owner_info[len] = 0;
  persistRoleSlot(self, role, slot);
#endif
  return true;
}

// beebo: not gated by `#if BEEBO_ENABLE_REPEATER_ROLE` (see
// tlvGetMultiAcks's comment in Beebo.cpp for why): also called with
// NODE_ROLE_COMPANION.
int Beebo::tlvGetName(Beebo* self, uint8_t role, uint8_t* out, size_t max_len) {
  // beebo: getRoleName(role) always reads that role's own resident slot
  // directly (role_state_store[role] -- see its own comment in Beebo.h),
  // regardless of which role is currently live; empty if that role's name
  // was never set, no fallback.
  const char* name = self->getRoleName(role);
  size_t n = strlen(name);
  if (n > max_len) n = max_len;
  memcpy(out, name, n);
  return (int)n;
}
bool Beebo::tlvSetName(Beebo* self, uint8_t role, const uint8_t* in, size_t len) {
  // beebo: silently truncates (never rejects) an overlong value, matching
  // the individual SET_REPEATER_NAME/SET_COMPANION_NAME handlers' own
  // min(len, sizeof-1) clamp.
  BeeboRoleState& slot = self->role_state_store[role];
  if (len > sizeof(slot.prefs.node_name) - 1) len = sizeof(slot.prefs.node_name) - 1;
  memcpy(slot.prefs.node_name, in, len);
  slot.prefs.node_name[len] = 0;
  persistRoleSlot(self, role, slot);
  return true;
}

const Beebo::PrefsTlvField Beebo::PREFS_TLV_FIELDS[] = {
  { PREFS_TLV_REPEAT_MODE,            TLV_U32,    tlvGetRepeatMode,            tlvSetRepeatMode,            nullptr, nullptr },
  { PREFS_TLV_TXDELAY_FACTOR,         TLV_FLOAT,  tlvGetTxDelayFactor,         tlvSetTxDelayFactor,         nullptr, nullptr },
  { PREFS_TLV_DIRECT_TXDELAY_FACTOR,  TLV_FLOAT,  tlvGetDirectTxDelayFactor,   tlvSetDirectTxDelayFactor,   nullptr, nullptr },
  { PREFS_TLV_ALLOW_READ_ONLY,        TLV_U32,    tlvGetAllowReadOnly,         tlvSetAllowReadOnly,         nullptr, nullptr },
  { PREFS_TLV_AGC_RESET_INTERVAL,     TLV_U32,    tlvGetAgcResetInterval,      tlvSetAgcResetInterval,      nullptr, nullptr },
  { PREFS_TLV_LOOP_DETECT,            TLV_U32,    tlvGetLoopDetect,            tlvSetLoopDetect,            nullptr, nullptr },
  { PREFS_TLV_FLOOD_ADVERT_INTERVAL,  TLV_U32,    tlvGetFloodAdvertInterval,   tlvSetFloodAdvertInterval,   nullptr, nullptr },
  { PREFS_TLV_INTERFERENCE_THRESHOLD, TLV_U32,    tlvGetInterferenceThreshold, tlvSetInterferenceThreshold, nullptr, nullptr },
  { PREFS_TLV_ADVERT_INTERVAL,        TLV_U32,    tlvGetAdvertInterval,        tlvSetAdvertInterval,        nullptr, nullptr },
  { PREFS_TLV_FLOOD_MAX,              TLV_U32,    tlvGetFloodMax,              tlvSetFloodMax,              nullptr, nullptr },
  { PREFS_TLV_FLOOD_MAX_UNSCOPED,     TLV_U32,    tlvGetFloodMaxUnscoped,      tlvSetFloodMaxUnscoped,      nullptr, nullptr },
  { PREFS_TLV_FLOOD_MAX_ADVERT,       TLV_U32,    tlvGetFloodMaxAdvert,        tlvSetFloodMaxAdvert,        nullptr, nullptr },
  { PREFS_TLV_OWNER_INFO,             TLV_STRING, nullptr, nullptr, tlvGetOwnerInfo,    tlvSetOwnerInfo },
  { PREFS_TLV_NAME,          TLV_STRING, nullptr, nullptr, tlvGetName, tlvSetName },
  { PREFS_TLV_WIFI_SSID,              TLV_STRING, nullptr, nullptr, tlvGetWifiSsid,     tlvSetWifiSsid },
  { PREFS_TLV_TRANSPORT_CONFIG,       TLV_U32,    tlvGetTransportConfig, tlvSetTransportConfig, nullptr, nullptr },
  { PREFS_TLV_MONRING_CONFIG,         TLV_U32,    tlvGetMonringConfig,   tlvSetMonringConfig,   nullptr, nullptr },
  { PREFS_TLV_RXDELAY,       TLV_FLOAT,  tlvGetRxDelayBase,   tlvSetRxDelayBase,   nullptr, nullptr },
  { PREFS_TLV_AIRTIME,       TLV_FLOAT,  tlvGetAirtimeFactor, tlvSetAirtimeFactor, nullptr, nullptr },
  { PREFS_TLV_DEDUP_WINDOW,  TLV_U32,    tlvGetDedupWindow,   tlvSetDedupWindow,   nullptr, nullptr },
  { PREFS_TLV_MULTI_ACKS,     TLV_U32,    tlvGetMultiAcks,     tlvSetMultiAcks,     nullptr, nullptr },
  { PREFS_TLV_PATH_HASH_MODE, TLV_U32,    tlvGetPathHashMode,  tlvSetPathHashMode,  nullptr, nullptr },
  { PREFS_TLV_LAT,            TLV_U32,    tlvGetLat,           tlvSetLat,           nullptr, nullptr },
  { PREFS_TLV_LON,            TLV_U32,    tlvGetLon,           tlvSetLon,           nullptr, nullptr },
  { PREFS_TLV_ADV_LOC_POLICY,          TLV_U32,    tlvGetAdvLocPolicy,  tlvSetAdvLocPolicy,  nullptr, nullptr },
  { PREFS_TLV_BLE_PIN,                 TLV_U32,    tlvGetBlePin,        tlvSetBlePin,        nullptr, nullptr },
  { PREFS_TLV_WIFI_PWD,                TLV_STRING, nullptr, nullptr, tlvGetWifiPwdSetStr, tlvSetWifiPwd },
  { PREFS_TLV_RADIO_FEM_RXGAIN, TLV_U32,   tlvGetRadioFemRxgain, tlvSetRadioFemRxgain, nullptr, nullptr },
  { PREFS_TLV_RADIO_RXGAIN,     TLV_U32,   tlvGetRadioRxgain,    tlvSetRadioRxgain,    nullptr, nullptr },
  { PREFS_TLV_ADC_MULTIPLIER,   TLV_FLOAT, tlvGetAdcMultiplier,  tlvSetAdcMultiplier,  nullptr, nullptr },
  { PREFS_TLV_ADC_RESOLUTION,   TLV_U32,   tlvGetAdcResolution,  tlvSetAdcResolution,  nullptr, nullptr },
  { PREFS_TLV_BATT_PRESENT,        TLV_U32, tlvGetBattPresent,       tlvSetBattPresent,       nullptr, nullptr },
  { PREFS_TLV_BATT_SAMPLE_PERIOD,  TLV_U32, tlvGetBattSamplePeriod,  tlvSetBattSamplePeriod,  nullptr, nullptr },
  { PREFS_TLV_BATT_SAMPLE_WINDOW,  TLV_U32, tlvGetBattSampleWindow,  tlvSetBattSampleWindow,  nullptr, nullptr },
  { PREFS_TLV_BATT_CHARGED_MV,     TLV_U32, tlvGetBattChargedMv,     tlvSetBattChargedMv,     nullptr, nullptr },
  { PREFS_TLV_IDLE_MARGIN,         TLV_U32, tlvGetIdleMargin,        tlvSetIdleMargin,        nullptr, nullptr },
  { PREFS_TLV_MANUAL_ADD_CONTACTS, TLV_U32, tlvGetManualAddContacts, tlvSetManualAddContacts, nullptr, nullptr },
  { PREFS_TLV_AUTOADD_CONFIG,      TLV_U32, tlvGetAutoaddConfig,     tlvSetAutoaddConfig,     nullptr, nullptr },
  { PREFS_TLV_REPEATER_PASSWORD, TLV_STRING, nullptr, nullptr, tlvGetRepeaterPasswordSetStr, tlvSetRepeaterPassword },
  { PREFS_TLV_GUEST_PASSWORD,    TLV_STRING, nullptr, nullptr, tlvGetGuestPasswordSetStr,    tlvSetGuestPassword },
  { PREFS_TLV_BOARD_NAME,        TLV_STRING, nullptr, nullptr, tlvGetBoardName,              tlvSetBoardName },
  { PREFS_TLV_OWNER_PASSWORD,    TLV_STRING, nullptr, nullptr, tlvGetOwnerPasswordSetStr,    tlvSetOwnerPassword },
  { PREFS_TLV_MONRING_EVENT_MASK, TLV_U32,   tlvGetMonringEventMask, tlvSetMonringEventMask, nullptr, nullptr },
};
const size_t Beebo::PREFS_TLV_FIELD_COUNT = sizeof(PREFS_TLV_FIELDS) / sizeof(PREFS_TLV_FIELDS[0]);

// beebo: GET_PREFS_TLV. Walks PREFS_TLV_FIELDS, encoding each as a
// [key:1][len:1][value] triplet via its get_raw/get_str -- the same
// function the matching individual GET_* handler calls, so this always
// matches those handlers' replies exactly. A field that doesn't fit in the
// remaining space is silently skipped (caller sees fewer triplets than
// PREFS_TLV_FIELD_COUNT; none of these fields currently come close to
// overflowing MAX_FRAME_SIZE, so this is a safety margin, not a real limit).
int Beebo::encodePrefsTlv(uint8_t role, uint8_t* out, size_t max_len) {
  int pos = 0;
  for (size_t i = 0; i < PREFS_TLV_FIELD_COUNT; i++) {
    const PrefsTlvField& f = PREFS_TLV_FIELDS[i];
    if (f.type == TLV_STRING) {
      uint8_t buf[128];
      int n = f.get_str(this, role, buf, sizeof(buf));
      if (n < 0 || (size_t)(pos + 2 + n) > max_len) continue;
      out[pos++] = f.key;
      out[pos++] = (uint8_t)n;
      memcpy(&out[pos], buf, n);
      pos += n;
    } else {
      if ((size_t)(pos + 6) > max_len) continue;
      uint32_t raw = f.get_raw(this, role);
      out[pos++] = f.key;
      out[pos++] = 4;
      memcpy(&out[pos], &raw, 4);
      pos += 4;
    }
  }
  return pos;
}

// beebo: one triplet of SET_PREFS_TLV's payload, applied via the matching
// field's set_raw/set_str -- NOT flushed to flash here (the caller, this
// command's CMD_BEEBO dispatch in Beebo.cpp, flushes once after every
// triplet in the payload has been applied). An unrecognized-but-well-formed
// key still advances `pos` past its triplet (the length byte says how far
// to skip) so parsing can continue; a malformed triplet (truncated buffer,
// or -- for a TLV_U32/TLV_FLOAT field -- a length other than 4) forces
// `pos` to `len` since there's no safe way to resync mid-stream. Either way
// returns false so the caller can report the failure.
bool Beebo::applyPrefsTlvTriplet(uint8_t role, const uint8_t* in, size_t len, size_t& pos) {
  if (pos + 2 > len) { pos = len; return false; }
  uint8_t key = in[pos];
  uint8_t vlen = in[pos + 1];
  pos += 2;
  if (pos + vlen > len) { pos = len; return false; }

  const PrefsTlvField* f = nullptr;
  for (size_t i = 0; i < PREFS_TLV_FIELD_COUNT; i++) {
    if (PREFS_TLV_FIELDS[i].key == key) { f = &PREFS_TLV_FIELDS[i]; break; }
  }
  if (f == nullptr) { pos += vlen; return false; }

  if (f->type == TLV_STRING) {
    bool ok = f->set_str(this, role, &in[pos], vlen);
    pos += vlen;
    return ok;
  }
  if (vlen != 4) { pos = len; return false; }
  uint32_t raw;
  memcpy(&raw, &in[pos], 4);
  pos += 4;
  return f->set_raw(this, role, raw);
}

// beebo: mirrors BaseChatMesh::createSelfAdvert
// (src/helpers/BaseChatMesh.cpp:19-39) exactly, but ADV_TYPE_REPEATER
// instead of the hardcoded ADV_TYPE_CHAT. That helper is shared library
// code used by other targets too, so it's not edited in place -- this is
// multi_role's own role-aware alternative, picked by
// Beebo::createSelfAdvertPacket()'s role dispatch (Beebo.cpp).
#if BEEBO_ENABLE_REPEATER_ROLE
// beebo: CommonCLICallbacks implementation -- see Beebo.h's inline overrides
// for the ones simple enough to be one-liners; the rest (needing more than a
// single existing accessor/setter) live here. See
// beebo/plans/COMMONCLI_TEXT_DISPATCH.md's "CommonCLICallbacks implementation
// surface" table for what each maps onto.

// beebo: PER_ROLE_IDENTITY -- this callback is only ever reached via
// CommonCLI's "prv.key" text command, which itself is only reachable while
// repeater is the live role (see the _is_repeater-gated cli.handleCommand()
// fallthroughs in Beebo.cpp's handleCommand()). So unlike the pre-split
// shared self_id this used to mirror, a rekey here always targets the
// *repeater* identity specifically and never touches companion's -- the
// two are independent identities now. Mirrors CMD_IMPORT_PRIVATE_KEY's own
// handler (Beebo.cpp) for the repeater-targeted case: recompute ACL shared
// secrets in place instead of the old unconditional companion contacts
// reload (companion state isn't even loaded while repeater is active, and
// doesn't exist at all in a companion-role-disabled static repeater build).
void Beebo::saveIdentity(const mesh::LocalIdentity& new_id) {
  if (_store->saveRoleIdentity(_board.role, new_id)) {
    self_id = new_id;
    acl.load(_store->getPrimaryFS(), self_id);
  }
}

// beebo: matches CommonCLI.cpp's own bare "advert" (flood, via repeater's
// own RegionMap default region) / zero-hop semantics exactly -- see the
// "advert"/"advert.zerohop" text handlers in Beebo.cpp's handleCommand(),
// which this callback is otherwise unreachable behind (both keys are
// handled in that chain before ever falling through to cli.handleCommand()).
void Beebo::sendSelfAdvertisement(int delay_millis, bool flood) {
  mesh::Packet* pkt = createSelfAdvertPacket();
  if (!pkt) return;
  if (flood) {
    TransportKey default_scope;
    getDefaultScope(NODE_ROLE_REPEATER, default_scope);
    sendFloodScoped(default_scope, pkt, delay_millis);
  } else {
    sendZeroHop(pkt, delay_millis);
  }
}

// beebo: this board's NeighbourInfo shape (pubkey/pubkey_len prefix, not a
// fixed mesh::Identity) -- "hex:secs_ago:snr" line-per-neighbour format,
// newest first, bounded well inside the 160-byte reply buffer.
void Beebo::formatNeighborsReply(char* reply) {
  char* dp = reply;
  int16_t count = 0;
  NeighbourInfo* sorted[MAX_NEIGHBOURS];
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    if (neighbours[i].heard_timestamp > 0) sorted[count++] = &neighbours[i];
  }
  std::sort(sorted, sorted + count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
    return a->heard_timestamp > b->heard_timestamp;
  });
  for (int i = 0; i < count && dp - reply < 134; i++) {
    NeighbourInfo* nb = sorted[i];
    if (i > 0) *dp++ = '\n';
    char hex[10];
    mesh::Utils::toHex(hex, nb->pubkey, min((int)nb->pubkey_len, 4));
    uint32_t secs_ago = getRTCClock()->getCurrentTime() - nb->heard_timestamp;
    sprintf(dp, "%s:%lu:%d", hex, (unsigned long)secs_ago, nb->snr);
    while (*dp) dp++;
  }
  if (dp == reply) {
    strcpy(dp, "-none-");
    dp += 6;
  }
  *dp = 0;
}

// beebo: CommonCLI fallback's 'tempradio <freq> <bw> <sf> <cr> <mins>' --
// two-stage timer: apply after a short delay so the CLI reply has time to
// go out first, then revert to the persisted companion radio prefs once
// timeout_mins elapses. Ticked from loopRepeater() below, since this
// command is only reachable via the repeater-only CommonCLI fallback.
// beebo: maps onto the same clear the binary BEEBO_CMD_CLEAR_PROFILE opcode
// already does (Beebo.cpp) -- one implementation for both entry points.
void Beebo::clearStats() {
  profile_log.clear();
}

void Beebo::applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) {
  _temp_set_radio_at = futureMillis(2000);
  _temp_pending_freq = freq;
  _temp_pending_bw = bw;
  _temp_pending_sf = sf;
  _temp_pending_cr = cr;
  _temp_revert_radio_at = futureMillis(2000 + (uint32_t)timeout_mins * 60 * 1000);
}
#endif // BEEBO_ENABLE_REPEATER_ROLE

#if BEEBO_ENABLE_REPEATER_ROLE
mesh::Packet* Beebo::createRepeaterSelfAdvert(const char* name) {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len;
  {
    AdvertDataBuilder builder(ADV_TYPE_REPEATER, name);
    app_data_len = builder.encodeTo(app_data);
  }
  return createAdvert(self_id, app_data, app_data_len);
}

mesh::Packet* Beebo::createRepeaterSelfAdvert(const char* name, double lat, double lon) {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len;
  {
    AdvertDataBuilder builder(ADV_TYPE_REPEATER, name, lat, lon);
    app_data_len = builder.encodeTo(app_data);
  }
  return createAdvert(self_id, app_data, app_data_len);
}
#endif // BEEBO_ENABLE_REPEATER_ROLE

/* ------------------------------------------------------------------------
 * beebo: repeater-role inbound login/ACL request handler, reusing
 * companion's own existing stats/telemetry/neighbour-table code wherever
 * it already covers the same ground (RepeaterStats' fields, telemetry
 * gathering, GET_NEIGHBOURS adapted to companion's own richer
 * NeighbourInfo table). The TXT_TYPE_CLI_DATA / handleCommand() path is
 * deliberately excluded -- deferred until multi_role has a text
 * dispatcher at all.
 * ------------------------------------------------------------------------ */

#if BEEBO_ENABLE_REPEATER_ROLE
void Beebo::onAnonDataRecv(mesh::Packet *packet, const uint8_t *secret, const mesh::Identity &sender,
                            uint8_t *data, size_t len) {
  if (!_is_repeater) return;  // companion never accepts inbound mesh admin requests

  if (packet->getPayloadType() == PAYLOAD_TYPE_ANON_REQ) { // received an initial request by a possible admin
                                                            // client (unknown at this stage)
    uint32_t timestamp;
    memcpy(&timestamp, data, 4);

    data[len] = 0;  // ensure null terminator
    uint8_t reply_len;

    reply_path_len = -1;
    if (data[4] == 0 || data[4] >= ' ') {   // is password, ie. a login request
      reply_len = handleLoginReq(sender, secret, timestamp, &data[4], packet->isRouteFlood());
    } else if (data[4] == ANON_REQ_TYPE_REGIONS && packet->isRouteDirect()) {
      reply_len = handleAnonRegionsReq(sender, timestamp, &data[5]);
    } else if (data[4] == ANON_REQ_TYPE_OWNER && packet->isRouteDirect()) {
      reply_len = handleAnonOwnerReq(sender, timestamp, &data[5]);
    } else if (data[4] == ANON_REQ_TYPE_BASIC && packet->isRouteDirect()) {
      reply_len = handleAnonClockReq(sender, timestamp, &data[5]);
    } else {
      reply_len = 0;  // unknown/invalid request type
    }

    if (reply_len == 0) return;   // invalid request

    if (packet->isRouteFlood()) {
      // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
      mesh::Packet* path = createPathReturn(sender, secret, packet->path, packet->path_len,
                                            PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
      if (path) sendFloodReply(path, ADMIN_REQ_SERVER_RESPONSE_DELAY, packet->getPathHashSize());
    } else if (reply_path_len < 0) {
      mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
      if (reply) sendFloodReply(reply, ADMIN_REQ_SERVER_RESPONSE_DELAY, packet->getPathHashSize());
    } else {
      mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
      uint8_t path_len = ((reply_path_hash_size - 1) << 6) | (reply_path_len & 63);
      if (reply) sendDirect(reply, reply_path, path_len, ADMIN_REQ_SERVER_RESPONSE_DELAY);
    }
  }
}

uint8_t Beebo::handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data, bool is_flood) {
  ClientInfo* client = NULL;
  if (data[0] == 0) {   // blank password, just check if sender is in ACL
    client = acl.getClient(sender.pub_key, PUB_KEY_SIZE);
  }
  if (client == NULL) {
    uint8_t perms;
    if (strcmp((char *)data, _role_state->prefs.password) == 0) { // check for valid admin password
      perms = PERM_ACL_ADMIN;
    } else if (_role_state->prefs.guest_password[0] != 0 && strcmp((char *)data, _role_state->prefs.guest_password) == 0) { // check guest password
      perms = PERM_ACL_GUEST;
    } else {
      MESH_DEBUG_PRINTLN("Invalid login attempt");
      return 0;
    }

    client = acl.putClient(sender, 0);  // add to contacts (if not already known)
    if (client == NULL) return 0;  // ACL table full
    if (sender_timestamp <= client->last_timestamp) {
      MESH_DEBUG_PRINTLN("Possible login replay attack!");
      return 0;  // FATAL: client table is full -OR- replay attack
    }

    client->last_timestamp = sender_timestamp;
    client->last_activity = getRTCClock()->getCurrentTime();
    client->permissions &= ~PERM_ACL_ROLE_MASK;
    client->permissions |= perms;
    memcpy(client->shared_secret, secret, PUB_KEY_SIZE);

    if (perms != PERM_ACL_GUEST) {   // keep number of FS writes to a minimum
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
    }
  }

  if (is_flood) {
    client->out_path_len = OUT_PATH_UNKNOWN;  // need to rediscover out_path
  }

  uint32_t now = getRTCClock()->getCurrentTimeUnique();
  memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp
  reply_data[4] = RESP_SERVER_LOGIN_OK;
  reply_data[5] = 0;  // Legacy: was recommended keep-alive interval (secs / 16)
  reply_data[6] = client->isAdmin() ? 1 : 0;
  reply_data[7] = client->permissions;
  getRNG()->random(&reply_data[8], 4);   // random blob to help packet-hash uniqueness
  reply_data[12] = FIRMWARE_VER_LEVEL;

  return 13;  // reply length
}

uint8_t Beebo::handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);

    memcpy(reply_data, &sender_timestamp, 4);   // prefix with sender_timestamp, like a tag
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);     // include our clock (for easy clock sync, and packet hash uniqueness)

    return 8 + region_map.exportNamesTo((char *) &reply_data[8], sizeof(reply_data) - 12, REGION_DENY_FLOOD);   // reply length
  }
  return 0;
}

uint8_t Beebo::handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);

    memcpy(reply_data, &sender_timestamp, 4);
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);
    sprintf((char *) &reply_data[8], "%s\n%s", _role_state->prefs.node_name, _role_state->prefs.owner_info);

    return 8 + strlen((char *) &reply_data[8]);   // reply length
  }
  return 0;
}

uint8_t Beebo::handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(getRTCClock()->getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);

    memcpy(reply_data, &sender_timestamp, 4);
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);
    reply_data[8] = 0;  // features (no bridge support in multi_role; disable_fwd concept doesn't apply -- see allowPacketForward)
    return 9;   // reply length
  }
  return 0;
}

int Beebo::handleRequest(ClientInfo *sender, uint32_t sender_timestamp, uint8_t *payload, size_t payload_len) {
  memcpy(reply_data, &sender_timestamp, 4); // reflect sender_timestamp back in response packet (kind of like a 'tag')

  if (payload[0] == REQ_TYPE_GET_STATUS) {  // guests can also access this now
    RepeaterStats stats;
    // beebo: a status query already forces a live ADC read here, so use it
    // to opportunistically drive the trend classifier too if due, instead
    // of only advancing on the next idle loop() tick (same pattern as
    // companion's own STATS_TYPE_CORE handler).
    stats.batt_milli_volts = updateBattTrend(true);
    stats.curr_tx_queue_len = _mgr->getOutboundTotal();
    stats.noise_floor = (int16_t)_radio->getNoiseFloor();
    stats.last_rssi = (int16_t)radio_driver.getLastRSSI();
    stats.n_packets_recv = radio_driver.getPacketsRecv();
    stats.n_packets_sent = radio_driver.getPacketsSent();
    stats.total_air_time_secs = getTotalAirTime() / 1000;
    stats.total_up_time_secs = _ms->getMillis() / 1000;
    stats.n_sent_flood = getNumSentFlood();
    stats.n_sent_direct = getNumSentDirect();
    stats.n_recv_flood = getNumRecvFlood();
    stats.n_recv_direct = getNumRecvDirect();
    stats.err_events = _err_flags;
    stats.last_snr = (int16_t)(radio_driver.getLastSNR() * 4);
    stats.n_direct_dups = ((SimpleMeshTables *)getTables())->getNumDirectDups();
    stats.n_flood_dups = ((SimpleMeshTables *)getTables())->getNumFloodDups();
    stats.total_rx_air_time_secs = getReceiveAirTime() / 1000;
    stats.n_recv_errors = radio_driver.getPacketsRecvErrors();
    memcpy(&reply_data[4], &stats, sizeof(stats));

    return 4 + sizeof(stats); //  reply_len
  }
  if (payload[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
    uint8_t perm_mask = ~(payload[1]); // first reserved byte (of 4) is inverse mask to apply to permissions

    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);

    if ((sender->permissions & PERM_ACL_ROLE_MASK) == PERM_ACL_GUEST) {
      perm_mask = 0x00;  // just base telemetry allowed
    }
    sensors.querySensors(perm_mask, telemetry);

    float temperature = board.getMCUTemperature();
    if (!isnan(temperature)) {
      telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature);
    }

    uint8_t tlen = telemetry.getSize();
    memcpy(&reply_data[4], telemetry.getBuffer(), tlen);
    return 4 + tlen;
  }
  if (payload[0] == REQ_TYPE_GET_ACCESS_LIST && sender->isAdmin()) {
    uint8_t res1 = payload[1];   // reserved for future  (extra query params)
    uint8_t res2 = payload[2];
    if (res1 == 0 && res2 == 0) {
      uint8_t ofs = 4;
      for (int i = 0; i < acl.getNumClients() && ofs + 7 <= sizeof(reply_data) - 4; i++) {
        auto c = acl.getClientByIdx(i);
        if (c->permissions == 0) continue;  // skip deleted entries
        memcpy(&reply_data[ofs], c->id.pub_key, 6); ofs += 6;  // just 6-byte pub_key prefix
        reply_data[ofs++] = c->permissions;
      }
      return ofs;
    }
  }
  if (payload[0] == REQ_TYPE_GET_NEIGHBOURS) {
    uint8_t request_version = payload[1];
    if (request_version == 0) {
      // beebo: uses companion's own neighbours[MAX_NEIGHBOURS] table
      // (variable-length pubkey prefix); repeater role has no separate one.
      int reply_offset = 4;

      uint8_t count = payload[2];
      uint16_t offset;
      memcpy(&offset, &payload[3], 2);
      uint8_t order_by = payload[5];
      uint8_t pubkey_prefix_length = payload[6];

      if (pubkey_prefix_length > PUB_KEY_SIZE) pubkey_prefix_length = PUB_KEY_SIZE;

      int16_t neighbours_count = 0;
      NeighbourInfo* sorted_neighbours[MAX_NEIGHBOURS];
      for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        auto neighbour = &neighbours[i];
        if (neighbour->heard_timestamp > 0) {
          sorted_neighbours[neighbours_count] = neighbour;
          neighbours_count++;
        }
      }

      if (order_by == 0) {
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->heard_timestamp > b->heard_timestamp;
        });
      } else if (order_by == 1) {
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->heard_timestamp < b->heard_timestamp;
        });
      } else if (order_by == 2) {
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->snr > b->snr;
        });
      } else if (order_by == 3) {
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->snr < b->snr;
        });
      }

      int results_count = 0;
      int results_offset = 0;
      uint8_t results_buffer[130];
      for (int index = 0; index < count && index + offset < neighbours_count; index++) {
        int entry_size = pubkey_prefix_length + 4 + 1;
        if (results_offset + entry_size > (int)sizeof(results_buffer)) break;

        auto neighbour = sorted_neighbours[index + offset];
        uint32_t heard_seconds_ago = getRTCClock()->getCurrentTime() - neighbour->heard_timestamp;
        uint8_t send_prefix_len = pubkey_prefix_length < neighbour->pubkey_len ? pubkey_prefix_length : neighbour->pubkey_len;
        memset(&results_buffer[results_offset], 0, pubkey_prefix_length);
        memcpy(&results_buffer[results_offset], neighbour->pubkey, send_prefix_len); results_offset += pubkey_prefix_length;
        memcpy(&results_buffer[results_offset], &heard_seconds_ago, 4); results_offset += 4;
        memcpy(&results_buffer[results_offset], &neighbour->snr, 1); results_offset += 1;
        results_count++;
      }

      memcpy(&reply_data[reply_offset], &neighbours_count, 2); reply_offset += 2;
      memcpy(&reply_data[reply_offset], &results_count, 2); reply_offset += 2;
      memcpy(&reply_data[reply_offset], &results_buffer, results_offset); reply_offset += results_offset;

      return reply_offset;
    }
  } else if (payload[0] == REQ_TYPE_GET_OWNER_INFO) {
    sprintf((char *) &reply_data[4], "%s\n%s\n%s", FIRMWARE_VERSION, _role_state->prefs.node_name, _role_state->prefs.owner_info);
    return 4 + strlen((char *) &reply_data[4]);
  }
  return 0; // unknown command
}
#endif // BEEBO_ENABLE_REPEATER_ROLE
