#pragma once

#include <helpers/IdentityStore.h>
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>
#include "BeeboBoardPrefs.h"
#include "BeeboAbi.h"

// beebo: BeeboPrefs unification (SETTINGS_REFACTOR.md Part 1) -- forward
// declared only. DataStore.h takes it exclusively by reference/pointer, and
// BeeboPrefs.h can only be included after Beebo.h's ComPrefs alias trick
// (#define NodePrefs ComPrefs / #include <helpers/CommonCLI.h> / #undef
// NodePrefs), which happens after DataStore.h's own #include in Beebo.h.
// DataStore.cpp includes "Beebo.h" (not just this header) to see the full
// definition.
struct BeeboPrefs;

class DataStoreHost {
public:
  virtual bool onContactLoaded(const ContactInfo& contact) =0;
  virtual bool getContactForSave(uint32_t idx, ContactInfo& contact) =0;
  virtual bool onChannelLoaded(uint8_t channel_idx, const ChannelDetails& ch) =0;
  virtual bool getChannelForSave(uint8_t channel_idx, ChannelDetails& ch) =0;
};

class DataStore {
  FILESYSTEM* _fs;
  FILESYSTEM* _fsExtra;
  mesh::RTCClock* _clock;
  IdentityStore identity_store;
  BeeboAbi _abi;

  void loadPrefsInt(const char *filename, BeeboPrefs& prefs, double& node_lat, double& node_lon);
  void saveAbi();  // stamps tool_version/saved_at and persists _abi as-is

public:
  DataStore(FILESYSTEM& fs, mesh::RTCClock& clock);
  void begin();
  // Loaded once in begin(), before any other file -- see BeeboAbi.h. Each
  // save*Prefs() below bumps its own file's version field and persists it;
  // callers needing prefs_tlv_abi_version (e.g. to report it over
  // GET_PREFS_TLV) read it from here.
  const BeeboAbi& getAbi() const { return _abi; }
  bool formatFileSystem();
  FILESYSTEM* getPrimaryFS() const { return _fs; }
  FILESYSTEM* getSecondaryFS() const { return _fsExtra; }
  bool loadMainIdentity(mesh::LocalIdentity &identity);
  bool saveMainIdentity(const mesh::LocalIdentity &identity);
  // beebo: per-role identity (PER_ROLE_IDENTITY plan) -- role is
  // NODE_ROLE_COMPANION/NODE_ROLE_REPEATER (BeeboCompanionPrefs.h). Each
  // role's keypair lives in its own IdentityStore entry, separate from the
  // legacy shared "_main" entry that loadMainIdentity/saveMainIdentity
  // still read/write untouched (kept for rollback to pre-split firmware).
  bool loadRoleIdentity(uint8_t role, mesh::LocalIdentity &identity);
  bool saveRoleIdentity(uint8_t role, const mesh::LocalIdentity &identity);
  // Copies the legacy "_main" identity into role's own entry, if role's
  // entry doesn't exist yet and "_main" does. Returns true if a copy
  // happened. Caller decides which role (if any) is eligible to inherit
  // "_main" -- only the role the node was already operating as before
  // the split, so already-trusted peers for that role see no pubkey
  // change; the other role must always start from a fresh identity.
  bool migrateLegacyIdentity(uint8_t role, mesh::LocalIdentity &identity);
  // beebo: SETTINGS_ISOLATION -- reads the pre-beebo companion NodePrefs
  // (/new_prefs, falling back to the older /node_prefs) as a read-only
  // one-time seed. Never writes, never deletes -- stock companion_radio's
  // own copy must survive a future reflash untouched. Only meant to be
  // called once, at the true first boot before /beebo_companion exists;
  // see Beebo::begin().
  void loadLegacyNodePrefs(BeeboPrefs& prefs, double& node_lat, double& node_lon);
  // beebo: NodePrefs (companion's radio/node settings, now folded into
  // BeeboPrefs -- see BeeboPrefs.h) is persisted folded into
  // /beebo_companion alongside the rest of BeeboPrefs's companion-relevant
  // bases, instead of its own /new_prefs file shared with stock
  // companion_radio -- see beebo/plans/SETTINGS_ISOLATION.md.
  // loadBeeboCompanionPrefs() returns true if /beebo_companion already
  // held a fully migrated NodePrefs copy -- NOT simply whether the file
  // existed, since /beebo_companion predates this fold-in and a real
  // device's existing file is old-format (no NodePrefs tail at all). False
  // means the caller should seed via loadLegacyNodePrefs().
  // `board` is only written by this function on a device below
  // COMPANION_PREFS_VERSION (BeeboAbi.h): role/board_password/board_name/
  // battery-ADC fields used to be echoed into /beebo_companion's own
  // on-disk layout too, read back here as a one-time migration seed for a
  // pre-/beebo_board device. /beebo_board is the sole authoritative store
  // for them once companion_prefs_version reaches COMPANION_PREFS_VERSION
  // -- this function no longer reads or writes them at all past that
  // point, `board` stays untouched.
  bool loadBeeboCompanionPrefs(BeeboPrefs& prefs, BeeboBoardPrefs& board, double& node_lat, double& node_lon);
  void saveBeeboCompanionPrefs(const BeeboPrefs& prefs, const BeeboBoardPrefs& board, double node_lat, double node_lon);
  // BeeboBoardPrefs's own file (role, board_password, board_name, and the
  // battery/ADC fields) -- the sole authoritative store for all of them,
  // genuinely untouched by the role-switch park/load handoff, distinct
  // from /beebo_companion and /beebo_repeater. Loaded first at boot
  // (Beebo::begin()), before either role's own file, so the role to
  // activate is known up front. Returns true if /beebo_board already
  // existed -- false means either these fields predate this file entirely
  // (a pre-refactor device) or the file predates the battery/ADC fields;
  // either way the caller should treat whatever loadBeeboCompanionPrefs's/
  // loadBeeboRepeaterPrefs's own legacy tail-read left in `board`'s fields
  // as the one-time migration seed, then call saveBeeboBoardPrefs() once
  // (after both roles have loaded -- see Beebo::begin()'s own comment on
  // this ordering) to persist it.
  bool loadBeeboBoardPrefs(BeeboBoardPrefs& prefs);
  void saveBeeboBoardPrefs(const BeeboBoardPrefs& prefs);
  // ComPrefs (CommonCLI.h's own struct, aliased in Beebo.h, inherited by
  // BeeboPrefs only when BEEBO_ENABLE_REPEATER_ROLE is set) folds into
  // /beebo_repeater alongside BeeboPrefs's own dedup_window_ms
  // (BeeboBasePrefs), but as a raw sizeof-blob rather than field-by-field
  // -- DataStore.cpp never needs to know ComPrefs's field layout at all,
  // so CommonCLI.h stays the single source of truth for that struct's
  // shape and CommonCLI.cpp never needs touching. Caller passes
  // static_cast<[const] ComPrefs*>(&prefs) and sizeof(ComPrefs) for
  // com_prefs/com_prefs_len. Returns true if /beebo_repeater already
  // existed (so the caller knows whether a one-time seed from
  // /com_prefs, via CommonCLI's own loadPrefs(), is needed). Same
  // "existence isn't enough" caveat as loadBeeboCompanionPrefs() --
  // /beebo_repeater predates the ComPrefs fold-in too. `board` is only
  // written by this function on a device below REPEATER_PREFS_VERSION,
  // same one-time migration-seed role as loadBeeboCompanionPrefs's own
  // `board` param -- /beebo_board is the sole authoritative store for the
  // battery/ADC fields once repeater_prefs_version reaches
  // REPEATER_PREFS_VERSION.
  bool loadBeeboRepeaterPrefs(BeeboPrefs& prefs, BeeboBoardPrefs& board, void* com_prefs, size_t com_prefs_len);
  void saveBeeboRepeaterPrefs(const BeeboPrefs& prefs, const BeeboBoardPrefs& board, const void* com_prefs, size_t com_prefs_len);
  void loadContacts(DataStoreHost* host);
  void saveContacts(DataStoreHost* host, bool (*filter)(const ContactInfo& c) = NULL);
  void loadChannels(DataStoreHost* host);
  void saveChannels(DataStoreHost* host);
  uint8_t getBlobByKey(const uint8_t key[], int key_len, uint8_t dest_buf[]);
  bool putBlobByKey(const uint8_t key[], int key_len, const uint8_t src_buf[], uint8_t len);
  bool deleteBlobByKey(const uint8_t key[], int key_len);
  File openRead(const char* filename);
  File openRead(FILESYSTEM* fs, const char* filename);
  bool removeFile(const char* filename);
  bool removeFile(FILESYSTEM* fs, const char* filename);
  uint32_t getStorageUsedKb() const;
  uint32_t getStorageTotalKb() const;

private:
  FILESYSTEM* _getContactsChannelsFS() const { if (_fsExtra) return _fsExtra; return _fs;};
};
