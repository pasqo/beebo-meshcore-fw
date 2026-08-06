#pragma once

#include <helpers/IdentityStore.h>
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>
#include "../companion_radio/NodePrefs.h"
#include "BeeboCompanionPrefs.h"
#include "BeeboRepeaterPrefs.h"

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

  void loadPrefsInt(const char *filename, NodePrefs& prefs, double& node_lat, double& node_lon);

public:
  DataStore(FILESYSTEM& fs, mesh::RTCClock& clock);
  void begin();
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
  void loadLegacyNodePrefs(NodePrefs& prefs, double& node_lat, double& node_lon);
  // beebo: NodePrefs (companion's radio/node settings) is now persisted
  // folded into /beebo_companion alongside BeeboCompanionPrefs, instead of
  // its own /new_prefs file shared with stock companion_radio -- see
  // beebo/plans/SETTINGS_ISOLATION.md. loadBeeboCompanionPrefs() returns
  // true if /beebo_companion already held a fully migrated NodePrefs copy
  // -- NOT simply whether the file existed, since /beebo_companion
  // predates this fold-in and a real device's existing file is
  // old-format (no NodePrefs tail at all). False means the caller should
  // seed via loadLegacyNodePrefs().
  bool loadBeeboCompanionPrefs(BeeboCompanionPrefs& prefs, NodePrefs& node, double& node_lat, double& node_lon);
  void saveBeeboCompanionPrefs(const BeeboCompanionPrefs& prefs, const NodePrefs& node, double node_lat, double node_lon);
  // beebo: SETTINGS_ISOLATION -- ComPrefs (CommonCLI.h's own struct,
  // aliased in Beebo.h) folds into /beebo_repeater alongside
  // BeeboRepeaterPrefs's own fields, but as a raw sizeof-blob rather than
  // field-by-field -- DataStore.cpp never needs to know ComPrefs's field
  // layout at all, so CommonCLI.h stays the single source of truth for
  // that struct's shape and CommonCLI.cpp never needs touching. Returns
  // true if /beebo_repeater already existed (so the caller knows whether a
  // one-time seed from /com_prefs, via CommonCLI's own loadPrefs(), is
  // needed). Same "existence isn't enough" caveat as
  // loadBeeboCompanionPrefs() -- /beebo_repeater predates the ComPrefs
  // fold-in too.
  bool loadBeeboRepeaterPrefs(BeeboRepeaterPrefs& prefs, void* com_prefs, size_t com_prefs_len);
  void saveBeeboRepeaterPrefs(const BeeboRepeaterPrefs& prefs, const void* com_prefs, size_t com_prefs_len);
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
