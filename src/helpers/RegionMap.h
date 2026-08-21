#pragma once

#include <Arduino.h>   // needed for PlatformIO
#include <Packet.h>
#include "TransportKeyStore.h"

#ifndef MAX_REGION_ENTRIES
  #define MAX_REGION_ENTRIES  32
#endif

#define REGION_DENY_FLOOD   0x01
#define REGION_DENY_DIRECT  0x02   // reserved for future

struct RegionEntry {
  uint16_t id;
  uint16_t parent;
  uint8_t flags;
  char name[31];

  bool isWildcard() const { return id == 0; }
};

// beebo: putRegion() can fail for two unrelated reasons -- an illegal
// name character, or a genuinely full table -- and callers need to tell
// them apart to report the right error code instead of a misleading
// ERR_CODE_TABLE_FULL for what's actually a bad name (see BUGS.md's
// region-table entry for the real-world cost of not distinguishing
// these: a client-side display-placeholder string fed back as a "region
// name" got rejected for illegal characters, but the resulting
// ERR_CODE_TABLE_FULL sent debugging down a completely wrong path
// chasing a table leak that never existed).
enum PutRegionError { PUT_REGION_OK = 0, PUT_REGION_ILLEGAL_NAME, PUT_REGION_TABLE_FULL };

class RegionMap {
  TransportKeyStore* _store;
  uint16_t next_id, home_id, default_id;
  uint16_t num_regions;
  RegionEntry regions[MAX_REGION_ENTRIES];
  RegionEntry wildcard;

  void printChildRegions(int indent, const RegionEntry* parent, Stream& out) const;

public:
  RegionMap(TransportKeyStore& store);

  static bool is_name_char(uint8_t c);

  bool load(FILESYSTEM* _fs, const char* path=NULL);
  bool save(FILESYSTEM* _fs, const char* path=NULL);

  // out_err, when given, is set to PUT_REGION_OK on success or the
  // specific failure reason on a NULL return -- existing call sites that
  // pass nullptr (the default) are unaffected.
  RegionEntry* putRegion(const char* name, uint16_t parent_id, uint16_t id = 0,
                        PutRegionError* out_err = nullptr);
  RegionEntry* findMatch(mesh::Packet* packet, uint8_t mask);
  RegionEntry& getWildcard() { return wildcard; }
  RegionEntry* findByName(const char* name);
  RegionEntry* findByNamePrefix(const char* prefix);
  RegionEntry* findById(uint16_t id);
  RegionEntry* getHomeRegion();   // NOTE: can be NULL
  void setHomeRegion(const RegionEntry* home);
  RegionEntry* getDefaultRegion();   // NOTE: can be NULL
  void setDefaultRegion(const RegionEntry* def);
  bool removeRegion(const RegionEntry& region);
  bool clear();
  void resetFrom(const RegionMap& src) { num_regions = 0; next_id = src.next_id; }
  int getCount() const { return num_regions; }
  const RegionEntry* getByIdx(int i) const { return &regions[i]; }
  const RegionEntry* getRoot() const { return &wildcard; }
  int exportNamesTo(char *dest, int max_len, uint8_t mask, bool invert = false);
  int getTransportKeysFor(const RegionEntry& src, TransportKey dest[], int max_num);

  void    exportTo(Stream& out) const;
  size_t  exportTo(char *dest, size_t max_len) const;
 
};
