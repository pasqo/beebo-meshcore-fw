#pragma once

#include "ConfigSerializer.h"

// Read only the fields needed to distinguish the two v1.18.0 prefs schemas.
// Both are emitted even when their values are at their defaults.
class UpstreamRoleProbe : public ConfigSerializer {
  class CompanionFields : public ConfigSerializer {
  protected:
    void structure() override { def("auto_max", auto_max); }
  public:
    uint8_t auto_max = 0xff;
  } companion;

protected:
  void structure() override {
    def("name", name, sizeof(name));
    def("pass", password, sizeof(password));
    def("comp", companion);
  }

public:
  char password[2] = {'\x01', '\0'};
  char name[2] = {'\x01', '\0'};
  bool hasName() const { return name[0] != '\x01'; }

  enum Role : uint8_t { UNKNOWN, COMPANION, REPEATER };
  Role role() const {
    const bool has_companion = companion.auto_max != 0xff;
    const bool has_repeater = password[0] != '\x01';
    if (has_companion == has_repeater) return UNKNOWN;
    return has_companion ? COMPANION : REPEATER;
  }
};
