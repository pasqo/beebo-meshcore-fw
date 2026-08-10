#pragma once

#include <Identity.h>
#include <helpers/ClientACL.h>
#include <helpers/RegionMap.h>
#include <helpers/TransportKeyStore.h>
#include "BeeboPrefs.h"

// beebo: SETTINGS_REFACTOR.md Part 3 -- everything one compiled-in role
// needs resident, self-contained, in one object -- so a function that
// operates on "a role" (a future loadRoleState(), a cross-role opcode,
// config pull/push --role) takes one pointer/reference and has everything,
// instead of juggling several same-length arrays kept in sync by
// convention. Board identity (role/board_password/board_name) is
// deliberately NOT a member here -- see BeeboPrefs.h's own comment and
// Beebo.h's `_board`.
//
// Stage 2 (current, structural-only slice -- see plan doc's Part 3
// "Consequences" section for the full design, not all landed yet): `acl`/
// `key_store`/`region_map` moved in here from being singleton Beebo
// members, `identity` added as the resident per-role keypair. This is
// purely a structural relocation -- Beebo.h keeps `acl`/`key_store`/
// `region_map` reference-aliased to
// `role_state_store[NODE_ROLE_REPEATER]`'s
// members (see that header), and `reloadIdentityForRole()`/self_id
// assignment sites are unchanged in *when* they run -- so
// ensureRepeaterStateLoaded()'s lazy load-on-first-repeater-entry and
// today's role-switch reload-from-disk semantics are untouched. The
// boot-time loadRoleState()/eager-load-every-role/repoint-on-switch
// behavior change described in the plan doc is deliberately NOT part of
// this slice.
struct BeeboRoleState {
  BeeboPrefs prefs;
  mesh::LocalIdentity identity;
#if BEEBO_ENABLE_REPEATER_ROLE
  ClientACL acl;
  TransportKeyStore key_store;
  RegionMap region_map{key_store};
#endif
};
