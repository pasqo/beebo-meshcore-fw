#pragma once

// Arduino-free builders for the valid empty replies a node in repeater role gives the phone
// app's sync, unit-testable from fw/test/. Codes match RESP_CODE_* in Beebo.h.

#include <cstddef>
#include <cstdint>
#include <cstring>

#define APP_SYNC_RESP_CONTACTS_START   2   // RESP_CODE_CONTACTS_START
#define APP_SYNC_RESP_END_OF_CONTACTS  4   // RESP_CODE_END_OF_CONTACTS
#define APP_SYNC_RESP_NO_MORE_MESSAGES 10  // RESP_CODE_NO_MORE_MESSAGES

// Contacts list with zero entries: start (count 0) ...
inline size_t appSyncContactsStart(uint8_t* out) {
  out[0] = APP_SYNC_RESP_CONTACTS_START;
  memset(&out[1], 0, 4);
  return 5;
}

// ... then end (most recent lastmod 0).
inline size_t appSyncContactsEnd(uint8_t* out) {
  out[0] = APP_SYNC_RESP_END_OF_CONTACTS;
  memset(&out[1], 0, 4);
  return 5;
}

inline size_t appSyncNoMoreMessages(uint8_t* out) {
  out[0] = APP_SYNC_RESP_NO_MORE_MESSAGES;
  return 1;
}
