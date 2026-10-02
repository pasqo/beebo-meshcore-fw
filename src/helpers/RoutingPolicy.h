#pragma once

#include <Packet.h>
#include <string.h>

namespace mesh {

/**
 * \brief  Test a flood packet against the configured hop limits.
 * \param  packet  inbound flood packet (caller has already checked isRouteFlood())
 * \param  flood_max            max hops for any flood packet
 * \param  flood_max_unscoped   max hops for ROUTE_TYPE_FLOOD (ie. un-scoped) packets
 * \param  flood_max_advert     max hops for ADVERT packets
 * \returns  true if the packet has exceeded a limit, and must not be forwarded
 */
inline bool isFloodHopLimitExceeded(const Packet* packet, uint8_t flood_max,
                                    uint8_t flood_max_unscoped, uint8_t flood_max_advert) {
  uint8_t hops = packet->getPathHashCount();
  if (hops >= flood_max) return true;
  if (packet->getRouteType() == ROUTE_TYPE_FLOOD && hops >= flood_max_unscoped) return true;
  if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT && hops >= flood_max_advert) return true;
  return false;
}

/**
 * \brief  Cap how far a flood packet this node originates can travel, by
 *         pre-filling its path. A forwarding repeater appends its hash only
 *         while (count + 1) * hash_size <= MAX_PATH_SIZE (Mesh::routeRecvPacket),
 *         so a path with MAX_PATH_SIZE/3 - max_hops entries of 3-byte hashes
 *         leaves room for exactly max_hops more. The last entry is the sender's
 *         own hash (self_hash, 3 bytes), as if it had forwarded the packet
 *         itself, so receivers see the real transmitter as the last hop (their
 *         neighbor); the entries before it are zeroes, which no node id starts
 *         with (0x00 is reserved), so no repeater mistakes one for itself.
 *         Call after the packet's route type is set; replaces the path.
 *         max_hops 0 leaves the packet untouched (no cap); above 21 is the
 *         same as 21 (the protocol maximum for 3-byte hashes), which has no
 *         path to pre-fill.
 */
inline void applyHopCap(Packet* packet, uint8_t max_hops, const uint8_t* self_hash) {
  if (max_hops == 0) return;
  const uint8_t size = 3;
  const uint8_t slots = MAX_PATH_SIZE / size;
  if (max_hops > slots) max_hops = slots;
  uint8_t fill = slots - max_hops;
  memset(packet->path, 0, fill * size);
  if (fill > 0) memcpy(&packet->path[(fill - 1) * size], self_hash, size);
  packet->setPathHashSizeAndCount(size, fill);
}

/**
 * \brief  How a server routes a reply back to the requesting client.
 */
enum ReplyRoute : uint8_t {
  REPLY_ROUTE_PATH_RETURN,       // request arrived by flood: reply with a PATH return, flooded back
  REPLY_ROUTE_DIRECT_SUPPLIED,   // reply DIRECT, along the return path supplied in the request
  REPLY_ROUTE_DIRECT_OUT_PATH,   // reply DIRECT, along the out_path already stored for this client
  REPLY_ROUTE_FLOOD,             // no return path known: flood the reply
};

/**
 * \param  inbound_is_flood    the request arrived as a flood packet
 * \param  have_supplied_path  the request payload carried an explicit reply path
 * \param  have_out_path       this server already has a stored out_path for the client
 */
inline ReplyRoute chooseReplyRoute(bool inbound_is_flood, bool have_supplied_path, bool have_out_path) {
  if (inbound_is_flood) return REPLY_ROUTE_PATH_RETURN;
  if (have_supplied_path) return REPLY_ROUTE_DIRECT_SUPPLIED;
  if (have_out_path) return REPLY_ROUTE_DIRECT_OUT_PATH;
  return REPLY_ROUTE_FLOOD;
}

/**
 * \brief  Which transport scope a flooded reply should be sent with.
 */
enum ReplyScope : uint8_t {
  REPLY_SCOPE_REQUEST,   // re-use the scope the request arrived on
  REPLY_SCOPE_DEFAULT,   // fall back to this node's default region scope
  REPLY_SCOPE_NONE,      // send un-scoped (ROUTE_TYPE_FLOOD)
};

/**
 * \param  request_scope_known         request arrived scoped, and we resolved its Region's key
 * \param  request_was_unscoped_flood  request arrived as an un-scoped flood
 * \param  default_scope_known         this node has a default Region with a usable transport key
 */
inline ReplyScope chooseReplyScope(bool request_scope_known, bool request_was_unscoped_flood,
                                   bool default_scope_known) {
  if (request_scope_known) return REPLY_SCOPE_REQUEST;
  if (request_was_unscoped_flood) return REPLY_SCOPE_NONE;  // requester chose un-scoped, so mirror it
  if (default_scope_known) return REPLY_SCOPE_DEFAULT;      // scope unknowable: DIRECT, or unresolved Region
  return REPLY_SCOPE_NONE;
}

}
