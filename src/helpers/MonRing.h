#pragma once

#include <stdint.h>
#include <string.h>

// beebo: continuous monitoring ring in PSRAM.
//
// A single fixed-stride ring of 16-byte records that interleaves several record
// kinds in one temporal timeline: high-frequency RX/TX captures plus
// low-frequency reference records (SYNC time base, RADIO config, ENV
// environment). Allocated once at begin(), it overwrites the oldest record when
// full and is read on demand, OLDEST first, paginated by a monotonic
// positional `seq`.
//
// Time model (no accumulation error, wrap-proof):
//   * A running absolute base lives in bookkeeping (`_base`, epoch seconds) and
//     never evicts. Every non-SYNC record stores a 16-bit `offset` = capture
//     time - base (0..~18h), so each record's time is reconstructed by a single
//     `base + offset` with no summing.
//   * A SyncRecord carries a full absolute timestamp and is committed BEFORE
//     anything that depends on it: appendRx/appendTx/noteRadio/sampleEnv all
//     call _ensureSync() first, which latches a new base and stores a SYNC
//     record carrying it immediately, forward semantics ("from here on this
//     is the base"). Reading forward, the reader always hits the Sync that
//     governs a record BEFORE the record itself.
//
// Reference records generalise the same rule: SYNC/RADIO/ENV each carry the
// NEW state, stored immediately when the running state actually changes (one
// row per real change, not one per read) — see _ensureSync()'s relatch and
// noteRadio()/sampleEnv(). Records are therefore always written in causal
// order (sync, radio, env, then the rx/tx that depend on them), so a plain
// forward walk from tail to head reconstructs the timeline directly with no
// reversal.
//
// Concurrent read safety: rather than reasoning about a live ring during a
// multi-page read, the caller brackets a read with pauseForRead()/
// resumeAfterRead() (append/note/sample are no-ops while capture is paused),
// so _head/_count are frozen for the whole read and a plain bounded forward
// loop is trivially safe.
//
// Ring-wrap wrinkle: when the ring wraps and evicts the oldest SYNC/RADIO/ENV,
// later still-resident RX/TX records would lose their governing reference.
// `start_sync`/`start_radio`/`start_env` each hold a reference that is always
// valid, from two sources: init()/clear() seed them directly (no ring slot
// consumed), before any real record exists; once the ring wraps, _store()
// overwrites the relevant one in O(1) at eviction time with the last real
// record of that kind that just fell off. Either way, peek()/emitStartRef()
// let a reader check, slot by slot (sync, radio, env, in that fixed order),
// whether the real forward walk already covers that slot or whether the
// stashed start-ref must be spliced in instead, so the reader always has a
// valid reference for the first RX/TX record it will see, from the moment of
// seeding through any number of wraps.

// ---- record kinds (byte 0 of every record) --------------------------------
enum : uint8_t {
  MON_SYNC = 0, MON_RX = 1, MON_TX = 2, MON_RADIO = 3, MON_ENV = 4, MON_BATT = 5,
  MON_TUNE = 6, MON_EVENT = 7,
  // beebo: split out of MON_EVENT -- EVENT_SETTING_CHANGED/EVENT_COMMAND_RUN
  // had their own fixed column shapes (setting/old_value/new_value/source;
  // command/source) that no other event type shares, unlike the genuinely
  // generic fault/overflow/queue-drop/wrap events MON_EVENT still carries.
  // Gated by the same MON_CAP_EVENT capture bit as MON_EVENT (no free
  // MON_CAP_* bits remain -- see that enum below -- and these are still
  // conceptually part of "admin/event activity capture" for that purpose).
  MON_COMMAND = 8, MON_SETTING = 9,
};

// ---- TUNE param IDs: which NodePrefs knob a TuneRecord proposal concerns ---
enum {
  TUNE_RX_DELAY_BASE = 0, TUNE_TX_DELAY_FACTOR, TUNE_DIRECT_TX_DELAY_FACTOR,
  TUNE_AGC_RESET_INTERVAL, TUNE_INTERFERENCE_THRESHOLD, TUNE_AIRTIME_FACTOR,
};

// ---- EVENT types: what kind of thing an EventRecord reports, plus its own
// event_type-specific payload layout within the record's 12-byte `data`
// field. Deliberately generic (not fault-specific) so future event kinds --
// reboot, role switch, transport change, ... -- reuse this one MonRing kind
// instead of consuming another of the few remaining MON_CAP_* bits (MON_EVENT
// is the last one before MON_CAP_ENABLED's bit7) -- but ONLY when the event
// genuinely has no fixed shape of its own; see MON_COMMAND/MON_SETTING above
// (SettingRecord/CommandRecord) for the two that outgrew this and got their
// own dedicated kind/struct instead of a generic data[12] blob. 0, 1 and 2 are
// retired (were EVENT_FAULT, EVENT_SETTING_CHANGED, EVENT_COMMAND_RUN) -- not
// reused, so a downloaded trace spanning the rename never misinterprets an
// old record.
enum : uint8_t {
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 -- "TX reception confirmation".
  // 3, 4 retired (were EVENT_ACK_RESULT/EVENT_ECHO_RESULT, a shared type
  // per side with a TXCONFIRM_* verdict byte in data[0] for SUCCESS vs.
  // TIMEOUT) -- not reused. Split into their own dedicated success/timeout
  // event types below (EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT/
  // EVENT_ECHO_SUCCESS/EVENT_ECHO_TIMEOUT, ids 20-23), same "a reader
  // shouldn't have to inspect the payload to tell outcomes apart" reasoning
  // that already split resource-exhaustion OVERFLOW out to its own type
  // (EVENT_ACK_OVERFLOW/EVENT_ECHO_OVERFLOW below) instead of a third
  // verdict value in this shared payload.
  // One record per RX drop that never reaches the ring as its own MON_RX
  // record -- the packet pool had no free buffer (Dispatcher::checkRecv(),
  // no Packet was ever allocated). Distinct cause from EVENT_RX_PARSE_ERROR
  // below (radio/allocation capacity vs. a malformed frame) -- separate
  // event types so a reader doesn't need to inspect a reason byte to tell
  // them apart. See MonRing::bumpRxDropCount().
  //   data[0:4]  = cumulative lifetime count for this reason, AFTER this
  //                occurrence (u32 LE) -- so a downloaded trace can plot the
  //                running total over time without needing separate state.
  //   data[4:12] = reserved
  EVENT_RX_POOL_FULL = 5,
  // Same shape as EVENT_RX_POOL_FULL, different cause: a frame WAS captured
  // (a Packet briefly existed) but failed to parse (bad version/path-mode/
  // corrupt/oversize) and was freed again before a MON_RX record could be
  // built. See Dispatcher::checkRecv()'s tryParsePacket() failure branch.
  EVENT_RX_PARSE_ERROR = 6,
  // A packet this node decided to relay (already logged RX_DISP_FORWARDED,
  // see Mesh.cpp) failed to actually enqueue because send_queue was full
  // (StaticPoolPacketManager::queueOutbound(), called from
  // Dispatcher::processRecvPacket()) -- deliberately NOT logged as its own
  // event: the richer, already-correlated trace for this exact case is the
  // RXREC_QUEUE_FULL bit on that same packet's own MON_RX record (SNR,
  // RSSI, neighbor, packet type, all still attached) -- a separate
  // standalone event here would just be a strictly poorer duplicate of
  // information the record already carries. See RX_DISP_QUEUE_FULL
  // (Dispatcher.h) and RXREC_QUEUE_FULL below instead.
  //
  // The two queue-full events that DO exist below are for the cases with NO
  // per-record alternative -- the dropped packet never had (or, for the
  // self-tx case, was never itself the subject of) a MON_RX record to carry
  // this information:
  //   data[0:4]  = cumulative lifetime count for this specific cause, AFTER
  //                this occurrence (u32 LE)
  //   data[4:12] = reserved
  //
  // Dispatcher::sendPacket()'s own queueOutbound() call failed -- a
  // self-originated transmission or ACK reply (NOT a relay of a received
  // packet -- routeDirectRecvAcks() builds a fresh reply packet with no
  // MON_RX record of its own) couldn't be queued for send. Emitted
  // immediately from Beebo's Dispatcher::logTxQueueFull(is_relay=false)
  // override -- the is_relay=true (relay) case is the RXREC_QUEUE_FULL case
  // above and intentionally does not also fire this event.
  EVENT_TX_QUEUE_FULL = 7,
  // Dispatcher::checkRecv()'s queueInbound() call failed -- an incoming
  // flood packet couldn't even be queued for its scored/delayed processing
  // window, dropped before onRecvPacket()/any disposition was ever reached.
  // Emitted immediately from Beebo's Dispatcher::logRxQueueFull() override.
  EVENT_RX_QUEUE_FULL = 8,
  // Node link (BLE/WiFi) transport queue-full drops -- app connectivity/UX,
  // NOT mesh routing capacity, a fundamentally different cause from the
  // mesh-side events above, hence their own types rather than a shared
  // "queue drop" bucket with a kind byte. Applies to every role (companion,
  // repeater) alike -- the transport link is role-independent, not
  // a companion-only concept, despite "companion protocol" being the wire
  // format's name. LINK_TX/LINK_RX combine BLE + WiFi (MultiSerialInterface
  // only ever has one transport session locked at a time, so a single
  // per-direction bucket is simpler than per-transport ones and no less
  // meaningful). Unlike the mesh-side events, these counters live in
  // SerialBLEInterface/SerialWifiInterface with no Dispatcher-level hook
  // available -- Beebo polls them once per tick (see
  // appendLinkQueueDropEvents()) and emits one record per detected
  // increase, so a downloaded trace still gets a timestamp even though the
  // raw counter itself isn't resident here.
  EVENT_LINK_TX_QUEUE_FULL = 9,   // BLE/WiFi writeFrame() -- node's send_queue full
  EVENT_LINK_RX_QUEUE_FULL = 10,  // BLE onWrite() -- node's recv_queue full (WiFi has no equivalent)
  // Resource-exhaustion faults split out of EVENT_ACK_RESULT/EVENT_ECHO_RESULT
  // (see that comment above for why) -- the ack table / echo ring wrapped
  // onto a slot that was still unresolved (neither confirmed nor timed out)
  // when a new generation needed that slot. Same data[12] layout as
  // EVENT_ACK_RESULT/EVENT_ECHO_RESULT but with no verdict byte needed (the
  // event type itself says what happened):
  //   data[0]    = reserved
  //   data[1:5]  = pkt_hash (u32 LE), same convention as above
  //   data[5:9]  = age_ms (u32 LE) -- how long the evicted slot had been
  //                pending, unresolved, before it was overwritten
  //   data[9:12] = reserved
  EVENT_ACK_OVERFLOW = 11,   // BaseChatMesh's expected_ack_table wrapped (ack_overflow_count)
  EVENT_ECHO_OVERFLOW = 12,   // SimpleMeshTables's echo ring wrapped (echo_overflow_count)
  // SimpleMeshTables::_hashes (the main dedup table, MAX_PACKET_HASHES=160)
  // had no free slot for a new hash and overwrote one that still held a real,
  // previously-inserted hash (not an empty/never-used slot) -- see
  // SimpleMeshTables::hasSeen(). Unlike the ring/table overflow events above,
  // this table has no notion of a slot being "resolved" first -- it's a
  // cyclic table by design, so there is no way to insert without evicting
  // whatever the next slot currently holds once the table has filled once.
  // What this event flags: dedup detection for a specific packet only works
  // if its hash is still resident when the duplicate arrives, so every
  // eviction of a live hash is a chance a genuine duplicate goes undetected
  // (wrongly treated as new -- re-forwarded, double-counted). Expect this to
  // fire on most inserts once the table has filled once after boot/clear --
  // that volume is itself the signal (a downloaded trace's occurrence rate
  // over time is the churn-rate indicator), not a sign something is broken.
  //   data[0:4]  = cumulative lifetime count, AFTER this occurrence (u32 LE)
  //   data[4:12] = reserved
  EVENT_RX_DEDUP_TABLE_FULL = 13,
  // beebo: split out of the old single EVENT_FAULT (retired id 0, see above)
  // -- that design bundled three genuinely different causes (packet-pool
  // exhaustion, CAD stuck busy, radio stuck out of RX mode) behind one event
  // type with a bitmask payload, which is exactly the "reason code buried in
  // a shared payload instead of the event's own identity" pattern this whole
  // catalog is meant to avoid. Each now has its own type, fires immediately
  // on every real occurrence (not just the first -- see
  // Dispatcher::logFaultEvent()), and carries a lifetime cumulative count as
  // its only payload, same shape as the other single-cause events above:
  //   data[0:4]  = cumulative lifetime count for this specific fault, AFTER
  //                this occurrence (u32 LE)
  //   data[4:12] = reserved
  // Dispatcher::_err_flags (ERR_EVENT_*, Dispatcher.h) is still the sticky,
  // never-clears-until-reboot summary these three bits feed -- unchanged,
  // still what `beebo report`'s "Active Faults" check and the ENV record's
  // err_flags snapshot read; these events are the discrete, timestamped
  // trail alongside it.
  EVENT_TX_POOL_FULL = 14,     // Dispatcher::obtainNewPacket() -- no free packet buffer to build an outgoing/forwarded packet into (ERR_EVENT_FULL). Same underlying pool as EVENT_RX_POOL_FULL's Dispatcher::checkRecv() case, different call site/cause: this fires when something this node is SENDING needed a fresh Packet, not an incoming reception.
  EVENT_TX_CAD_TIMEOUT = 15,      // Dispatcher::checkSend() -- channel-activity detection stuck busy past its max duration (ERR_EVENT_CAD_TIMEOUT)
  EVENT_RX_START_TIMEOUT = 16,  // Dispatcher::checkRecv() -- radio stuck out of RX mode for >8s (ERR_EVENT_STARTRX_TIMEOUT)
  // beebo: repeater-role Beebo::allowPacketForward()'s three flood-decline
  // causes (Beebo.cpp), split out of the generic RX_DISP_NO_FORWARD routing
  // bit for the same "no free axis value left to split it further" reason
  // EVENT_RX_DEDUP_TABLE_FULL was split out above -- the RxRecord.disp routing
  // axis is only 2 bits and already fully used (NONE/FORWARDED/NO_FORWARD/
  // PATH_FULL), so a discrete event per cause is the only way to tell these
  // three causes apart in a downloaded trace. All three are deliberately
  // excluded from MonRing::SohStats/computeSoh() -- each is a configured
  // policy drop (hop/loop limits, region scoping), not this node's own
  // infrastructure struggling. The lifetime cumulative count for each cause
  // lives in the GET_MONRING header instead (Beebo::_max_hop_no_fwd_count/
  // _region_no_fwd_count/_loop_no_fwd_count, surfaced in `beebo monitor
  // status`'s Service Status table) -- NOT duplicated into this payload, so
  // the declining packet's own hash is all data[] carries: the correlation
  // key is the useful part of a per-occurrence record (which packet), a
  // second copy of a total already available from the header is not:
  //   data[0]    = reserved
  //   data[1:5]  = pkt_hash (u32 LE) -- Packet::calculateMonRingHash()
  //   data[5:12] = reserved
  EVENT_MAX_HOP_NO_FWD = 17,  // getPathHashCount() past _fwd_flood_max/_fwd_flood_max_unscoped/_fwd_flood_max_advert -- too many hops already
  EVENT_REGION_NO_FWD = 18,     // recv_pkt_region == NULL -- transport code matched no flood-allowed region this repeater has defined, or an unscoped packet hit a flood-denied wildcard
  EVENT_LOOP_NO_FWD = 19,       // isLooped() -- this node already appears in the packet's path too many times (repeater.forward.loop_detect)
  // beebo: DYNAMIC_OPTIMIZER_PLAN.md item 9 follow-up -- the retired
  // EVENT_ACK_RESULT/EVENT_ECHO_RESULT (ids 3/4 above) split into their own
  // success/timeout event types per side. One record per verdict reached on
  // a specific outgoing transmission this node made -- ring/table OVERFLOW
  // (starvation: the slot wrapped before reaching either verdict) stays its
  // own separate event type (EVENT_ACK_OVERFLOW/EVENT_ECHO_OVERFLOW above),
  // not a third value here. Same data[12] layout as EVENT_ACK_OVERFLOW/
  // EVENT_ECHO_OVERFLOW (no verdict byte needed -- the event type itself
  // says what happened):
  //   data[0]    = reserved
  //   data[1:5]  = pkt_hash (u32 LE) -- SHA256(payload)[0:4], Packet::
  //                calculateMonRingHash()'s convention, the SAME one
  //                TxRecord/RxRecord.pkt_hash already use. This is the
  //                correlation key: it matches the MON_TX record for the
  //                original transmission this event reports on, and for
  //                EVENT_ECHO_SUCCESS, ALSO the MON_RX record of the
  //                specific echo/dup packet that confirmed it (same packet,
  //                same hash, by construction -- that's literally what
  //                "echo" means here). The ACK side has no equivalent
  //                RX-side match (an ACK reply is a different packet from
  //                the original message, with its own different hash) --
  //                EVENT_ACK_SUCCESS only correlates back to the origin
  //                MON_TX record, not to the ACK packet's own MON_RX record.
  //   data[5:9]  = age_ms (u32 LE) -- elapsed time since the original
  //                transmission until this verdict (trip time).
  //   data[9:12] = reserved
  EVENT_ACK_SUCCESS = 20,   // DM/ACK side, ACK received (Beebo::processAck())
  EVENT_ACK_TIMEOUT = 21,   // DM/ACK side, no ACK within deadline (checkAckTableTimeouts())
  EVENT_ECHO_SUCCESS = 22,  // flood-echo side, rebroadcast heard (SimpleMeshTables::hasSeen())
  EVENT_ECHO_TIMEOUT = 23,  // flood-echo side, echo window elapsed (checkEchoTimeouts())
  // beebo: temporary diagnostic for the transport-independent GET_PREFS_TLV
  // stall investigation (BUGS.md 2026-08-16) -- Beebo::loop() times each of
  // its top-level segments and logs one of these whenever a single loop()
  // iteration takes longer than LOOP_STALL_THRESHOLD_MS (Beebo.cpp), so a
  // captured MonRing trace shows exactly which segment was running during a
  // real-world multi-second stall instead of guessing from source alone.
  //   data[0]    = segment_id -- which segment (see Beebo.cpp's
  //                LOOP_SEG_* enum) took the longest within this iteration
  //   data[1:3]  = that segment's own duration_ms (u16 LE)
  //   data[3:5]  = total loop() iteration duration_ms (u16 LE)
  //   data[5:12] = reserved
  EVENT_LOOP_STALL = 24,
};

// ---- TXCONFIRM_*: verdict enum used ONLY for internal bookkeeping now
// (SimpleMeshTables/AckTableEntry track which verdict, if any, a slot has
// reached) -- no longer an EventRecord payload byte. It used to double as
// EVENT_ACK_RESULT/EVENT_ECHO_RESULT's data[0] verdict, but those retired in
// favor of EVENT_ACK_SUCCESS/EVENT_ACK_TIMEOUT/EVENT_ECHO_SUCCESS/
// EVENT_ECHO_TIMEOUT (own event type per verdict, see above) -- kept here
// only because source code still benefits from naming all three
// possibilities (including OVERFLOW, EVENT_ACK_OVERFLOW/EVENT_ECHO_OVERFLOW's
// cause) on one axis internally.
enum : uint8_t {
  TXCONFIRM_SUCCESS = 0,   // ACK received / echo heard
  TXCONFIRM_TIMEOUT = 1,   // no ACK / no echo within the deadline or echo window
  TXCONFIRM_OVERFLOW = 2,  // the ring/table wrapped onto this still-unresolved slot before it reached either verdict -- never emitted via EVENT_ACK_RESULT/EVENT_ECHO_RESULT itself, see EVENT_ACK_OVERFLOW/EVENT_ECHO_OVERFLOW
};

// ---- EVENT_SOURCE_*: which dispatch path produced a SettingRecord or
// CommandRecord -- the binary CMD_BEEBO/companion protocol (reachable over
// BLE/TCP/USB), or the USB/mesh-admin text CLI (DualModeSerialInterface's
// local text path, or an authenticated remote mesh admin session --
// handleCommand()'s two call sites in Beebo.cpp don't distinguish between
// them, so neither does this).
enum : uint8_t {
  EVENT_SOURCE_BINARY = 0, EVENT_SOURCE_TEXT_CLI = 1,
};

// ---- SETTING_* keys (SettingRecord.setting): reuses Beebo.h's PrefsTlvKey
// numbering (1-17) for
// any field already in that table, so a downloaded event can be
// cross-referenced against that same key space if a future field there ever
// gets wired up to emit this event too. Settings with no ComPrefs/TLV
// equivalent (RAM-only, or a companion NodePrefs field outside that table)
// get their own IDs starting at 100, well clear of PrefsTlvKey's current or
// likely future range, so the two numberings can never collide.
enum : uint8_t {
  SETTING_NODE_ROLE = 100, SETTING_TUNE_ENABLED = 101, SETTING_TUNE_APPLIED_MASK = 102,
  // beebo: companion.routing.dedup_window (NodePrefs.dedup_window_ms) -- no
  // PrefsTlvKey/ComPrefs equivalent (same shape as idle_margin_ms/
  // batt_sample_*, none of which are TLV-registered either), so its own ID
  // here, same as the three above. The repeater copy reuses
  // PREFS_TLV_REPEATER_DEDUP_WINDOW (20) instead, per this enum's own
  // comment -- it IS TLV-registered.
  SETTING_DEDUP_WINDOW = 103,
};

// ---- persisted capture config: per-kind mask + global enable (bit7) -------
// SYNC is always-on (not maskable) — it's driven by time-base relatch, not an
// independent capture point.
#define MON_CAP_RX      0x01
#define MON_CAP_TX      0x02
#define MON_CAP_RADIO   0x04
#define MON_CAP_ENV     0x08
#define MON_CAP_BATT    0x10
#define MON_CAP_TUNE    0x20
#define MON_CAP_EVENT   0x40
#define MON_CAP_ALL     (MON_CAP_RX | MON_CAP_TX | MON_CAP_RADIO | MON_CAP_ENV | MON_CAP_BATT | MON_CAP_TUNE | MON_CAP_EVENT)
#define MON_CAP_ENABLED 0x80

// ---- RX flags byte: nbr_len + hop count (rx-gain moved to the RADIO record)
// hops mirrors Packet::getPathHashCount() exactly (0-63, same 6-bit width),
// captured in logRxRaw() before a Packet object even exists -- lets
// zero-hop receptions (e.g. an ADVERT heard directly, same check
// onAdvertRecv() uses to populate the direct-neighbour table) be
// distinguished after the fact via `analyze packets --hops 0`, without
// needing a live device's neighbour table. b2-7 were reserved-and-always-
// zeroed before MONRING_ABI_VERSION 2 -- see that define below for why a
// hops of 0 from an older capture must NOT be trusted as "zero hops".
#define RXREC_FLAG_NBRLEN_MASK  0x03
#define RXREC_FLAG_HOPS_MASK    0xFC
#define RXREC_FLAG_HOPS_SHIFT   2

// ---- RX disposition byte: three INDEPENDENT axes so a record can carry
// several outcomes at once (e.g. delivered to us AND relayed) instead of one
// lossy "best" value. Called "endpoint" here and in the *_EP_* names below
// (this node acting as the packet's destination, vs. "routing" = this node
// acting as a relay) -- surfaced to the CLI/analyze side as "verdict"
// instead (monitor.py/analysis.py), since "endpoint" reads as a network
// address there, not an outcome; RXEP_NONE (below) displays as "captured"
// there too (0 means "captured only, not yet resolved" -- see RxDisposition
// enum, Dispatcher.h), not "no verdict at all":
//   [0..3] endpoint : the local/terminal verdict (mutually exclusive)
//   [4..5] routing  : what we did as a relay (mutually exclusive)
//   [6]    distill  : rider set in logRxRaw when the frame couldn't be distilled
//   [7]    queue_full : RX_DISP_QUEUE_FULL correction (see below)
// applyDisposition() maps each RxDisposition (Dispatcher.h) reason onto its axis;
// endpoint is write-once (ACCEPTED always wins), routing is write-once, distill
// and queue_full are both a sticky OR.
#define RXREC_EP_MASK    0x0F
#define RXREC_RT_MASK    0x30
#define RXREC_RT_SHIFT   4
#define RXREC_DISTILL    0x40
// beebo: routing already read RXRT_FORWARDED by the time a real
// queueOutbound() failure is known (see Dispatcher::processRecvPacket()) --
// the routing axis is write-once and only 2 bits (already fully used by
// NONE/FORWARDED/NO_FORWARD/PATH_FULL), so rather than lie by overwriting a
// decision that genuinely was made, this bit records the outcome
// separately: routing==FORWARDED + this bit set means "decided to forward,
// but it never actually left the radio (send_queue was full)". Offline
// analysis (delivery_proxy etc.) should treat that combination as NOT
// delivered.
#define RXREC_QUEUE_FULL 0x80
enum {
  RXEP_NONE = 0, RXEP_ACCEPTED, RXEP_NOT_FOR_US, RXEP_DUP, RXEP_WRONG_HOP,
  RXEP_FILTERED, RXEP_INCOMPLETE, RXEP_FORGED_SIG, RXEP_SELF, RXEP_UNKNOWN_TYPE,
  RXEP_PARSE_ERR, RXEP_POOL_FULL, RXEP_MISC_DROP,
};
enum { RXRT_NONE = 0, RXRT_FORWARDED, RXRT_NO_FORWARD, RXRT_PATH_FULL };

// ---- TX result byte: outcome of our own transmission ------------------------
enum { TXR_OK = 0, TXR_TIMEOUT = 1 };

// ---- RADIO flags byte ------------------------------------------------------
#define RADIO_FLAG_RXBOOST    0x01
#define RADIO_FLAG_FEMRXGAIN  0x02

// ---- BATT flags byte: idle bit + BattTrend.h's BATT_STATE_* -----------------
#define BATTREC_FLAG_IDLE   0x01  // 1 = idle-gated (soft window), 0 = forced/deadline
#define BATTREC_STATE_MASK  0x0E
#define BATTREC_STATE_SHIFT 1
// beebo: independent transport power-draw correlates at sample time, from
// BaseSerialInterface::is24GUp()/isUsbUp() -- two separate bits, not an
// enum, since USB can be up at the same time as BLE/TCP (BLE and TCP are
// mutually exclusive with each other, but not with USB). BLE/TCP idle at
// ~75mA vs USB's much lower draw makes 24G the useful correlate; USB is
// tracked too since it's a distinct power rail.
#define BATTREC_FLAG_XPORT_24G  0x10  // BLE and/or TCP currently powered/enabled
#define BATTREC_FLAG_XPORT_USB  0x20  // USB currently powered/enabled

// ---- forward-compat: version of the MonRecord layout this build writes.
// Stamped into every SyncRecord (real or start-ref) so an offline dataset
// spanning a future layout change still carries, on its very first record,
// which version governs everything that follows it -- bump this whenever a
// record struct's field layout changes in a way that would misparse under
// the old FMT_* struct formats (beebo/src/beebo/monitor.py). Adding a new
// record KIND, or a new EVENT_*/SETTING_*/TUNE_* enum value, does not need a
// bump (old decoders already handle "unknown enum value" gracefully); only
// changing the byte layout of an existing struct does.
//
// 2: exception to the above -- RxRecord.flags's byte layout is unchanged
// (still 1 byte, still packed/parsed the same way), but b2-7 flipped from
// "reserved, always zero" to RXREC_FLAG_HOPS_MASK. That makes 0 genuinely
// ambiguous on an old capture: a real zero-hop reception vs. firmware that
// never wrote the field. Bumped anyway so the host side
// (monitor.py's _build_sequence) can gate on abi_version and null out
// `hops` for any record governed by a SYNC stamped < 2, rather than
// silently misreporting every pre-bump rx record as zero-hop.
#define MONRING_ABI_VERSION 2

// ---- the union members (each exactly 16 bytes, packed, kind at byte 0) -----
struct __attribute__((packed)) SyncRecord {
  uint8_t  kind;        // MON_SYNC
  uint32_t timestamp;   // absolute epoch seconds — the base for NEWER records
  uint8_t  abi_version; // MONRING_ABI_VERSION at capture time -- see above
  uint8_t  _rsvd[10];
};
struct __attribute__((packed)) RxRecord {
  uint8_t  kind;        // MON_RX
  uint16_t offset;      // seconds since current Sync base
  uint32_t pkt_hash;    // SHA256(payload)[0:4] LE — rxlog-compatible
  int8_t   snr;         // snr * 4
  int8_t   rssi;        // dBm
  uint8_t  header;      // route(2b) | payload-type(4b) | ver(2b)
  uint8_t  flags;       // nbr_len(b0-1) | hops(b2-7)
  uint8_t  disp;        // endpoint(b0-3) | routing(b4-5) | distill(b6) | rsvd(b7)
  uint8_t  nbr[3];      // last-hop hash = immediate neighbour (nbr_len valid)
  uint8_t  _rsvd;
};
struct __attribute__((packed)) TxRecord {
  uint8_t  kind;        // MON_TX
  uint16_t offset;
  uint32_t pkt_hash;    // our transmitted packet's hash (cross-node correlation)
  uint8_t  header;
  uint8_t  result;      // TXR_* send outcome
  uint16_t airtime_ms;  // estimated on-air time (deterministic from sf/bw/cr/len)
  uint8_t  dst[3];      // next-hop neighbour hash (DIRECT only; zero for FLOOD)
  uint8_t  _rsvd[2];
};
struct __attribute__((packed)) RadioRecord {
  uint8_t  kind;        // MON_RADIO
  uint16_t offset;
  uint32_t freq;        // Hz
  uint8_t  sf;
  uint16_t bw;          // bandwidth in kHz * 10 (e.g. 250.0 kHz -> 2500)
  uint8_t  cr;
  int8_t   tx_power;    // dBm
  uint8_t  flags;       // rx_boosted_gain(b0) | fem_rxgain(b1)
  uint8_t  _rsvd[3];
};
struct __attribute__((packed)) EnvRecord {
  uint8_t  kind;        // MON_ENV
  uint16_t offset;
  int8_t   noise_floor; // dBm (was the per-RX 'noise' field)
  uint16_t batt_mv;
  int8_t   temp_c;
  uint16_t free_heap;   // KB
  uint8_t  pool_free;   // free packet buffers
  uint8_t  tx_queue;
  uint8_t  err_flags;
  uint8_t  cad_busy_events; // beebo: low byte of Dispatcher::getCADBusyEventCount(), sampled alongside the rest of env
  uint8_t  _rsvd[3];
};
struct __attribute__((packed)) BattRecord {
  uint8_t  kind;        // MON_BATT
  uint16_t offset;
  uint16_t batt_mv;
  uint8_t  flags;        // idle(b0) | BattTrend state(b1-3) | xport_24g(b4) | xport_usb(b5) | rsvd(b6-7) -- see BATTREC_*
  uint8_t  _rsvd[10];
};
// beebo: one proposal/decision emitted by the dynamic tuning optimizer per
// re-tune tick. `applied` distinguishes observe-only logging (the controller
// computes what it would set but never calls the NodePrefs setter) from live
// actuation. `reward_after` is deliberately not a field: the NEXT TuneRecord
// for the same param_id carries the post-change indicator as its own
// reward_before, so a before/after pair is reconstructed from two consecutive
// records instead of a duplicated field.
struct __attribute__((packed)) TuneRecord {
  uint8_t  kind;            // MON_TUNE
  uint16_t offset;
  uint8_t  param_id;        // TUNE_* -- which knob this proposal concerns
  uint8_t  applied;         // 0 = observe-only, 1 = applied
  int16_t  old_value;       // current param value, native fixed-point scale
  int16_t  proposed_value;  // what the controller would set / did set
  uint16_t reward_before;   // delivery-proxy indicator over the preceding window (0-10000 scaled)
  uint8_t  _rsvd[5];
};
// beebo: general-purpose event log -- one record per notable state
// transition, so events get real timestamps and context instead of a bare
// sticky flag with no history (see Dispatcher::_err_flags, which never
// clears until reboot and carries no timing information at all). event_type
// (EVENT_*, see the enum above) selects how `data` is interpreted --
// deliberately generic so future event kinds with no fixed shape of their
// own share this same MonRing kind/capture bit rather than each claiming a
// new one.
struct __attribute__((packed)) EventRecord {
  uint8_t  kind;         // MON_EVENT
  uint16_t offset;
  uint8_t  event_type;   // EVENT_*
  uint8_t  data[12];     // event_type-specific payload
};
// beebo: one admin-visible setting change -- split out of MON_EVENT into its
// own kind/struct (see MON_SETTING's comment above) since every instance
// shares this exact shape, unlike the genuinely one-off fault/overflow/wrap
// events MON_EVENT still carries as a generic blob. old_value/new_value are
// the raw u32 LE bit pattern (float settings use the same float-bits
// convention Beebo.h's PrefsTlvField.get_raw/set_raw already use). One
// record per change -- infrequent, and valuable for spotting user/admin
// interaction in a downloaded trace (e.g. distinguishing "the bandit changed
// this" from "someone flipped this switch over the text CLI").
struct __attribute__((packed)) SettingRecord {
  uint8_t  kind;         // MON_SETTING
  uint16_t offset;
  uint8_t  setting;      // SETTING_* key
  uint8_t  source;       // EVENT_SOURCE_* -- which dispatch path made the change
  uint32_t old_value;
  uint32_t new_value;
  uint8_t  _rsvd[3];
};
// beebo: one command run -- split out of MON_EVENT the same way (see
// MON_COMMAND's comment above). Logged for a command that mutates state or
// is otherwise a deliberate one-shot action (SET_*/"set ...", OTA begin/end,
// reboot, ...) -- NOT for routine reads/polling (GET_*/"get ...", paginated
// fetches), which would otherwise flood this infrequent-by-design log. See
// Beebo.cpp's checkRecvFrame() (binary) and handleCommand() (text) for the
// exact inclusion filters.
struct __attribute__((packed)) CommandRecord {
  uint8_t  kind;          // MON_COMMAND
  uint16_t offset;
  uint8_t  source;        // EVENT_SOURCE_* -- which dispatch path
  // EVENT_SOURCE_BINARY: command[0:2] = command_id (uint16 LE,
  //   (outer_cmd<<8)|sub_id -- the exact same encoding the profiling ring's
  //   PROFILE_SCOPE id already uses, so a downloaded record's command_id is
  //   directly comparable against a `beebo profile` capture), command[2:12]
  //   reserved.
  // EVENT_SOURCE_TEXT_CLI: command[0:12] = up to 12 raw ASCII bytes
  //   identifying the command (the text after "set "/"get " for a keyed
  //   command, or the bare command itself for a one-shot action like
  //   "reboot"; zero-padded, not necessarily NUL-terminated if it fills all
  //   12 bytes).
  uint8_t  command[12];
};

union MonRecord {
  uint8_t     kind;     // common discriminant (byte 0 of every arm)
  SyncRecord  sync;
  RxRecord    rx;
  TxRecord    tx;
  RadioRecord radio;
  EnvRecord   env;
  BattRecord  batt;
  TuneRecord  tune;
  EventRecord event;
  SettingRecord setting;
  CommandRecord command;
  uint8_t     raw[16];
};

static_assert(sizeof(SyncRecord)  == 16, "SyncRecord must be 16 bytes");
static_assert(sizeof(RxRecord)    == 16, "RxRecord must be 16 bytes");
static_assert(sizeof(TxRecord)    == 16, "TxRecord must be 16 bytes");
static_assert(sizeof(RadioRecord) == 16, "RadioRecord must be 16 bytes");
static_assert(sizeof(EnvRecord)   == 16, "EnvRecord must be 16 bytes");
static_assert(sizeof(BattRecord)  == 16, "BattRecord must be 16 bytes");
static_assert(sizeof(TuneRecord)  == 16, "TuneRecord must be 16 bytes");
static_assert(sizeof(EventRecord) == 16, "EventRecord must be 16 bytes");
static_assert(sizeof(SettingRecord) == 16, "SettingRecord must be 16 bytes");
static_assert(sizeof(CommandRecord) == 16, "CommandRecord must be 16 bytes");
static_assert(sizeof(MonRecord)   == 16, "MonRecord must be 16 bytes");

class MonRing {
  MonRecord *_buf = nullptr;
  uint32_t  _cap = 0;       // capacity in records
  uint32_t  _head = 0;      // next slot to write
  uint32_t  _count = 0;     // valid records (<= _cap)
  uint32_t  _next_seq = 0;  // seq to assign to the next appended record
  // beebo: this in-class default is effectively unreachable in practice on
  // real hardware -- Beebo::begin() always calls
  // setConfig(_prefs.monring_config) right after computing/loading that
  // field (Beebo.cpp), so the real fresh-device default lives there
  // (examples/multi_role/Beebo.cpp's own monring_config initializer, kept in
  // sync with this one). Both exclude ENV and TUNE (opt-in: ENV is
  // relatively high-volume, TUNE is the still-experimental dynamic-tuning
  // optimizer's own record kind, off until a user deliberately turns
  // repeater.routing.tune.enabled on) but include EVENT (fault transitions/
  // setting changes/command-run trace -- low-volume, always useful). Kept
  // in sync with Beebo.cpp's value anyway so a native unit test constructing
  // a bare MonRing (no Beebo/_prefs involved) sees the same default.
  // IMPORTANT: on an already-provisioned device, this (and Beebo.cpp's)
  // default only ever applies once, when /companion_prefs is first created
  // -- a device whose file predates a given MON_CAP_* bit being added keeps
  // whatever byte it already had persisted; new bits are never retroactively
  // ORed in for existing devices (`beebo settings monitor.<kind>` must be
  // turned on explicitly after such an upgrade).
  uint8_t   _config = (MON_CAP_ALL & ~MON_CAP_ENV & ~MON_CAP_TUNE) | MON_CAP_ENABLED;  // persisted enable + per-kind capture mask
  // beebo: per-EVENT_TYPE capture mask, bit N = EVENT_* id N, 1=captured --
  // a second, finer filter UNDER MON_CAP_EVENT (that bit still gates MON_EVENT
  // capture at all; this only decides which types within it are kept).
  // RAM-only, defaults to all-1s (capture everything, matching pre-existing
  // behavior) -- unlike _config, this isn't persisted: capture volume is a
  // live tuning knob (a busy repeater drowning in tx_ack_success/
  // tx_echo_success records might exclude those while debugging something
  // else), not a standing device policy worth surviving a reboot. See
  // GET/SET_MONRING_EVENT_MASK, beebo/protocol.yaml.
  uint32_t  _event_type_mask = 0xFFFFFFFFu;
  bool      _paused_for_read = false;  // capture was force-disabled by pauseForRead()

  // time base (bookkeeping, never evicts)
  uint32_t  _base = 0;            // running absolute base (epoch seconds)
  uint32_t  _sync_period = 3600;  // re-latch cadence (seconds); variable

  // running RADIO config
  RadioRecord _radio;
  bool        _radio_valid = false;

  // running ENV sample
  EnvRecord _env;
  bool      _env_valid = false;

  // The reference record that governs the current oldest surviving prefix of
  // the ring, for each of the three reference kinds — seeded by init()/
  // clear(), reanchored in O(1) at eviction time (see _store()), replayed via
  // emitStartRef() at read time.
  SyncRecord  start_sync{};
  RadioRecord start_radio{};
  EnvRecord   start_env{};

  // Shared by init()/clear(): latch the base and running radio/env state, and
  // (re)seed the three start-refs from them directly, without consuming a
  // ring slot.
  void _seed(uint32_t now, const RadioRecord &radio, const EnvRecord &env) {
    _base = (now == 0) ? 1 : now;
    _radio = radio; _radio.kind = MON_RADIO; _radio.offset = 0; _radio_valid = true;
    _env   = env;   _env.kind   = MON_ENV;   _env.offset   = 0; _env_valid   = true;

    memset(&start_sync, 0, sizeof(start_sync));
    start_sync.kind = MON_SYNC;
    start_sync.timestamp = _base;
    start_sync.abi_version = MONRING_ABI_VERSION;
    start_radio = _radio;
    start_env = _env;
  }

  // Count of RX records currently resident in the ring (incremented on
  // appendRx, decremented on eviction in _store()). Per-record detail (which
  // endpoint/routing outcome each one carried) is no longer aggregated here --
  // recreate that by downloading and inspecting records offline (`beebo
  // monitor download`) instead of keeping a resident histogram in RAM.
  uint32_t  _rx_count = 0;

  // Count of TX records currently resident in the ring, same
  // incremented-on-append/decremented-on-eviction pattern as _rx_count.
  uint32_t  _tx_count = 0;

  // Lifetime counters (never decremented) for the two RX dispositions that
  // never reach the ring as their own record -- RX_DISP_POOL_FULL (no free
  // packet buffer) and RX_DISP_PARSE_ERR (malformed/oversize frame). Both
  // fire before a finished Packet exists to build a MON_RX record from, so
  // unlike every other disposition there is nothing here for offline
  // download/analysis to reconstruct -- these two scalars (plus the
  // EVENT_RX_POOL_FULL/EVENT_RX_PARSE_ERROR trail bumpRxDropCount() also
  // emits) are the only record of them. See bumpRxDropCount().
  uint32_t  _rx_pool_exhausted_count = 0;
  uint32_t  _rx_parse_error_count = 0;

  // Count of BATT records currently resident in the ring, same
  // incremented-on-append/decremented-on-eviction pattern as _rx_count.
  uint32_t  _batt_count = 0;

  // Count of TUNE records currently resident in the ring, same
  // incremented-on-append/decremented-on-eviction pattern as _rx_count.
  uint32_t  _tune_count = 0;

  // Count of EVENT records currently resident in the ring, same
  // incremented-on-append/decremented-on-eviction pattern as _rx_count.
  uint32_t  _event_count = 0;

  // Count of SETTING/COMMAND records currently resident in the ring, same
  // incremented-on-append/decremented-on-eviction pattern as _rx_count.
  uint32_t  _setting_count = 0;
  uint32_t  _command_count = 0;

  // Resident counts for the reference kinds, same incremented-on-store,
  // decremented-on-eviction pattern as _rx_count (see _store()'s eviction
  // switch). Sync bumps in _ensureSync(), radio/env in noteRadio()/sampleEnv(),
  // all only when a new record is actually stored (real change / relatch).
  uint32_t  _sync_count = 0;
  uint32_t  _radio_count = 0;
  uint32_t  _env_count = 0;

  // Last capture time (capture-time, not live clock, so it is pause-proof).
  // Always the newest record's timestamp, since only the OLDEST record ever
  // evicts — 0 until the first capture.
  uint32_t  _end_time = 0;

  // Called first by every append*/note*/sample* entry point. Latches a new
  // base (and stores a SYNC record carrying it immediately) if none exists
  // yet or the current epoch has run its course — forward semantics: "from
  // here on this is the base", so SYNC always precedes anything using it.
  void _ensureSync(uint32_t now) {
    if (_base == 0 || now < _base || (now - _base) >= _sync_period) {
      _base = (now == 0) ? 1 : now;
      MonRecord r; memset(&r, 0, sizeof(r));
      r.sync.kind = MON_SYNC;
      r.sync.timestamp = _base;
      r.sync.abi_version = MONRING_ABI_VERSION;
      // No real SYNC has ever been stored yet, so start_sync is still just
      // the boot/clear seed — which may predate a clock correction (e.g. the
      // RTC's hardcoded fallback epoch until real mesh traffic sets it).
      // This relatch is about to become the oldest resident reference, so
      // reanchor now rather than waiting for eviction (which may never come
      // if the ring never wraps).
      if (_sync_count == 0) start_sync = r.sync;
      _sync_count++;
      _store(r);
    }
  }

  uint16_t _offset(uint32_t now) const {
    uint32_t d = (now < _base) ? 0 : now - _base;
    return d > 0xFFFF ? 0xFFFF : (uint16_t)d;
  }

  // Raw append of a fully-formed record. Assigns the next seq, wraps the ring.
  uint32_t _store(const MonRecord &rec) {
    if (_count == _cap) {
      // Full ring: _buf[_head] is the oldest record, about to be overwritten.
      // If it is a reference kind, it is the floor for whatever of its
      // dependents are still resident — remember it before it is lost.
      // init()/clear() also seed start_sync/start_radio/start_env directly
      // (so a reader has a reference before the ring ever wraps); this
      // eviction path is what keeps them current once wrapping starts
      // overwriting real sync/radio/env records, superseding the boot/clear
      // seed with the last real value that actually existed.
      // RX/TX evictions symmetrically un-bump the counters bumped at
      // append time, so _rx_count/_tx_count always reflect only what is
      // currently resident, never lifetime totals.
      switch (_buf[_head].kind) {
        case MON_SYNC:
          start_sync = _buf[_head].sync;
          if (_sync_count) _sync_count--;
          break;
        case MON_RADIO:
          start_radio = _buf[_head].radio;
          if (_radio_count) _radio_count--;
          break;
        case MON_ENV:
          start_env = _buf[_head].env;
          if (_env_count) _env_count--;
          break;
        case MON_RX:
          if (_rx_count) _rx_count--;
          break;
        case MON_TX:
          if (_tx_count) _tx_count--;
          break;
        case MON_BATT:
          if (_batt_count) _batt_count--;
          break;
        case MON_TUNE:
          if (_tune_count) _tune_count--;
          break;
        case MON_EVENT:
          if (_event_count) _event_count--;
          break;
        case MON_SETTING:
          if (_setting_count) _setting_count--;
          break;
        case MON_COMMAND:
          if (_command_count) _command_count--;
          break;
        default: break;
      }
    }
    uint32_t seq = _next_seq;
    _buf[_head] = rec;
    _head = (_head + 1) % _cap;
    if (_count < _cap) _count++;
    _next_seq++;
    return seq;
  }

public:
  // Allocate `bytes` of PSRAM for the ring and seed start_sync/start_radio/
  // start_env from the given radio/env snapshot, in one call. Returns false
  // if allocation fails (ring then stays disabled and appends are no-ops).
  // Call once at boot.
  bool init(uint8_t *psram, uint32_t bytes, uint32_t now, const RadioRecord &radio, const EnvRecord &env) {
    if (psram == nullptr || bytes < sizeof(MonRecord)) return false;
    _buf = (MonRecord *)psram;
    _cap = bytes / sizeof(MonRecord);
    _head = _count = _next_seq = 0;
    _seed(now, radio, env);
    return true;
  }

  bool     allocated() const { return _buf != nullptr; }
  void     setConfig(uint8_t c) { _config = c; }
  uint8_t  config() const { return _config; }
  bool     enabled() const { return _config & MON_CAP_ENABLED; }
  void     setEventTypeMask(uint32_t m) { _event_type_mask = m; }
  uint32_t eventTypeMask() const { return _event_type_mask; }
  uint32_t capacity() const { return _cap; }
  uint32_t count() const { return _count; }
  uint32_t oldestSeq() const { return _next_seq - _count; }
  uint32_t nextSeq() const { return _next_seq; }
  void     setSyncPeriod(uint32_t s) { if (s) _sync_period = s; }
  uint32_t rxCount() const { return _rx_count; }
  uint32_t battCount() const { return _batt_count; }
  uint32_t tuneCount() const { return _tune_count; }
  uint32_t eventCount() const { return _event_count; }
  uint32_t settingCount() const { return _setting_count; }
  uint32_t commandCount() const { return _command_count; }
  uint32_t txCount() const { return _tx_count; }
  uint32_t syncCount() const { return _sync_count; }
  uint32_t radioCount() const { return _radio_count; }
  uint32_t envCount() const { return _env_count; }
  // See bumpRxDropCount()/_rx_pool_exhausted_count/_rx_parse_error_count above.
  uint32_t rxPoolExhaustedCount() const { return _rx_pool_exhausted_count; }
  uint32_t rxParseErrorCount() const { return _rx_parse_error_count; }
  // Epoch of the oldest resident record's governing SYNC: the real SYNC still
  // resident at oldestSeq() if there is one (it is always the most current
  // reference when present), otherwise the reanchored start_sync — either the
  // boot/clear seed (no wrap yet) or the last real SYNC that fell off (wrapped).
  uint32_t startTime() const {
    MonRecord rec;
    if (peek(oldestSeq(), &rec) && rec.kind == MON_SYNC) return rec.sync.timestamp;
    return start_sync.timestamp;
  }
  uint32_t endTime() const { return _end_time; }

  // Freeze capture for the duration of a multi-page read (append/note/sample
  // are no-ops while paused, per the enabled() gating below); resumeAfterRead()
  // restores the previous state, so a manually-paused ring stays paused.
  void pauseForRead() {
    _paused_for_read = enabled();
    _config &= ~MON_CAP_ENABLED;
  }
  void resumeAfterRead() {
    if (_paused_for_read) _config |= MON_CAP_ENABLED;
    _paused_for_read = false;
  }

  // Wipes the ring AND re-seeds start_sync/start_radio/start_env from the
  // given radio/env snapshot, in one call — so the ring is never left without
  // a valid sync/radio/env reference, and a stale pre-clear config can't be
  // mistaken for "unchanged" by noteRadio()/sampleEnv() and silently skip
  // storing a fresh record after the clear.
  void clear(uint32_t now, const RadioRecord &radio, const EnvRecord &env) {
    _head = _count = 0; _next_seq = 0;
    _rx_count = _tx_count = _sync_count = _radio_count = _env_count = _batt_count = _tune_count = _event_count = _setting_count = _command_count = _end_time = 0;
    // beebo: _rx_pool_exhausted_count/_rx_parse_error_count are deliberately
    // NOT reset here -- they're lifetime-since-boot counters (see their
    // declaration above), and clearing the ring shouldn't erase evidence that
    // the packet pool was exhausted earlier in this boot.
    _seed(now, radio, env);
  }

  // Append one fully-resolved reception: the caller fills every RxRecord field
  // (including the final disp byte) before calling this, so it is written to
  // the ring exactly once. Returns the assigned seq, or UINT32_MAX if nothing
  // was stored (ring disabled/unallocated).
  uint32_t appendRx(RxRecord rx, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_RX) || _buf == nullptr) return 0xFFFFFFFFu;
    _ensureSync(now);
    _end_time = now;
    _rx_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.rx = rx;
    r.rx.kind = MON_RX;
    r.rx.offset = _offset(now);
    return _store(r);
  }

  // Append one of our own transmissions (kind/offset stamped here).
  void appendTx(TxRecord tx, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_TX) || _buf == nullptr) return;
    _ensureSync(now);
    _end_time = now;
    _tx_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.tx = tx;
    r.tx.kind = MON_TX;
    r.tx.offset = _offset(now);
    _store(r);
  }

  // Append one battery sample (kind/offset stamped here). Unconditional, like
  // appendTx() -- every sample the caller took is charted, not just changes,
  // since the point is to see the full Vbat trace (IR-drop sag/recovery,
  // idle vs forced reads) rather than a diffed reference like RADIO/ENV.
  void appendBatt(BattRecord batt, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_BATT) || _buf == nullptr) return;
    _ensureSync(now);
    _end_time = now;
    _batt_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.batt = batt;
    r.batt.kind = MON_BATT;
    r.batt.offset = _offset(now);
    _store(r);
  }

  // Append one tuning-optimizer decision (kind/offset stamped here).
  // Unconditional, like appendTx()/appendBatt() -- every tick the controller
  // runs is charted (observe-only or applied), not just changes, since the
  // point is to see the full decision history for offline review before any
  // param is promoted to live actuation.
  void appendTune(TuneRecord tune, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_TUNE) || _buf == nullptr) return;
    _ensureSync(now);
    _end_time = now;
    _tune_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.tune = tune;
    r.tune.kind = MON_TUNE;
    r.tune.offset = _offset(now);
    _store(r);
  }

  // Append one general-purpose event (kind/offset stamped here). Unlike
  // ENV/RADIO's dedup-on-change, every call charts a record -- an event is
  // by definition a discrete occurrence, not a running state to diff
  // against.
  void appendEvent(EventRecord event, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_EVENT) || _buf == nullptr) return;
    if (!(_event_type_mask & (1u << event.event_type))) return;
    _ensureSync(now);
    _end_time = now;
    _event_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.event = event;
    r.event.kind = MON_EVENT;
    r.event.offset = _offset(now);
    _store(r);
  }

  // Append one admin-visible setting change (kind/offset stamped here).
  // Gated by the same MON_CAP_EVENT bit as appendEvent() -- see MON_SETTING's
  // comment for why this has its own kind/struct but shares the capture bit.
  void appendSetting(SettingRecord setting, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_EVENT) || _buf == nullptr) return;
    _ensureSync(now);
    _end_time = now;
    _setting_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.setting = setting;
    r.setting.kind = MON_SETTING;
    r.setting.offset = _offset(now);
    _store(r);
  }

  // Append one command-run record (kind/offset stamped here). Same capture
  // gating as appendSetting()/appendEvent() -- see MON_COMMAND's comment.
  void appendCommand(CommandRecord command, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_EVENT) || _buf == nullptr) return;
    _ensureSync(now);
    _end_time = now;
    _command_count++;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.command = command;
    r.command.kind = MON_COMMAND;
    r.command.offset = _offset(now);
    _store(r);
  }

  // Note the current radio config. On a real change, store the NEW config as
  // a record immediately (forward semantics), then adopt it as the running
  // state. Same shape as sampleEnv() (and shares its caller-builds-the-record
  // pattern with _seed()) so both reference kinds are seeded/diffed
  // identically.
  void noteRadio(RadioRecord radio, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_RADIO) || _buf == nullptr) return;
    bool changed = !_radio_valid || _radio.freq != radio.freq || _radio.sf != radio.sf ||
                   _radio.bw != radio.bw || _radio.cr != radio.cr ||
                   _radio.tx_power != radio.tx_power || _radio.flags != radio.flags;
    if (!changed) return;
    _ensureSync(now);
    _radio = radio;
    _radio_valid = true;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.radio = _radio;
    r.radio.kind = MON_RADIO;
    r.radio.offset = _offset(now);
    _radio_count++;
    _store(r);
  }

  // Note the current environment sample. On a real change, store the NEW
  // sample as a record immediately, then adopt it as the running state. Same
  // rule as noteRadio().
  void sampleEnv(EnvRecord env, uint32_t now) {
    if (!enabled() || !(_config & MON_CAP_ENV) || _buf == nullptr) return;
    bool changed = !_env_valid || _env.noise_floor != env.noise_floor ||
                   _env.batt_mv != env.batt_mv || _env.temp_c != env.temp_c ||
                   _env.free_heap != env.free_heap || _env.pool_free != env.pool_free ||
                   _env.tx_queue != env.tx_queue || _env.err_flags != env.err_flags ||
                   _env.cad_busy_events != env.cad_busy_events;
    if (!changed) return;
    _ensureSync(now);
    _env = env;
    _env_valid = true;
    MonRecord r; memset(&r, 0, sizeof(r));
    r.env = _env;
    r.env.kind = MON_ENV;
    r.env.offset = _offset(now);
    _env_count++;
    _store(r);
  }

  // Look up the resident record at `seq` without consuming anything. Returns
  // false if seq isn't currently in [oldestSeq(), nextSeq()).
  bool peek(uint32_t seq, MonRecord *out) const {
    if (_buf == nullptr || _count == 0) return false;
    uint32_t oldest = _next_seq - _count;
    if (seq < oldest || seq >= _next_seq) return false;
    uint32_t oldest_idx = (_head + _cap - _count) % _cap;
    uint32_t idx = (oldest_idx + (seq - oldest)) % _cap;
    *out = _buf[idx];
    return true;
  }

  // Fill `dest` with the stashed start-ref for one reference kind. Always
  // populated from the moment init()/clear() run, and kept current by
  // _store() across evictions.
  bool emitStartRef(uint8_t kind, MonRecord *dest) const {
    switch (kind) {
      case MON_SYNC:  memset(dest, 0, sizeof(*dest)); dest->sync  = start_sync;  return true;
      case MON_RADIO: memset(dest, 0, sizeof(*dest)); dest->radio = start_radio; return true;
      case MON_ENV:   memset(dest, 0, sizeof(*dest)); dest->env   = start_env;   return true;
      default: return false;
    }
  }

  // Bump the lifetime counter for a disposition reason that never reaches the
  // ring as its own record (no Packet was allocated, or it was freed before a
  // record could be built) — RX_DISP_POOL_FULL (13) / RX_DISP_PARSE_ERR (12),
  // see Dispatcher.h. Never decremented, since there is no corresponding
  // record for eviction to ever remove. Returns the counter's new value (0 if
  // `reason` isn't one of the two handled here) so the caller can stamp it
  // into an EVENT_RX_POOL_FULL/EVENT_RX_PARSE_ERROR record without a second
  // lookup.
  uint32_t bumpRxDropCount(uint8_t reason) {
    if (reason == 13) return ++_rx_pool_exhausted_count;
    if (reason == 12) return ++_rx_parse_error_count;
    return 0;
  }

private:
  // Endpoint axis: write-once, except ACCEPTED which always wins.
  static void _setEndpoint(uint8_t &disp, uint8_t v, bool force) {
    if (force || (disp & RXREC_EP_MASK) == RXEP_NONE)
      disp = (disp & ~RXREC_EP_MASK) | (v & RXREC_EP_MASK);
  }
  // Routing axis: write-once (a frame gets one relay outcome).
  static void _setRouting(uint8_t &disp, uint8_t v) {
    if (((disp & RXREC_RT_MASK) >> RXREC_RT_SHIFT) == RXRT_NONE)
      disp = (disp & ~RXREC_RT_MASK) | ((v << RXREC_RT_SHIFT) & RXREC_RT_MASK);
  }

public:
  // Accumulate one RX disposition reason onto a not-yet-committed record's disp
  // byte, routed to the right axis so several calls accumulate (e.g. ACCEPTED
  // then FORWARDED coexist) instead of masking. Call this as a packet's
  // disposition resolves; the byte is only written to the ring once, by
  // appendRx(), after disposition is fully known.
  static void applyDisposition(uint8_t &disp, uint8_t reason) {
    switch (reason) {
      case 2:  _setRouting(disp, RXRT_FORWARDED);        break;  // RX_DISP_FORWARDED
      case 6:  _setRouting(disp, RXRT_NO_FORWARD);       break;  // RX_DISP_NO_FORWARD
      case 15: _setRouting(disp, RXRT_PATH_FULL);        break;  // RX_DISP_PATH_FULL
      case 14: disp |= RXREC_DISTILL;                    break;  // RX_DISP_DISTILL_BAD
      case 17: disp |= RXREC_QUEUE_FULL;                 break;  // RX_DISP_QUEUE_FULL
      case 1:  _setEndpoint(disp, RXEP_ACCEPTED, true);  break;  // ACCEPTED always wins
      case 3:  _setEndpoint(disp, RXEP_DUP, false);         break;
      case 4:  _setEndpoint(disp, RXEP_NOT_FOR_US, false);  break;
      case 5:  _setEndpoint(disp, RXEP_WRONG_HOP, false);   break;
      case 7:  _setEndpoint(disp, RXEP_FILTERED, false);    break;
      case 8:  _setEndpoint(disp, RXEP_INCOMPLETE, false);  break;
      case 9:  _setEndpoint(disp, RXEP_FORGED_SIG, false);  break;
      case 10: _setEndpoint(disp, RXEP_SELF, false);        break;
      case 11: _setEndpoint(disp, RXEP_UNKNOWN_TYPE, false); break;
      case 12: _setEndpoint(disp, RXEP_PARSE_ERR, false);   break;
      case 13: _setEndpoint(disp, RXEP_POOL_FULL, false);   break;
      case 16: _setEndpoint(disp, RXEP_MISC_DROP, false);   break;
      default: break;  // RX_DISP_NONE / unknown — leave unresolved
    }
  }

  // ---- Live objective functions: QoS (what the tuning optimizer maximizes)
  // and SoH (is the node's own infrastructure intact) -- DYNAMIC_OPTIMIZER_
  // PLAN.md item 10, reward redesigned per the same plan's "goodput" section
  // (2026-08-24): QoS alone (a confirmed/attempted RATIO) is structurally
  // blind to routing VOLUME, and specifically blind to any parameter (radio
  // txpower, FEM LNA, RX boosted gain) whose main effect is on how many
  // packets are receivable/forwardable at all rather than the ratio's
  // ability to confirm ones already attempted -- see 802.11 rate-adaptation
  // (SampleRate/Minstrel: expected throughput = attempt_rate x P(success),
  // never P(success) alone) for the precedent. `computeQos()` below is now
  // scoped to pure link-confirmation-quality ("confirm ratio") -- rx_drop
  // is deliberately no longer part of its denominator (that capacity signal
  // is SoH's job, see SohStats below) -- and `computeRos()` reports
  // the companion raw-volume half. Both are exported side by side; a single
  // combined goodput-style reward (ros_count, baseline-normalized, times
  // confirm ratio) is the target shape once the decision-window gate's
  // per-window deltas exist -- this step ships the two ingredients on
  // lifetime counters first, not yet the combined/normalized product.
  //
  // QoS and SoH remain two DIFFERENT numbers, not two views of the same
  // one: a node can be perfectly healthy (SoH 100%) while its delivery
  // quality is mediocre for reasons entirely outside its own control (RF
  // noise, topology, a neighbor's own congestion) -- QoS reflects that
  // outcome, SoH doesn't conflate it with "something is wrong with THIS
  // node." Both are computed fresh on every call from the same lifetime
  // counters the event catalog (see beebo-cli docs) already tracks --
  // nothing new is captured, this is pure arithmetic over existing
  // diagnostics, same spirit as TuneController::txConfirmReward() (which
  // QoS *is* -- see below).

  // QoS inputs -- identical shape to TuneController::TxConfirmStats
  // (TuneController.h delegates its txConfirmReward() here so there is only
  // one implementation of this formula, not two that could drift apart).
  struct QosStats {
    uint32_t ack_success_count;
    uint32_t ack_timeout_count;
    uint32_t echo_attempt_count;
    uint32_t echo_success_count;
  };

  // QoS ("confirm ratio") = confirmed-delivered / attempted, 0-10000 scaled
  // (0-100.00%). 0 (not a meaningful "0% success") when there's no exposure
  // yet (denominator 0). Deliberately does NOT include rx_drop (pool-
  // exhausted/parse-error/queue-full) in the denominator -- that's a
  // capacity-fault signal already tracked by SoH below, and folding it in
  // here conflated "this link's confirmations are weak" with "this node is
  // dropping traffic for unrelated internal-resource reasons," which also
  // structurally hid any parameter (radio txpower/RX gain) whose effect is
  // mostly on routed volume rather than this ratio -- see the class-level
  // comment above. Same lifetime-cumulative-counter caveat as everywhere
  // else this pattern is used: early history dominates, less responsive to
  // recent conditions as the denominator grows over uptime.
  static uint16_t computeQos(const QosStats &s) {
    uint32_t numerator = s.ack_success_count + s.echo_success_count;
    uint32_t denominator = computeQosExposure(s);
    if (denominator == 0) return 0;
    uint32_t scaled = (numerator * 10000UL) / denominator;
    return (uint16_t)(scaled > 10000UL ? 10000UL : scaled);
  }

  // computeQos()'s denominator, exported on its own so a caller (`beebo
  // check`) can tell "zero confirmable attempts yet" (exposure 0) apart
  // from "confirmable attempts happened and none succeeded" (exposure > 0,
  // qos still 0) -- computeQos() alone collapses both to the same 0%,
  // which reads as a real delivery failure even on a freshly-booted node.
  static uint32_t computeQosExposure(const QosStats &s) {
    return s.ack_success_count + s.ack_timeout_count + s.echo_attempt_count;
  }

  // Raw routing volume: total confirmed-delivered count (DM ACKs + flood
  // echoes), lifetime. The companion half of the goodput reward -- QoS
  // alone can't distinguish a node routing 1 packet/hour at 100% from one
  // routing 1000/hour at 100%; this is the number that can. Not yet
  // normalized against a rolling baseline (that needs the decision-window
  // gate's per-window deltas -- see DYNAMIC_OPTIMIZER_PLAN.md) or combined
  // with QoS into a single scalar -- exported alongside it for now so
  // `beebo check`/`beebo monitor` can show both while that design lands.
  static uint32_t computeRos(const QosStats &s) {
    return s.ack_success_count + s.echo_success_count;
  }

  // SoH inputs -- every lifetime counter that represents THIS node's own
  // internal capacity/resource running out, i.e. the EVENT_*/MON_EVENT
  // causes that are about the node itself rather than the RF environment or
  // mesh traffic from other nodes. Deliberately excludes rx_parse_error
  // (malformed frames are usually someone else's RF noise or firmware
  // mismatch, not this node's own health) and rx_dedup_table_full (by design
  // this fires on nearly every insert once the table has filled once --
  // see EVENT_RX_DEDUP_TABLE_FULL's own comment -- so it's expected background
  // volume, not a fault signal; folding it in here would make SoH converge
  // toward 0 on every long-uptime node regardless of actual health).
  struct SohStats {
    uint32_t tx_pool_full_count;
    uint32_t rx_pool_exhausted_count;
    uint32_t tx_cad_timeout_count;
    uint32_t rx_start_timeout_count;
    uint32_t tx_queue_full_count;
    uint32_t rx_queue_full_count;
    uint32_t link_tx_queue_full_count;
    uint32_t link_rx_queue_full_count;
    uint32_t tx_ack_overflow_count;
    uint32_t tx_echo_overflow_count;
    // Lifetime RX+TX packet activity (Dispatcher::getNumRecvFlood()/
    // getNumRecvDirect()/getNumSentFlood()/getNumSentDirect()) -- the
    // "opportunity" denominator: how much traffic this node has actually
    // processed, so SoH is a FAULT RATE, not a raw cumulative fault count
    // (a long-uptime node isn't automatically less healthy than a
    // freshly-booted one just for having lived through more packets).
    uint32_t rx_activity_count;
    uint32_t tx_activity_count;
  };

  // SoH = 1 - (internal faults / total activity), 0-10000 scaled. No
  // activity yet -> 10000 (nothing has gone wrong is not the same as
  // "unhealthy"); faults >= activity (pathological/very early boot) floors
  // at 0 rather than underflowing. First-pick formula, like
  // ROLLBACK_THRESHOLD/MAX_ECHO_HASHES elsewhere in this codebase --
  // equal weighting per fault type, no real-traffic data behind that choice
  // yet; revisit once there's live gatto/beebo history to check it against.
  static uint16_t computeSoh(const SohStats &s) {
    uint32_t faults = s.tx_pool_full_count + s.rx_pool_exhausted_count +
                       s.tx_cad_timeout_count + s.rx_start_timeout_count +
                       s.tx_queue_full_count + s.rx_queue_full_count +
                       s.link_tx_queue_full_count + s.link_rx_queue_full_count +
                       s.tx_ack_overflow_count + s.tx_echo_overflow_count;
    uint32_t activity = s.rx_activity_count + s.tx_activity_count;
    if (activity == 0) return 10000;
    if (faults >= activity) return 0;
    uint32_t scaled = ((activity - faults) * 10000UL) / activity;
    return (uint16_t)(scaled > 10000UL ? 10000UL : scaled);
  }

public:
  // Serialize records with seq >= after_seq, OLDEST first (walking forward
  // from the tail), up to max_len bytes (whole records only). Returns the
  // byte count and writes the number of records emitted to *out_count.
  //
  // Only safe to call while the ring is frozen (see pauseForRead()): _head/
  // _count don't change mid-walk, so this is a plain bounded loop with no
  // concurrent-mutation reasoning needed.
  int serialize(uint8_t *dest, size_t max_len, uint32_t after_seq, uint32_t *out_count) const {
    *out_count = 0;
    if (_buf == nullptr || _count == 0) return 0;
    uint32_t oldest = _next_seq - _count;
    uint32_t from = (after_seq > oldest) ? after_seq : oldest;
    if (from >= _next_seq) return 0;

    uint32_t available = _next_seq - from;
    uint32_t avail = max_len / sizeof(MonRecord);
    uint32_t n = (available < avail) ? available : avail;
    uint32_t oldest_idx = (_head + _cap - _count) % _cap;

    int pos = 0;
    for (uint32_t j = 0; j < n; j++) {
      uint32_t seq = from + j;                          // oldest first
      uint32_t idx = (oldest_idx + (seq - oldest)) % _cap;
      memcpy(&dest[pos], &_buf[idx], sizeof(MonRecord));
      pos += sizeof(MonRecord);
    }
    *out_count = n;
    return pos;
  }
};
