#pragma once

#include <stdint.h>
#include "MonRing.h"

// beebo: the dynamic optimizer's decision-window evaluator. A window opens
// with a snapshot of the reward counters, and closes by rule -- once the
// node has heard at least one packet and `min_ms` elapsed, or as
// insufficient_data at `max_ms` with no activity at all -- so sparse nodes
// get longer windows and busy ones shorter. The length is deliberately not
// something the optimizer tunes: it changes measurement precision, not
// delivery, so a reward could not judge it. Reward inputs are wrap-safe
// counter DELTAS over the window, never lifetime totals.
//
// Validity is RX activity, not confirmable-attempt count (`exposure`, still
// computed and reported, just no longer the gate): a window with real RX but
// few or zero forwards is real information -- for a repeater, failing to
// convert reception into forwards IS the service quality being measured, not
// noise to discard -- and gets a real (if noisy) confirm_ratio/ros_rate;
// only a window that heard nothing at all has no information. Per-block
// noise at low exposure is what the paired statistics (TrialFSM's mean/SE
// across many blocks) already exist to absorb, not something to solve by
// discarding thin windows outright.
//
// This class only measures. accepted/rollback is the controller's call (it
// owns the comparison against the value it just changed); measured windows
// report `confirm_ratio`, the per-hour `ros` rate and that rate normalized
// against a RAM-only rolling median of the last BASELINE_WINDOWS exposed
// windows. Until MIN_BASELINE windows exist, a window is insufficient_data
// (cold start) but still feeds the baseline. No dynamic allocation.
class EvalWindow {
public:
  static constexpr uint8_t BASELINE_WINDOWS = 8;
  static constexpr uint8_t MIN_BASELINE = 4;
  // First-pick constant, same "revisit once real data exists" posture as
  // AdaptiveController::ROLLBACK_THRESHOLD.
  static constexpr uint8_t GUARDRAIL_UTIL_PCT = 80;
  // Bumped 1 -> 2 when RX Valid/Invalid/Errors and TX Dispatched were added
  // as a third MON_EVAL slot (EvalRecordC, MonRing.h) -- a version-1 run
  // has no slot 3. Not itself load-bearing for the CLI's slot-count
  // decision (that's RLOG_CONT_BIT chaining, self-describing per run); this
  // is the human-readable "what format wrote this" marker.
  static constexpr uint8_t RECORD_VERSION = 2;

  struct Config {
    uint32_t min_ms = 300000;
    uint32_t max_ms = 3600000;
    // false: no rolling baseline and no cold start -- a window is measured as
    // soon as it is exposed (used by TrialFSM blocks, which compare arms
    // directly rather than against a baseline).
    bool use_baseline = true;
  };

  struct Result {
    bool     measured;        // false: see `outcome` (insufficient_data/invalidated)
    uint8_t  outcome;         // EVAL_* when !measured
    uint8_t  flags;           // EVALF_*
    uint32_t exposure;        // confirmable attempts in the window (diagnostic only)
    uint32_t ros_count;       // confirmed deliveries in the window
    uint16_t confirm_ratio;   // 0-10000
    uint8_t  util_pct;
    uint8_t  cad_busy_pct;
    uint32_t window_ms;
    uint32_t ros_rate;        // ros per hour over this window
    uint32_t baseline_ros_rate;  // rolling median, ros per hour (before this window)
    uint16_t ros_norm;        // ros_rate / baseline, fixed point (1000 = 1.0)
    uint8_t  n_baseline;      // windows in the baseline (before this window)
    // Neighbor-table reach for the window (NeighborReach::scan()); filled in
    // by the caller after poll() closes the window, not measured here.
    uint8_t  reach_heard;
    uint8_t  reach_marginal;
    // On-device RX/TX breakdown (EvalRecordC) -- deltas over the window,
    // same convention as ros_count/exposure above. rx_valid duplicates
    // what `active`/window-validity already reads off `rx_count` (see
    // open()/poll() below); it just wasn't stored into a visible field
    // until now.
    uint32_t rx_valid;       // RadioLibWrapper::getPacketsRecv() delta
    uint32_t rx_invalid;     // MonRing::rxParseErrorCount() delta
    uint32_t rx_errors;      // RadioLibWrapper::getPacketsRecvErrors() delta
    uint32_t tx_dispatched;  // RadioLibWrapper::getPacketsSent() delta
  };

  void begin(const Config &cfg) {
    _cfg = cfg;
    _open = false;
    resetBaseline();
  }

  void resetBaseline() {
    _n = 0;
    _next = 0;
  }

  uint8_t baselineCount() const { return _n; }
  bool isOpen() const { return _open; }
  uint16_t windowId() const { return _window_id; }

  // Replace the window rule. Keeps the baseline; a window already open was
  // opened under the old rule, so it is invalidated (EVALF_CONFIG).
  void setConfig(const Config &cfg) {
    _cfg = cfg;
    if (_open) _flags |= EVALF_CONFIG;
  }

  // `cad_wait_ms` is the cumulative TX-blocked time (Dispatcher::
  // getTxWaitCadMs(): CAD-busy plus TX backoff); its delta over the window
  // is `cad_busy_pct`. `rx_count` is the radio's lifetime received-packet
  // counter (RadioLibWrapper::getPacketsRecv()); its delta over the window
  // is what decides validity now, see class comment, and is also reported
  // as `rx_valid`. `rx_invalid`/`rx_errors`/`tx_count` are the lifetime
  // counterparts of Result's rx_invalid/rx_errors/tx_dispatched
  // (MonRing::rxParseErrorCount(), RadioLibWrapper::getPacketsRecvErrors(),
  // RadioLibWrapper::getPacketsSent()) -- diagnostic only, none of them
  // gate anything.
  void open(uint32_t now_ms, const MonRing::QosStats &snap, uint16_t window_id,
            uint32_t cad_wait_ms = 0, uint32_t rx_count = 0,
            uint32_t rx_invalid = 0, uint32_t rx_errors = 0, uint32_t tx_count = 0) {
    _open = true;
    _t0 = now_ms;
    _snap = snap;
    _cad0 = cad_wait_ms;
    _rx0 = rx_count;
    _rxinv0 = rx_invalid;
    _rxerr0 = rx_errors;
    _tx0 = tx_count;
    _window_id = window_id;
    _flags = 0;
    _util_sum = 0;
    _util_n = 0;
  }

  // Packet-pool occupancy sample (0-100), called about once a second by the
  // firmware loop; `util_pct` is their mean over the window (a 16-slot pool
  // peaks at 100% in any normal burst, so peak would be noise). Ignored while
  // no window is open.
  void sampleUtil(uint8_t pool_used_pct) {
    if (!_open) return;
    _util_sum += (pool_used_pct > 100) ? 100 : pool_used_pct;
    _util_n++;
  }

  // Mark the open window invalid (config change, capture gap, ...). It closes
  // as `invalidated` on the next poll().
  void invalidate(uint8_t evalf) { _flags |= evalf; }

  // Returns true (and fills `out`) when the window closes this call.
  bool poll(uint32_t now_ms, const MonRing::QosStats &cur, uint32_t cad_wait_ms,
            Result &out, uint32_t rx_count = 0,
            uint32_t rx_invalid = 0, uint32_t rx_errors = 0, uint32_t tx_count = 0) {
    if (!_open) return false;
    uint32_t elapsed = now_ms - _t0;  // unsigned: millis() rollover-safe
    uint32_t d_ack_ok = cur.ack_success_count - _snap.ack_success_count;
    uint32_t d_ack_to = cur.ack_timeout_count - _snap.ack_timeout_count;
    uint32_t d_echo_att = cur.echo_attempt_count - _snap.echo_attempt_count;
    uint32_t d_echo_ok = cur.echo_success_count - _snap.echo_success_count;
    uint32_t d_rx = rx_count - _rx0;
    uint32_t d_rxinv = rx_invalid - _rxinv0;
    uint32_t d_rxerr = rx_errors - _rxerr0;
    uint32_t d_tx = tx_count - _tx0;
    // A counter that went backwards (device-side reset) shows up as a huge
    // unsigned delta.
    if (d_ack_ok > 0x80000000u || d_ack_to > 0x80000000u ||
        d_echo_att > 0x80000000u || d_echo_ok > 0x80000000u ||
        d_rx > 0x80000000u || d_rxinv > 0x80000000u ||
        d_rxerr > 0x80000000u || d_tx > 0x80000000u) {
      _flags |= EVALF_COUNTER_RST;
    }

    memset(&out, 0, sizeof(out));
    out.flags = _flags;
    out.util_pct = _util_n ? (uint8_t)(_util_sum / _util_n) : 0;
    uint64_t cad_pct = elapsed ? (uint64_t)(cad_wait_ms - _cad0) * 100ULL / elapsed : 0;
    out.cad_busy_pct = (uint8_t)(cad_pct > 100 ? 100 : cad_pct);
    out.window_ms = elapsed;

    if (_flags) {
      _open = false;
      out.measured = false;
      out.outcome = EVAL_INVALIDATED;
      return true;
    }

    MonRing::QosStats delta{d_ack_ok, d_ack_to, d_echo_att, d_echo_ok};
    out.exposure = MonRing::computeQosExposure(delta);
    out.ros_count = MonRing::computeRos(delta);
    out.confirm_ratio = MonRing::computeQos(delta);
    out.rx_valid = d_rx;
    out.rx_invalid = d_rxinv;
    out.rx_errors = d_rxerr;
    out.tx_dispatched = d_tx;

    bool active = d_rx > 0;
    if (!(active && elapsed >= _cfg.min_ms) && elapsed < _cfg.max_ms) return false;

    _open = false;
    if (!active) {
      out.measured = false;
      out.outcome = EVAL_INSUFFICIENT_DATA;
      return true;
    }

    out.ros_rate = elapsed ? (uint32_t)((uint64_t)out.ros_count * 3600000ULL / elapsed) : 0;
    out.n_baseline = _n;
    out.baseline_ros_rate = medianRate();
    if (out.baseline_ros_rate > 0) {
      uint64_t norm = (uint64_t)out.ros_rate * 1000ULL / out.baseline_ros_rate;
      out.ros_norm = (uint16_t)(norm > 0xFFFF ? 0xFFFF : norm);
    }
    if (out.util_pct > GUARDRAIL_UTIL_PCT) out.flags |= EVALF_GUARDRAIL;

    if (!_cfg.use_baseline) {
      out.measured = true;
      out.outcome = EVAL_ACCEPTED;
      return true;
    }

    // Every active window feeds the baseline, cold-start ones included.
    _rates[_next] = out.ros_rate;
    _next = (uint8_t)((_next + 1) % BASELINE_WINDOWS);
    if (_n < BASELINE_WINDOWS) _n++;

    if (out.n_baseline < MIN_BASELINE) {
      out.measured = false;
      out.outcome = EVAL_INSUFFICIENT_DATA;
    } else {
      out.measured = true;
      out.outcome = EVAL_ACCEPTED;  // placeholder; the controller decides accepted/rollback
    }
    return true;
  }

  // Build the three MON_EVAL slots from a closed window. `outcome` is the
  // final EVAL_* the controller decided (accepted/rollback for a measured
  // window, else the result's own).
  static void toRecords(const Result &r, uint16_t window_id, uint8_t outcome,
                        EvalRecordA &a, EvalRecordB &b, EvalRecordC &c) {
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    memset(&c, 0, sizeof(c));
    a.window_id = window_id;
    a.outcome = outcome;
    a.flags = r.flags;
    a.exposure = sat16(r.exposure);
    a.ros_count = sat16(r.ros_count);
    a.confirm_ratio = r.confirm_ratio;
    a.util_pct = r.util_pct;
    a.version = RECORD_VERSION;
    b.window_id = window_id;
    b.window_s = sat16(r.window_ms / 1000);
    b.baseline_ros_rate = sat16(r.baseline_ros_rate);
    b.ros_norm = r.ros_norm;
    b.n_baseline = r.n_baseline;
    b.cad_busy_pct = r.cad_busy_pct;
    b.reach_heard = r.reach_heard;
    b.reach_marginal = r.reach_marginal;
    c.window_id = window_id;
    c.rx_valid = sat16(r.rx_valid);
    c.rx_invalid = sat16(r.rx_invalid);
    c.rx_errors = sat16(r.rx_errors);
    c.tx_dispatched = sat16(r.tx_dispatched);
  }

private:
  static uint16_t sat16(uint32_t v) { return (uint16_t)(v > 0xFFFF ? 0xFFFF : v); }

  // Median of the baseline rates (mean of the middle two for an even count).
  uint32_t medianRate() const {
    if (_n == 0) return 0;
    uint32_t tmp[BASELINE_WINDOWS];
    for (uint8_t i = 0; i < _n; i++) tmp[i] = _rates[i];
    for (uint8_t i = 1; i < _n; i++) {  // insertion sort, n <= 8
      uint32_t v = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = v;
    }
    if (_n & 1) return tmp[_n / 2];
    return (uint32_t)(((uint64_t)tmp[_n / 2 - 1] + tmp[_n / 2]) / 2);
  }

  Config _cfg;
  bool _open = false;
  uint32_t _t0 = 0;
  uint32_t _cad0 = 0;
  uint32_t _rx0 = 0;
  uint32_t _rxinv0 = 0;
  uint32_t _rxerr0 = 0;
  uint32_t _tx0 = 0;
  uint32_t _util_sum = 0;
  uint32_t _util_n = 0;
  uint16_t _window_id = 0;
  uint8_t _flags = 0;
  MonRing::QosStats _snap{};
  uint32_t _rates[BASELINE_WINDOWS] = {};
  uint8_t _n = 0;
  uint8_t _next = 0;
};
