#pragma once
#include <stdint.h>

// beebo: NodePrefs.batt_present values -- companion_radio and simple_repeater
// each have their own NodePrefs (see CommonCLI.h / examples/companion_radio/
// NodePrefs.h), so these live here rather than being pulled from either one.
#define BATT_PRESENT_UNKNOWN  0
#define BATT_PRESENT_NO       1  // no battery -- external power only ("plugged")
#define BATT_PRESENT_YES      2

// beebo: battery charge-trend state machine, shared between companion_radio
// and simple_repeater so the two firmwares can't drift on thresholds.
#define BATT_STATE_INIT         0  // no anchor seeded yet, or no edge seen since seeding
#define BATT_STATE_CHARGING     2  // voltage rose past hysteresis since last sample
#define BATT_STATE_DISCHARGING  3  // voltage fell past hysteresis since last sample
#define BATT_STATE_CHARGED      4  // charging trend reached charged_mv (real battery, full)
#define BATT_STATE_PLUGGED      5  // battery.present is explicitly "no" -- external power only

// beebo: build-flag overridable, like LORA_TX_POWER/ENV_INCLUDE_GPS -- a variant's
// platformio.ini can set -D BATT_SAMPLE_PERIOD_DEFAULT_SECS=<n> to change the
// out-of-box default; a runtime pref (0=default) overrides this per-node.
#ifndef BATT_SAMPLE_PERIOD_DEFAULT_SECS
  #define BATT_SAMPLE_PERIOD_DEFAULT_SECS 300  // 5 min
#endif
// beebo: how many seconds before the period deadline to start trying for a
// radioIsIdle() sample instead of always sampling right at the deadline (see
// MyMesh::updateBattTrend()) -- must be > IDLE_MARGIN or the window could
// close before an idle gap has any chance to appear.
#ifndef BATT_SAMPLE_WINDOW_DEFAULT_SECS
  #define BATT_SAMPLE_WINDOW_DEFAULT_SECS 30
#endif
#define BATT_TREND_HYSTERESIS_MV_10BIT 51  // 3 ADC LSB - 1 (~17.5mV/LSB at default 5.42x divider) -- rejects a -1/+1 LSB jitter round-trip
#define BATT_TREND_HYSTERESIS_MV_12BIT 12  // 12-bit LSB is 4x finer (~4.4mV) -- same 3*LSB-1 margin scaled down
// beebo: CHARGING only reports CHARGED once it actually reaches a real
// LiPo/Li-ion top-of-charge voltage -- otherwise a flat run mid-charge (a
// temporary current-limit plateau, IR-drop recovery, ADC jitter) would latch
// CHARGED at any voltage, however low. Distinct from the batt_present-based
// "plugged" check above: that one is about telling a real battery apart from
// no battery at all, where voltage alone can't work; this is a physical
// constant of the chemistry once we know a battery IS present, so a
// calibrated absolute threshold is meaningful here. Build-flag overridable
// only (-D BATT_FULL_MV=<mv>), no runtime override.
#ifndef BATT_FULL_MV
  #define BATT_FULL_MV 4200  // mV
#endif

// margin (ms) radioIsIdle() waits past the last RX/TX before calling the
// radio quiet -- keeps the periodic Vbat sample from landing mid RX/TX or
// too soon after, while an IR-drop sag is still recovering, and skewing
// the trend classifier. Build-flag overridable only (-D
// IDLE_MARGIN_MS=<ms>), no runtime override; companion_radio and
// simple_repeater each keep their own NodePrefs, so this constant lives
// here so the two can't drift.
#ifndef IDLE_MARGIN_MS
  #define IDLE_MARGIN_MS 100
#endif

// Classifies a new battery reading with an explicit switch(state) transition
// table -- state is the single source of truth the caller persists between
// calls, and PLUGGED is a state in that same table like any other, entered/
// left from within its own case. Only call this with an idle (IR-drop-clear)
// reading -- comparing a busy sample against a clean anchor would misread
// sag recovery as a real edge.
//
// ref_mv is the rolling comparison anchor, caller-owned like state. The
// caller is responsible for seeding it (to the first live reading) before
// ever calling this -- see MyMesh::updateBattTrend()'s first-sample check --
// so this function can always compute a meaningful dir against it.
//
// Purely threshold-driven: no consecutive-sample confirmation, hysteresis
// alone decides every transition. CHARGING and DISCHARGING are sticky: an
// in-band (dir==0) reading does not settle back to INIT -- a flat stretch
// mid-charge or mid-discharge is still charging/discharging, since nothing
// has actually reversed. Both track a peak (CHARGING) or trough
// (DISCHARGING) reference: ref_mv only moves further in the current
// direction, so the hysteresis reversal check is always against the most
// extreme point seen this run, not a stale entry-point anchor. The only way
// out of CHARGING without a real dir<0 reversal is crossing charged_mv,
// which reports CHARGED immediately (an absolute threshold, not an
// edge-settle judgement). DISCHARGING has no such absolute floor, so it
// only ever exits via a real dir>0 reversal back to CHARGING.
//
// CHARGED is likewise sticky in one direction only: once reached, a further
// rise just tracks a higher peak and stays CHARGED. A fall only exits to
// DISCHARGING once it drops back below charged_mv -- a dip that's still
// above the threshold is ripple near the top, not a real discharge, so it
// stays CHARGED. INIT gets the same charged_mv check as CHARGING, so
// booting (or returning from PLUGGED) already at/above the threshold
// reports CHARGED immediately instead of waiting on a dir>0 edge that will
// never come.
//
// beebo: the CHARGED exit is a bare absolute-threshold check
// (new_mv <= charged_mv), NOT gated on dir<0 like every other transition
// here -- deliberately, unlike CHARGING/DISCHARGING's own edge-confirmed
// exits. ref_mv is never updated while state stays CHARGED (there's
// nothing to track it against exit-wise), so it's still pinned at
// whatever peak first crossed charged_mv; on a slow multi-hour decline
// that peak-relative delta does eventually blow past the hysteresis
// margin same as any real edge would, but requiring that confirmation at
// all is the wrong test for CHARGED specifically -- unlike CHARGING/
// DISCHARGING, which track a moving peak/trough every sample and so
// always have a fresh dir to confirm against, CHARGED has no such
// reference to move, so gating its exit behind "one more hysteresis-sized
// single-step edge, happening to land in the sampling window that
// triggers this call" is an availability bug waiting to happen, not a
// real ripple-rejection guard (the threshold comparison alone already
// rejects ripple: a dip that's still above charged_mv fails the
// comparison and stays CHARGED regardless of dir). A confirmed-elsewhere
// case: a node observed stuck reporting CHARGED for 22+ hours while idle
// voltage fell ~150mV past charged_mv, sampled correctly and continuously
// throughout (see BUGS.md 2026-08-23).
//
// batt_present (NodePrefs.batt_present, see BATT_PRESENT_* above) replaces
// what used to be a fixed absolute-voltage "is this plugged in?" threshold.
// That can't work in general: once adc.multiplier is properly calibrated, a
// real LiPo's own max voltage (~4.2V) IS the old threshold, so a full
// battery and "no battery, running on the charger IC's no-load rail" become
// indistinguishable by voltage alone. Presence is instead an explicit
// persisted setting the operator sets once per board. Every non-PLUGGED case
// checks it first to enter PLUGGED; PLUGGED's own case checks the reverse.
inline uint8_t classifyBattTrend(
  uint16_t new_mv,
  uint16_t& ref_mv,
  uint8_t& state,
  uint8_t batt_present,
  uint8_t adc_resolution_bits,
  uint16_t charged_mv = BATT_FULL_MV
) {
  int32_t hysteresis_mv = (adc_resolution_bits == 12)
                           ? BATT_TREND_HYSTERESIS_MV_12BIT : BATT_TREND_HYSTERESIS_MV_10BIT;
  int32_t delta = (int32_t)new_mv - (int32_t)ref_mv;
  int8_t dir = (delta > hysteresis_mv) ? 1 : (delta < -hysteresis_mv) ? -1 : 0;

  switch (state) {
    case BATT_STATE_PLUGGED:
      if (batt_present != BATT_PRESENT_NO)
        state = BATT_STATE_INIT;
      break;

    case BATT_STATE_INIT:
      if (batt_present == BATT_PRESENT_NO) {
        state = BATT_STATE_PLUGGED;
      } else if (new_mv >= charged_mv) {
        ref_mv = new_mv;
        state = BATT_STATE_CHARGED;
      } else if (dir > 0) {
        ref_mv = new_mv;
        state = BATT_STATE_CHARGING;
      } else if (dir < 0) {
        ref_mv = new_mv;
        state = BATT_STATE_DISCHARGING;
      }
      break;

    case BATT_STATE_CHARGING:
      if (batt_present == BATT_PRESENT_NO) {
        state = BATT_STATE_PLUGGED;
      } else if (new_mv >= charged_mv) {
        ref_mv = new_mv;
        state = BATT_STATE_CHARGED;
      } else if (dir < 0) {
        ref_mv = new_mv;
        state = BATT_STATE_DISCHARGING;
      } else if (dir > 0) {
        ref_mv = new_mv;
      }
      break;

    case BATT_STATE_CHARGED:
      // Only exit (besides PLUGGED) is falling back at/below charged_mv --
      // a bare absolute-threshold check, not gated on dir<0 like every
      // other transition here (see the long comment above this function
      // for why: ref_mv is pinned at the entry peak and never moves while
      // staying CHARGED, so requiring a fresh hysteresis-sized edge against
      // that stale peak can indefinitely miss a real, slow decline that's
      // already well past the threshold).
      if (batt_present == BATT_PRESENT_NO) {
        state = BATT_STATE_PLUGGED;
      } else if (new_mv <= charged_mv) {
        ref_mv = new_mv;
        state = BATT_STATE_DISCHARGING;
      }
      break;

    case BATT_STATE_DISCHARGING:
      if (batt_present == BATT_PRESENT_NO) {
        state = BATT_STATE_PLUGGED;
      } else if (dir > 0) {
        ref_mv = new_mv;
        state = BATT_STATE_CHARGING;
      } else if (dir < 0) {
        ref_mv = new_mv;
      }
      break;
  }
  return state;
}

// (Re)initializes the trend state -- at boot from the just-loaded prefs, or
// later to reset it (e.g. an explicit ring clear or an LNA experiment
// restart). Picks PLUGGED vs INIT the same way classifyBattTrend() would on
// the first call, so boot doesn't have to duplicate that check. Does not
// touch ref_mv -- it already holds the last real ADC reading, which is still
// correct and doesn't need re-seeding.
inline void resetBattTrendRef(uint8_t& state, uint16_t& ref_mv, uint8_t batt_present) {
  state = (batt_present == BATT_PRESENT_NO) ? BATT_STATE_PLUGGED : BATT_STATE_INIT;
}
