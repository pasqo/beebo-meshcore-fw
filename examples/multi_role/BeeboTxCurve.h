#pragma once
// beebo: TX power at the antenna connector, estimated from radio.txpower.
// No Arduino dependencies, so it is unit-tested natively (test/test_txcurve).
//
// The built-in stage is the Heltec V4 TX chain (SX1262, pad, KCT8103L PA): the
// power measured at the SMA for each radio.txpower (docs/rf-experiments,
// "TX Radiated Power"; `beebo measure txpower`, CW, default PA table, +-1 dB).
// The external stage is board.ext.txgain, the net dB of whatever sits between
// the SMA and the antenna, measured at the operating point: the model is exact
// there and, since an external amplifier only compresses harder above it,
// over-estimates above it, so the cap built from it errs on the safe side.

#include <stdint.h>

#define BEEBO_TX_MIN_POWER  (-9)

// SMA output, tenths of dBm, for radio.txpower -9 .. 22
static const int16_t BEEBO_TX_SMA_X10[32] = {
    33,  38,  48,  63,  73,  83,  93,  98,   // -9 .. -2
   108, 118, 123, 133, 143, 168, 178, 183,   // -1 ..  6
   183, 193, 198, 223, 228, 233, 243, 253,   //  7 .. 14
   258, 263, 273, 273, 273, 273, 273, 273,   // 15 .. 22
};

// Power at the SMA for a radio.txpower, tenths of dBm; out of range clamps to the table.
inline int16_t beeboTxSmaX10(int tx_power) {
  int i = tx_power - BEEBO_TX_MIN_POWER;
  if (i < 0) i = 0;
  if (i > 31) i = 31;
  return BEEBO_TX_SMA_X10[i];
}

// Estimated power at the antenna connector, tenths of dBm.
inline int16_t beeboTxAntX10(int tx_power, int ext_txgain_db) {
  return (int16_t)(beeboTxSmaX10(tx_power) + ext_txgain_db * 10);
}

// The highest radio.txpower <= hw_max whose antenna power stays within limit_dbm. The
// limit always applies. Never below the minimum radio.txpower.
inline int beeboTxMaxPower(int hw_max, int ext_txgain_db, int limit_dbm) {
  int best = BEEBO_TX_MIN_POWER;
  for (int tp = BEEBO_TX_MIN_POWER; tp <= hw_max; tp++)
    if (beeboTxAntX10(tp, ext_txgain_db) <= limit_dbm * 10) best = tp;
  return best;
}
