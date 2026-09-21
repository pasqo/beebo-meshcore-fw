#pragma once

#include <stdint.h>

// beebo: per-block "reach" of the direct-neighbor table, for the on-device
// RX front-end trial (see TrialFSM.h). Every RX that names a neighbor bumps
// two counters on its slot: a lifetime `rx_count` (u16, exposed on beebo's
// neighbor paths) and a per-block `win_count` (u8) the trial resets each
// block. A neighbor counts as heard in a block only with at least MIN_HEARD
// packets, so a one-off noise decode -- which an LNA change can add -- is not
// counted as reach. Templates over the slot type (Beebo.h's NeighbourInfo, or
// a mock in the native tests) so the logic stays testable with no Arduino
// dependency. Constants are first picks.
namespace NeighborReach {

static constexpr uint8_t MIN_HEARD = 2;
// SNR x4 as stored in NeighbourInfo: below this (-5 dB) a neighbor is
// marginal -- close to the SF8 demodulation floor.
static constexpr int8_t MARGINAL_SNR_X4 = -20;

template <typename T>
inline void bump(T &nb) {
  if (nb.rx_count != 0xFFFF) nb.rx_count++;
  if (nb.win_count != 0xFF) nb.win_count++;
}

template <typename T>
inline void resetWindow(T *nb, int n) {
  for (int i = 0; i < n; i++) nb[i].win_count = 0;
}

template <typename T>
inline void scan(const T *nb, int n, uint8_t &heard, uint8_t &marginal) {
  uint32_t h = 0, m = 0;
  for (int i = 0; i < n; i++) {
    if (nb[i].heard_timestamp == 0 || nb[i].win_count < MIN_HEARD) continue;
    h++;
    if (nb[i].snr < MARGINAL_SNR_X4) m++;
  }
  heard = (uint8_t)(h > 255 ? 255 : h);
  marginal = (uint8_t)(m > 255 ? 255 : m);
}

}  // namespace NeighborReach
