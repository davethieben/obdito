#pragma once

// Bench self-test: core 1 bit-bangs sample VPW frames (3.3 V logic, not bus
// voltage) on `pin`. Jumper that pin to the RX pin to exercise the PIO capture
// and decoder without a truck. Every 4th response has a deliberately bad CRC.
namespace selftest {

void start(unsigned pin);

}  // namespace selftest
