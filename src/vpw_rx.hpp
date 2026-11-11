#pragma once

#include <cstdint>

// PIO-based capture of J1850 VPW level periods. A PIO state machine measures
// each active/passive period; an IRQ handler drains the PIO FIFO into a RAM
// ring buffer that the main loop reads with pop().
namespace vpw_rx {

// Periods longer than this are reported as timeouts (duration_us == kTimedOut).
constexpr uint32_t kTimeoutUs = 1000;
constexpr uint32_t kTimedOut = UINT32_MAX;

struct Pulse {
    uint64_t end_us;       // time (time_us_64) the period ended
    uint32_t duration_us;  // length of the period, or kTimedOut
    bool active;           // true = bus was active (~7 V) during the period
};

// Claims a PIO state machine and starts capturing on `pin`.
// `invert` = the front end inverts the signal (bus active reads as LOW).
bool init(unsigned pin, bool invert);

// Takes the oldest captured period. Returns false if none are waiting.
bool pop(Pulse& out);

// Periods lost because the ring buffer was full.
uint32_t dropped();

}  // namespace vpw_rx
