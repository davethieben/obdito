#pragma once

#include <cstddef>
#include <cstdint>

// SAE J1850 VPW frame decoder. Pure C++ with no Pico dependencies, so it can
// be unit tested on a PC (see test/test_decoder.cpp).
//
// Feed it level periods in order. A frame looks like:
//
//   SOF (active ~200us) | bits... | EOD/EOF (passive >= ~200us)
//
// Bits alternate passive/active starting with passive right after SOF:
//   passive: short (64us) = 1, long (128us) = 0
//   active:  short (64us) = 0, long (128us) = 1
// Bytes are MSB first; the last byte is a CRC-8 (poly 0x1D, init 0xFF, inverted).
namespace vpw {

// Receiver timing windows in microseconds (1x speed). A period is classified
// by the window its duration falls into.
namespace timing {
constexpr uint32_t kNoiseMaxUs = 34;   // <= this: glitch
constexpr uint32_t kShortMaxUs = 96;   // short symbol, nominal 64
constexpr uint32_t kLongMaxUs = 163;   // long symbol, nominal 128
constexpr uint32_t kSofMaxUs = 239;    // active: SOF; passive: EOD (both nominal 200)
// Longer than kSofMaxUs: active = BREAK, passive = EOF (nominal >= 280).
}  // namespace timing

constexpr size_t kMaxFrameBytes = 16;  // spec max is 12 including CRC

struct Frame {
    uint64_t time_us = 0;  // when the SOF (or IFR normalization bit) ended
    uint8_t data[kMaxFrameBytes] = {};
    uint8_t len = 0;
    bool ifr = false;        // in-frame response (EOD + normalization bit + data)
    bool crc_ok = false;     // last byte == CRC-8 of the bytes before it
    bool truncated = false;  // ended mid-byte, or cut off by noise/SOF/BREAK
    bool overflow = false;   // more than kMaxFrameBytes arrived
};

struct Stats {
    uint32_t frames = 0;
    uint32_t crc_errors = 0;
    uint32_t framing_errors = 0;
    uint32_t noise = 0;
    uint32_t breaks = 0;
};

// SAE J1850 CRC-8.
uint8_t crc8(const uint8_t* data, size_t len);

class Decoder {
public:
    using FrameHandler = void (*)(const Frame& frame, void* ctx);

    Decoder(FrameHandler handler, void* ctx) : handler_(handler), ctx_(ctx) {}

    // One bus level period. `active` = bus was driven during it.
    // `duration_us` = its length (UINT32_MAX if it hit the capture timeout).
    // `end_us` = timestamp when it ended.
    void feed(bool active, uint32_t duration_us, uint64_t end_us);

    const Stats& stats() const { return stats_; }

private:
    enum class State : uint8_t { Idle, InFrame, AfterEod };

    void start(uint64_t time_us, bool ifr);
    void push_bit(bool bit);
    void finish(bool truncated);

    FrameHandler handler_;
    void* ctx_;
    State state_ = State::Idle;
    Frame frame_;
    uint8_t cur_ = 0;
    uint8_t nbits_ = 0;
    Stats stats_;
};

}  // namespace vpw
