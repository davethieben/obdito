#include "vpw_decoder.hpp"

namespace vpw {

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return static_cast<uint8_t>(~crc);
}

void Decoder::feed(bool active, uint32_t d, uint64_t end_us) {
    using namespace timing;

    if (d <= kNoiseMaxUs) {
        // A glitch breaks the level alternation, so the rest of the frame
        // can't be trusted. Report what we have and resync on the next SOF.
        ++stats_.noise;
        if (state_ == State::InFrame) {
            finish(true);
        }
        state_ = State::Idle;
        return;
    }

    if (active) {
        if (d > kSofMaxUs) {  // BREAK
            ++stats_.breaks;
            if (state_ == State::InFrame) {
                finish(true);
            }
            state_ = State::Idle;
        } else if (d > kLongMaxUs) {  // SOF
            if (state_ == State::InFrame) {
                finish(true);
            }
            start(end_us, false);
        } else if (state_ == State::InFrame) {
            push_bit(d > kShortMaxUs);  // active: long = 1, short = 0
        } else if (state_ == State::AfterEod) {
            start(end_us, true);  // normalization bit, then an IFR follows
        }
        // Idle: stray pulse (e.g. we started listening mid-frame). Ignore.
        return;
    }

    // Passive period.
    if (state_ != State::InFrame) {
        if (d > kSofMaxUs) {
            state_ = State::Idle;
        }
        return;
    }
    if (d > kSofMaxUs) {  // EOF
        finish(false);
        state_ = State::Idle;
    } else if (d > kLongMaxUs) {  // EOD: an IFR may follow
        finish(false);
        state_ = State::AfterEod;
    } else {
        push_bit(d <= kShortMaxUs);  // passive: short = 1, long = 0
    }
}

void Decoder::start(uint64_t time_us, bool ifr) {
    frame_ = Frame{};
    frame_.time_us = time_us;
    frame_.ifr = ifr;
    cur_ = 0;
    nbits_ = 0;
    state_ = State::InFrame;
}

void Decoder::push_bit(bool bit) {
    cur_ = static_cast<uint8_t>((cur_ << 1) | (bit ? 1 : 0));
    if (++nbits_ == 8) {
        if (frame_.len < kMaxFrameBytes) {
            frame_.data[frame_.len++] = cur_;
        } else {
            frame_.overflow = true;
        }
        cur_ = 0;
        nbits_ = 0;
    }
}

void Decoder::finish(bool truncated) {
    if (frame_.len == 0 && nbits_ == 0) {
        return;  // nothing received (e.g. SOF followed by noise)
    }
    frame_.truncated = truncated || nbits_ != 0;
    frame_.crc_ok = frame_.len >= 2 &&
                    crc8(frame_.data, frame_.len - 1u) == frame_.data[frame_.len - 1u];

    ++stats_.frames;
    if (frame_.truncated || frame_.overflow) {
        ++stats_.framing_errors;
    } else if (!frame_.crc_ok) {
        ++stats_.crc_errors;
    }
    if (handler_ != nullptr) {
        handler_(frame_, ctx_);
    }
}

}  // namespace vpw
