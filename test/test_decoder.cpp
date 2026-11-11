// Host-side tests for the VPW decoder (no Pico needed).
//   g++ -std=c++17 -Wall -Wextra -I../src test_decoder.cpp ../src/vpw_decoder.cpp -o test_decoder && ./test_decoder

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "vpw_decoder.hpp"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

struct Period {
    bool active;
    uint32_t us;
};

// Encode bytes (+ CRC) into level periods, with optional timing skew.
std::vector<Period> encode(const std::vector<uint8_t>& bytes, bool add_crc = true, int skew = 0) {
    std::vector<uint8_t> all = bytes;
    if (add_crc) {
        all.push_back(vpw::crc8(bytes.data(), bytes.size()));
    }
    std::vector<Period> out{{true, static_cast<uint32_t>(200 + skew)}};
    bool active = false;
    for (uint8_t b : all) {
        for (int i = 7; i >= 0; --i) {
            const bool bit = (b >> i) & 1;
            out.push_back({active, static_cast<uint32_t>((bit != active ? 64 : 128) + skew)});
            active = !active;
        }
    }
    out.push_back({false, UINT32_MAX});  // EOF / idle timeout
    return out;
}

std::vector<vpw::Frame> g_frames;
void collect(const vpw::Frame& f, void*) { g_frames.push_back(f); }

std::vector<vpw::Frame> run(const std::vector<Period>& periods) {
    g_frames.clear();
    vpw::Decoder dec(collect, nullptr);
    uint64_t t = 0;
    for (const Period& p : periods) {
        t += (p.us == UINT32_MAX) ? 1000 : p.us;
        dec.feed(p.active, p.us, t);
    }
    return g_frames;
}

void test_crc() {
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(vpw::crc8(check, sizeof check) == 0x4B);  // SAE J1850 check value
    const uint8_t req[] = {0x68, 0x6A, 0xF1, 0x01, 0x00};
    CHECK(vpw::crc8(req, sizeof req) == 0x17);  // well-known "68 6A F1 01 00 17"
}

void test_roundtrip(int skew) {
    const std::vector<uint8_t> msg = {0x48, 0x6B, 0x10, 0x41, 0x0C, 0x1A, 0xF8};
    auto frames = run(encode(msg, true, skew));
    CHECK(frames.size() == 1);
    if (frames.size() != 1) return;
    const vpw::Frame& f = frames[0];
    CHECK(f.len == msg.size() + 1);
    for (size_t i = 0; i < msg.size(); ++i) CHECK(f.data[i] == msg[i]);
    CHECK(f.crc_ok);
    CHECK(!f.truncated);
    CHECK(!f.ifr);
}

void test_bad_crc() {
    auto p = encode({0x68, 0x6A, 0xF1, 0x01, 0x0C, 0x00}, false);  // 0x00 is not the CRC
    auto frames = run(p);
    CHECK(frames.size() == 1);
    if (!frames.empty()) CHECK(!frames[0].crc_ok);
}

void test_back_to_back_and_garbage_prefix() {
    std::vector<Period> p = {{true, 64}, {false, 128}, {true, 64}};  // mid-frame junk
    p.push_back({false, 400});
    auto a = encode({0x68, 0x6A, 0xF1, 0x01, 0x0C});
    auto b = encode({0x48, 0x6B, 0x10, 0x41, 0x0C, 0x0B, 0xB8});
    a.back() = {false, 300};  // EOF via long passive rather than timeout
    p.insert(p.end(), a.begin(), a.end());
    p.insert(p.end(), b.begin(), b.end());
    auto frames = run(p);
    CHECK(frames.size() == 2);
    if (frames.size() == 2) {
        CHECK(frames[0].len == 6 && frames[0].crc_ok && frames[0].data[4] == 0x0C);
        CHECK(frames[1].len == 8 && frames[1].crc_ok && frames[1].data[6] == 0xB8);
    }
}

void test_noise_truncates() {
    auto p = encode({0x24, 0x1A, 0x00});
    p.insert(p.begin() + 5, {false, 10});  // glitch partway through the first byte
    auto frames = run(p);
    CHECK(frames.size() == 1);
    if (!frames.empty()) CHECK(frames[0].truncated);
}

void test_ifr() {
    auto p = encode({0x68, 0x6A, 0xF1});
    p.back() = {false, 200};          // EOD instead of EOF
    p.push_back({true, 64});          // normalization bit
    auto ifr = encode({0x55}, true);  // reuse bit encoding, drop its SOF
    p.insert(p.end(), ifr.begin() + 1, ifr.end());
    auto frames = run(p);
    CHECK(frames.size() == 2);
    if (frames.size() == 2) {
        CHECK(!frames[0].ifr && frames[0].crc_ok);
        CHECK(frames[1].ifr && frames[1].len == 2 && frames[1].data[0] == 0x55);
    }
}

}  // namespace

int main() {
    test_crc();
    for (int skew : {-25, -10, 0, 3, 10, 25}) test_roundtrip(skew);
    test_bad_crc();
    test_back_to_back_and_garbage_prefix();
    test_noise_truncates();
    test_ifr();
    if (g_failures == 0) std::printf("all decoder tests passed\n");
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
