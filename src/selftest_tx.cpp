#include "selftest_tx.hpp"

#include <cstddef>
#include <cstdint>

#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "vpw_decoder.hpp"

namespace selftest {
namespace {

unsigned g_pin = 0;

void hold(bool active, uint32_t us) {
    gpio_put(g_pin, active);
    busy_wait_us_32(us);
}

void send(const uint8_t* data, size_t len, bool corrupt_crc) {
    bool active = false;  // first bit after SOF is passive
    auto put_byte = [&](uint8_t b) {
        for (int i = 7; i >= 0; --i) {
            const bool bit = (b >> i) & 1;
            // passive: 1 = short, 0 = long; active: 1 = long, 0 = short
            hold(active, bit != active ? 64 : 128);
            active = !active;
        }
    };

    hold(true, 200);  // SOF
    for (size_t i = 0; i < len; ++i) {
        put_byte(data[i]);
    }
    uint8_t crc = vpw::crc8(data, len);
    if (corrupt_crc) {
        crc ^= 0x01;
    }
    put_byte(crc);
    hold(false, 300);  // EOF + inter-frame separation
}

void core1_main() {
    // OBD-II mode 01 PID 0C (engine RPM) request and a response of 0x1AF8 / 4 = 1726 rpm.
    static const uint8_t kRequest[] = {0x68, 0x6A, 0xF1, 0x01, 0x0C};
    static const uint8_t kResponse[] = {0x48, 0x6B, 0x10, 0x41, 0x0C, 0x1A, 0xF8};

    for (uint32_t n = 0;; ++n) {
        send(kRequest, sizeof kRequest, false);
        sleep_ms(5);
        send(kResponse, sizeof kResponse, (n % 4) == 3);
        sleep_ms(1000);
    }
}

}  // namespace

void start(unsigned pin) {
    g_pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
    multicore_launch_core1(core1_main);
}

}  // namespace selftest
