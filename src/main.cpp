// obdito - a little J1850 VPW (OBD-II) bus sniffer for the Raspberry Pi Pico.
//
// Captures bus level periods with PIO, decodes them into frames, and logs the
// frames to stdio (USB CDC and UART0).
//
// Serial commands:  r = toggle raw pulse output   s = stats   h = help

#include <cstdio>

#include "pico/stdlib.h"
#include "selftest_tx.hpp"
#include "vpw_decoder.hpp"
#include "vpw_rx.hpp"

#ifndef OBDITO_RX_PIN
#define OBDITO_RX_PIN 2
#endif
#ifndef OBDITO_RX_INVERT
#define OBDITO_RX_INVERT 0
#endif
#ifndef OBDITO_SELFTEST
#define OBDITO_SELFTEST 0
#endif
#ifndef OBDITO_SELFTEST_PIN
#define OBDITO_SELFTEST_PIN 3
#endif

namespace {

bool g_raw = false;
bool g_raw_line_open = false;

void print_timestamp(uint64_t t_us) {
    printf("[%7lu.%03lu] ", static_cast<unsigned long>(t_us / 1000000),
           static_cast<unsigned long>((t_us / 1000) % 1000));
}

void led_toggle() {
#ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
#endif
}

void on_frame(const vpw::Frame& f, void* /*ctx*/) {
    if (g_raw_line_open) {
        printf("\n");
        g_raw_line_open = false;
    }
    print_timestamp(f.time_us);
    if (f.ifr) {
        printf("IFR ");
    }
    for (uint8_t i = 0; i < f.len; ++i) {
        printf("%02X ", f.data[i]);
    }
    if (f.truncated) {
        printf(" <truncated>");
    } else if (f.overflow) {
        printf(" <overflow>");
    } else if (!f.crc_ok) {
        printf(" <bad crc>");
    }
    printf("\n");
    led_toggle();
}

void print_raw(const vpw_rx::Pulse& p) {
    const char level = p.active ? 'A' : 'P';
    if (p.duration_us == vpw_rx::kTimedOut) {
        printf("%c>%lu\n", level, static_cast<unsigned long>(vpw_rx::kTimeoutUs));
        g_raw_line_open = false;
    } else {
        printf("%c%lu ", level, static_cast<unsigned long>(p.duration_us));
        g_raw_line_open = true;
    }
}

void print_stats(const vpw::Decoder& dec) {
    const vpw::Stats& s = dec.stats();
    printf("frames=%lu crc_err=%lu framing_err=%lu noise=%lu breaks=%lu dropped=%lu\n",
           static_cast<unsigned long>(s.frames), static_cast<unsigned long>(s.crc_errors),
           static_cast<unsigned long>(s.framing_errors), static_cast<unsigned long>(s.noise),
           static_cast<unsigned long>(s.breaks), static_cast<unsigned long>(vpw_rx::dropped()));
}

void print_help() {
    printf("commands: r = toggle raw pulses (A=active, P=passive, in us)  s = stats  h = help\n");
}

void handle_command(int c, const vpw::Decoder& dec) {
    switch (c) {
        case 'r':
        case 'R':
            g_raw = !g_raw;
            printf("\nraw pulse output %s\n", g_raw ? "on" : "off");
            g_raw_line_open = false;
            break;
        case 's':
        case 'S':
            print_stats(dec);
            break;
        case 'h':
        case 'H':
        case '?':
            print_help();
            break;
        default:
            break;
    }
}

}  // namespace

int main() {
    stdio_init_all();

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    printf("\nobdito - J1850 VPW sniffer\n");
    printf("rx pin GP%d%s\n", OBDITO_RX_PIN, OBDITO_RX_INVERT ? " (inverted)" : "");

    if (!vpw_rx::init(OBDITO_RX_PIN, OBDITO_RX_INVERT != 0)) {
        printf("error: no free PIO state machine\n");
        for (;;) {
            tight_loop_contents();
        }
    }

#if OBDITO_SELFTEST
    printf("self-test frames on GP%d (jumper it to GP%d)\n", OBDITO_SELFTEST_PIN, OBDITO_RX_PIN);
    selftest::start(OBDITO_SELFTEST_PIN);
#endif

    print_help();

    vpw::Decoder decoder(on_frame, nullptr);

    for (;;) {
        vpw_rx::Pulse p;
        while (vpw_rx::pop(p)) {
            if (g_raw) {
                print_raw(p);
            }
            decoder.feed(p.active, p.duration_us, p.end_us);
        }

        const int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            handle_command(c, decoder);
        }
    }
}
