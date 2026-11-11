#include "vpw_rx.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "vpw_rx.pio.h"

namespace vpw_rx {
namespace {

constexpr uint32_t kCountsPerSecond = 1'000'000;  // 1 count = 1 us
constexpr uint32_t kCyclesPerCount = 2;           // PIO counting loop length
// Between periods the PIO spends ~5 cycles (2.5 us) on in/in/push/jmp/mov
// before counting the new level. Add it back, rounded.
constexpr uint32_t kOverheadUs = 3;
constexpr uint32_t kTimeoutMarker = 0x7FFFFFFFu;

constexpr uint32_t kRingSize = 1024;  // must be a power of two
static_assert((kRingSize & (kRingSize - 1)) == 0, "ring size must be a power of two");

PIO g_pio = nullptr;
uint g_sm = 0;
Pulse g_ring[kRingSize];
volatile uint32_t g_head = 0;  // written by the IRQ handler only
volatile uint32_t g_tail = 0;  // written by pop() only
volatile uint32_t g_dropped = 0;

void __isr on_rx_fifo() {
    while (!pio_sm_is_rx_fifo_empty(g_pio, g_sm)) {
        const uint32_t word = pio_sm_get(g_pio, g_sm);
        const uint32_t x = word & kTimeoutMarker;

        Pulse p;
        p.end_us = time_us_64();
        p.active = (word >> 31) != 0;
        p.duration_us = (x == kTimeoutMarker) ? kTimedOut : (kTimeoutUs - x) + kOverheadUs;

        const uint32_t next = (g_head + 1) & (kRingSize - 1);
        if (next == g_tail) {
            g_dropped = g_dropped + 1;
            continue;
        }
        g_ring[g_head] = p;
        __compiler_memory_barrier();
        g_head = next;
    }
}

}  // namespace

bool init(unsigned pin, bool invert) {
    uint offset = 0;
    if (!pio_claim_free_sm_and_add_program(&vpw_rx_program, &g_pio, &g_sm, &offset)) {
        return false;
    }

    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    // Keep an unconnected input quiet (reads as "passive").
    if (invert) {
        gpio_pull_up(pin);
    } else {
        gpio_pull_down(pin);
    }
    // Inversion applies to everything reading the pin, including the PIO.
    gpio_set_inover(pin, invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

    pio_sm_config c = vpw_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, /*shift_right=*/false, /*autopush=*/false, 32);
    sm_config_set_clkdiv(&c, static_cast<float>(clock_get_hz(clk_sys)) /
                                 static_cast<float>(kCountsPerSecond * kCyclesPerCount));

    pio_sm_set_consecutive_pindirs(g_pio, g_sm, pin, 1, /*is_out=*/false);
    pio_sm_init(g_pio, g_sm, offset, &c);  // also clears the FIFOs

    // Preload Y = timeout and OSR = all ones (the "active" tag bit source).
    pio_sm_put(g_pio, g_sm, kTimeoutUs);
    pio_sm_exec(g_pio, g_sm, pio_encode_pull(/*if_empty=*/false, /*block=*/true));
    pio_sm_exec(g_pio, g_sm, pio_encode_mov(pio_y, pio_osr));
    pio_sm_exec(g_pio, g_sm, pio_encode_mov_not(pio_osr, pio_null));

    const uint irq_num = static_cast<uint>(pio_get_irq_num(g_pio, 0));
    irq_set_exclusive_handler(irq_num, on_rx_fifo);
    pio_set_irqn_source_enabled(g_pio, 0, pio_get_rx_fifo_not_empty_interrupt_source(g_sm), true);
    irq_set_enabled(irq_num, true);

    pio_sm_set_enabled(g_pio, g_sm, true);
    return true;
}

bool pop(Pulse& out) {
    const uint32_t tail = g_tail;
    if (tail == g_head) {
        return false;
    }
    __compiler_memory_barrier();
    out = g_ring[tail];
    __compiler_memory_barrier();
    g_tail = (tail + 1) & (kRingSize - 1);
    return true;
}

uint32_t dropped() { return g_dropped; }

}  // namespace vpw_rx
