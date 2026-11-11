# obdito

A little J1850 VPW (OBD-II) bus sniffer for the Raspberry Pi Pico. Built for a
2005 Dodge Ram 1500 (Chrysler PCI bus = J1850 VPW on OBD pin 2).

A PIO state machine measures how long each bus level lasts (1 us resolution);
an IRQ moves those measurements into a ring buffer; the main loop decodes VPW
symbols into frames, checks the CRC, and logs to stdio (USB serial + UART0).

```
obdito - J1850 VPW sniffer
rx pin GP2
[     12.345] 68 6A F1 01 0C 8B
[     12.351] 48 6B 10 41 0C 1A F8 B2
[     15.372] 48 6B 10 41 0C 1A F8 B3  <bad crc>
```

## Build

Requires `PICO_SDK_PATH` to point at the pico-sdk (2.x) and an `arm-none-eabi` toolchain.

```
cmake -S . -B build -G Ninja
cmake --build build
```

Copy `build/obdito.uf2` to the Pico in BOOTSEL mode, then open the USB serial port (any baud rate).

| CMake option | Default | Meaning |
|---|---|---|
| `PICO_BOARD` | `pico` | Use `pico2` for a Pico 2 |
| `OBDITO_RX_PIN` | `2` | GPIO fed by the front end |
| `OBDITO_RX_INVERT` | `OFF` | `ON` if the front end inverts (e.g. NPN stage) |
| `OBDITO_SELFTEST` | `OFF` | Bit-bang test frames from core 1 on `OBDITO_SELFTEST_PIN` |
| `OBDITO_SELFTEST_PIN` | `3` | Jumper this to the RX pin for bench testing |

Serial commands: `r` toggle raw pulse output (`A200 P128 A64 ...` in us), `s` stats, `h` help.

## Bench test (no truck)

```
cmake -S . -B build -G Ninja -DOBDITO_SELFTEST=ON
```

Jumper GP3 to GP2. You should see an RPM request/response pair every second, with every 4th response flagged `<bad crc>` on purpose.

## Wiring to the truck (receive only)

The bus idles at 0 V and is driven to ~7 V. The RP2040's GPIOs are 3.3 V only, so don't connect pin 2 directly.

```
OBD pin 2 (J1850) ──[10k]──┬──────────── GP2
                           │      BAT85 (cathode to 3V3) clamps faults
                         [6.8k]   GP2 ──|>|── 3V3
                           │
OBD pin 5 (signal GND) ────┴──────────── GND
```

7 V on the bus gives ~2.6 V at GP2 (the internal pull-down is in parallel), comfortably above the 2.0 V logic-high threshold.

- Power the Pico from the laptop's USB. You don't need OBD pin 16 (+12 V) for listening. Running the laptop on battery avoids ground loops.
- This front end only listens and never drives the bus, so it can't disturb the truck.
- Alternative: an NPN stage (bus → 22k → base, 10k base → GND, collector → 10k pull-up to 3V3 → GP2, emitter → GND) gives a cleaner threshold but inverts the signal, so build with `-DOBDITO_RX_INVERT=ON`.

## Decoder tests (PC)

```
cd test
g++ -std=c++17 -Wall -Wextra -I../src test_decoder.cpp ../src/vpw_decoder.cpp -o test_decoder && ./test_decoder
```

## VPW cheat sheet

| Symbol | Nominal | Accepted window |
|---|---|---|
| Short | 64 us | 35–96 us |
| Long | 128 us | 97–163 us |
| SOF (active) / EOD (passive) | 200 us | 164–239 us |
| EOF (passive) / BREAK (active) | ≥ 280 us | > 239 us |

Passive short = 1, passive long = 0, active short = 0, active long = 1. Bytes are MSB first, and the last byte is a CRC-8 (poly 0x1D, init 0xFF, inverted).
