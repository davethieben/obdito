cmake_minimum_required(VERSION 3.13)

# Locates the SDK via the PICO_SDK_PATH environment variable.
include(pico_sdk_import.cmake)

project(obdito C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---- Options --------------------------------------------------------------
set(OBDITO_RX_PIN 2 CACHE STRING "GPIO that receives the J1850 bus (after the level-shifting front end)")
option(OBDITO_RX_INVERT "Front end inverts the signal (bus active reads as LOW), e.g. an NPN transistor stage" OFF)
option(OBDITO_SELFTEST "Bit-bang test frames on OBDITO_SELFTEST_PIN from core 1 (jumper it to the RX pin)" OFF)
set(OBDITO_SELFTEST_PIN 3 CACHE STRING "GPIO that outputs self-test frames when OBDITO_SELFTEST is ON")

# ---- Firmware -------------------------------------------------------------
add_executable(obdito
    src/main.cpp
    src/vpw_rx.cpp
    src/vpw_decoder.cpp
    src/selftest_tx.cpp
)

pico_generate_pio_header(obdito ${CMAKE_CURRENT_LIST_DIR}/src/vpw_rx.pio)

target_compile_definitions(obdito PRIVATE
    OBDITO_RX_PIN=${OBDITO_RX_PIN}
    OBDITO_RX_INVERT=$<BOOL:${OBDITO_RX_INVERT}>
    OBDITO_SELFTEST=$<BOOL:${OBDITO_SELFTEST}>
    OBDITO_SELFTEST_PIN=${OBDITO_SELFTEST_PIN}
    # Give a serial terminal a moment to attach so the banner isn't lost.
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
)

target_compile_options(obdito PRIVATE -Wall -Wextra)

target_link_libraries(obdito PRIVATE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
)

# Log over USB CDC (and UART0 on GP0/GP1, handy with a debug probe).
pico_enable_stdio_usb(obdito 1)
pico_enable_stdio_uart(obdito 1)

pico_add_extra_outputs(obdito)
