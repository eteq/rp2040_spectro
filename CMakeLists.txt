cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(spectro_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD adafruit_feather_rp2040)

pico_sdk_init()

add_executable(spectro
  spectro.c
)

pico_enable_stdio_usb(spectro 1)
pico_enable_stdio_uart(spectro 1)

pico_add_extra_outputs(spectro)

target_link_libraries(spectro
                      pico_stdlib
                      hardware_adc
                      hardware_dma
                      hardware_i2c)