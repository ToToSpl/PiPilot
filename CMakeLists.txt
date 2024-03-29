cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# https://github.com/raspberrypi/pico-sdk must be downloaded in folder above
# also add PICO_SDK_PATH to your env 

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pi_pilot)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(vendors)

add_executable(
  pilot
  src/main.c
  src/madgwick.c
  src/crsf_pio_rx.c
  src/servo.c
)

pico_generate_pio_header(
  pilot
  ${CMAKE_CURRENT_LIST_DIR}/src/uart_rx.pio
)

# pull in common dependencies
target_link_libraries(
  pilot 
  pico_stdlib 
  pico_multicore
  hardware_pio
  hardware_i2c 
  hardware_pwm
  hardware_clocks
  haw_mpu6050
)

pico_enable_stdio_usb(pilot 1)
pico_enable_stdio_uart(pilot 0)

pico_add_extra_outputs(pilot)
