cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pwm_to_sbus_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pwm_sbus_project
    main.c
)

pico_enable_stdio_usb(pwm_sbus_project 1)
pico_enable_stdio_uart(pwm_sbus_project 0)

# Generate PIO header
pico_generate_pio_header(pwm_sbus_project ${CMAKE_CURRENT_LIST_DIR}/pwm_in.pio)

target_link_libraries(pwm_sbus_project
    pico_stdlib
    hardware_pio
    hardware_uart
)

pico_add_extra_outputs(pwm_sbus_project)