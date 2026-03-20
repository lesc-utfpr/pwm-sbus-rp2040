# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pwm_to_sbus_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 1. Added usb_descriptors.c to the executable list
add_executable(pwm_sbus_project
    main.c
    usb_descriptors.c
)

# 2. Disabled standard stdio over USB so TinyUSB can take full control of the USB port
pico_enable_stdio_usb(pwm_sbus_project 0)
pico_enable_stdio_uart(pwm_sbus_project 0)

# Generate PIO header
pico_generate_pio_header(pwm_sbus_project ${CMAKE_CURRENT_LIST_DIR}/pwm_in.pio)

# 3. Linked the TinyUSB device and board libraries
target_link_libraries(pwm_sbus_project
    pico_stdlib
    hardware_pio
    hardware_uart
    tinyusb_device
    tinyusb_board
)

# 4. Added include directory so the compiler can find tusb_config.h
target_include_directories(pwm_sbus_project PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(pwm_sbus_project)