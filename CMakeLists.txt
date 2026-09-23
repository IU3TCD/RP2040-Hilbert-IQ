# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rp2040hilbert C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rp2040hilbert rp2040hilbert.cpp )

pico_set_program_name(rp2040hilbert "rp2040hilbert")
pico_set_program_version(rp2040hilbert "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rp2040hilbert 0)
pico_enable_stdio_usb(rp2040hilbert 1)

# Add the standard library to the build
target_link_libraries(rp2040hilbert
        pico_stdlib
        pico_multicore
        hardware_adc
        hardware_pwm
        hardware_pio
        hardware_clocks
        hardware_vreg
        hardware_i2c
        pico_i2c_slave)

# Add the standard include files to the build
target_include_directories(rp2040hilbert PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_generate_pio_header(rp2040hilbert ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

pico_add_extra_outputs(rp2040hilbert)

