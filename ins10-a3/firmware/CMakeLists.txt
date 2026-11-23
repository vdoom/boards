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
cmake_minimum_required(VERSION 3.13)

# INS10-A3 attitude and heading reference system for the Raspberry Pi Pico (RP2040)
# and Pico 2 (RP2350): app/, drivers/, lib/ins10/.
#
# The fusion library, the drivers and their tests also build on a PC: see
# tests/CMakeLists.txt.

set(PICO_BOARD pico CACHE STRING "Raspberry Pi Pico board")
if(NOT PICO_BOARD MATCHES "^(pico|pico2)$")
    message(FATAL_ERROR "Select PICO_BOARD=pico or PICO_BOARD=pico2.")
endif()
if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to the Pico SDK checkout (2.2.0 or newer).")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(ins10_ahrs C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_subdirectory(lib/ins10)

add_executable(ins10_ahrs
    app/main.c
    app/console.c
    app/storage.c
    drivers/bus.c
    drivers/icm42688.c
    drivers/mmc5983ma.c
    drivers/lps22df.c
)
target_include_directories(ins10_ahrs PRIVATE app drivers bsp)
target_compile_options(ins10_ahrs PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(ins10_ahrs PRIVATE
    # A host that stops reading must not stall the sensor loop: drop output instead.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000
    # The default 2 kB stack is tight for float formatting on top of the storage code
    # (two copies of the persistent record live on the stack while saving).
    PICO_STACK_SIZE=0x1000
)
target_link_libraries(ins10_ahrs
    ins10 pico_stdlib pico_flash pico_bootrom
    hardware_spi hardware_flash hardware_watchdog m
)
pico_enable_stdio_usb(ins10_ahrs 1)
pico_enable_stdio_uart(ins10_ahrs 0)
pico_set_program_name(ins10_ahrs "INS10 AHRS")
pico_set_program_version(ins10_ahrs "2.0.0")
pico_add_extra_outputs(ins10_ahrs)
