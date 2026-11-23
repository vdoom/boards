# INS10 Studio: desktop companion for the INS10-A3 board running the ins10_ahrs firmware.
#
#   cmake -S . -B <build dir> -DCMAKE_PREFIX_PATH=<Qt>/<kit>
#   cmake --build <build dir> --config Release
#   ctest --test-dir <build dir> -C Release --output-on-failure
#
# Keep the build directory outside the source tree (see build.ps1 / build.sh).
cmake_minimum_required(VERSION 3.21)
project(ins10studio VERSION 0.1.0 DESCRIPTION "INS10 Studio" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Qt6 6.5 REQUIRED COMPONENTS
    Core Gui Widgets OpenGL OpenGLWidgets SerialPort Concurrent Test LinguistTools)
qt_standard_project_setup(REQUIRES 6.5)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Ins10Eigen)
include(Ins10Warnings)

# Everything that runs lands in one folder, next to each other.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(src/core)
add_subdirectory(src/device)

# Optional: replaying logs through the firmware's fusion library. Needs the firmware
# sources next to this project (../firmware).
set(INS10_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../firmware"
    CACHE PATH "The ins10_ahrs firmware sources (for the log replay)")
if(EXISTS "${INS10_FIRMWARE_DIR}/lib/ins10/include/ins10/fusion.h")
    set(INS10_HAVE_REPLAY ON)
    enable_language(C) # lib/ins10 is C11; enabled here so that the tests can use it too
    add_subdirectory(src/replay)
    message(STATUS "Log replay through lib/ins10 from ${INS10_FIRMWARE_DIR}")
else()
    set(INS10_HAVE_REPLAY OFF)
    message(STATUS "Firmware sources not found: built without the log replay")
endif()

add_subdirectory(src/app)

option(INS10_BUILD_TESTS "Build the test suite" ON)
if(INS10_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
