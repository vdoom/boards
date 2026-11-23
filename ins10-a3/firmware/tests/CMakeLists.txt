# Host build: the fusion library, its test suite and the log replay utility.
# Stand-alone project; any C11 compiler works (MSVC, GCC, Clang).
#
#   cmake -S tests -B <build dir>
#   cmake --build <build dir> --config Release
#   ctest --test-dir <build dir> -C Release --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(ins10_host C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(../lib/ins10 ins10)

add_executable(ins10_tests test_main.c sim.c)
target_link_libraries(ins10_tests PRIVATE ins10)

add_executable(ins10_replay replay.c)
target_link_libraries(ins10_replay PRIVATE ins10)

# The sensor drivers, compiled unchanged against simulated devices: fake_pico/ stands in
# for the Pico SDK header and device_models.c for the SPI bus and the three chips.
add_executable(ins10_driver_tests
    test_drivers.c
    device_models.c
    ../drivers/icm42688.c
    ../drivers/mmc5983ma.c
    ../drivers/lps22df.c
)
target_include_directories(ins10_driver_tests PRIVATE fake_pico ../drivers)

if(MSVC)
    target_compile_options(ins10_tests PRIVATE /W3)
    target_compile_definitions(ins10_tests PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(ins10_replay PRIVATE /W3)
    target_compile_definitions(ins10_replay PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(ins10_driver_tests PRIVATE /W3)
else()
    target_compile_options(ins10_tests PRIVATE -Wall -Wextra)
    target_compile_options(ins10_replay PRIVATE -Wall -Wextra)
    target_compile_options(ins10_driver_tests PRIVATE -Wall -Wextra)
    target_link_libraries(ins10_driver_tests PRIVATE m)
endif()

enable_testing()
foreach(name math integrator calibration persist thermal_model motion_detector static
        tumble no_mag accel_disturbance mag_disturbance thermal_drift recovery saturation realistic slow_rotation
        declination_hemisphere start_attitudes live_calibration vibration robustness)
    add_test(NAME ${name} COMMAND ins10_tests ${name})
endforeach()
foreach(name icm_clock icm_decoding icm_faults mmc lps)
    add_test(NAME driver_${name} COMMAND ins10_driver_tests ${name})
endforeach()
