# ins10replay: logs run through the firmware's fusion library on the PC.
#
# The library is compiled from the firmware's own sources (read only), with the
# firmware's own CMakeLists and warning flags; nothing is copied.
add_subdirectory(${INS10_FIRMWARE_DIR}/lib/ins10 ${CMAKE_CURRENT_BINARY_DIR}/lib-ins10)

add_library(ins10replay STATIC replay.cpp replay.h)
target_include_directories(ins10replay PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(ins10replay PUBLIC Qt6::Core PRIVATE ins10)
ins10_set_warnings(ins10replay)
