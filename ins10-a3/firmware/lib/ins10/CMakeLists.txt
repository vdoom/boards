# Portable sensor-fusion core. No platform dependencies: builds for the Pico and for
# the host test suite from the same sources.
add_library(ins10 STATIC
    src/calib.c
    src/ekf.c
    src/fusion.c
    src/math3.c
    src/motion.c
    src/persist.c
    src/thermal.c
)
target_include_directories(ins10 PUBLIC include)
set_target_properties(ins10 PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
if(MSVC)
    target_compile_options(ins10 PRIVATE /W4 /WX)
else()
    target_compile_options(ins10 PRIVATE -Wall -Wextra -Werror -Wdouble-promotion -Wshadow)
    target_link_libraries(ins10 PUBLIC m)
endif()
