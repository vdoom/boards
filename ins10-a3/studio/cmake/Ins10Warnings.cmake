# Strict warnings for the project's own code. Third-party headers (Qt, Eigen) come in
# as system includes and are not affected.
function(ins10_set_warnings target)
    if(MSVC)
        # C4702 (unreachable code) comes from the optimiser after inlining, so it is
        # reported inside Qt's own headers and /external:W0 cannot silence it there.
        target_compile_options(${target} PRIVATE /W4 /WX /utf-8 /permissive- /external:W0 /wd4702)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
endfunction()
