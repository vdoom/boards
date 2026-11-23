#!/usr/bin/env sh
# Builds and tests INS10 Studio on Linux or macOS.
# NOT TESTED on this machine: written for portability, checked only by review.
#
#   ./build.sh                       # Release build into ~/.cache/ins10-build/studio
#   QT_PREFIX=/opt/Qt/6.12.0/gcc_64 ./build.sh
#   ./build.sh test                  # build, then run the tests
#
# Needs CMake >= 3.21, a C++20 compiler and Qt >= 6.5 with SerialPort.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
build_dir=${INS10_BUILD_DIR:-"${XDG_CACHE_HOME:-$HOME/.cache}/ins10-build/studio"}
config=${CONFIG:-Release}

prefix_arg=""
if [ -n "${QT_PREFIX:-}" ]; then
    prefix_arg="-DCMAKE_PREFIX_PATH=$QT_PREFIX"
fi

generator=""
if command -v ninja >/dev/null 2>&1; then
    generator="-GNinja"
fi

# shellcheck disable=SC2086
cmake -S "$here" -B "$build_dir" $generator -DCMAKE_BUILD_TYPE="$config" $prefix_arg
cmake --build "$build_dir" --parallel
echo "Built: $build_dir/bin/ins10studio"

if [ "${1:-}" = "test" ]; then
    QT_QPA_PLATFORM=offscreen ctest --test-dir "$build_dir" --output-on-failure
fi
