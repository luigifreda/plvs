#!/usr/bin/env bash
# Configure, build, and run the focused line-mapping regression tests.
# Requires the project dependencies and an existing CMake build directory.

set -euo pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
SCRIPT_DIR=$(readlink -f "$SCRIPT_DIR")
ROOT_DIR=$(readlink -f "$SCRIPT_DIR/..")

# shellcheck source=../bash_utils.sh
source "$ROOT_DIR/bash_utils.sh"

BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build}"
JOBS="${JOBS:-2}"

if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    print_red "No configured CMake build at $BUILD_DIR"
    print_red "Configure PLVS first (./build_plvs.sh), then rerun this script."
    exit 1
fi

cd "$ROOT_DIR"

print_blue '================================================'
print_blue 'Configuring line mapping tests'
print_blue '================================================'
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DPLVS_BUILD_TESTS=ON "$@"

print_blue '================================================'
print_blue 'Building test_line_mapping'
print_blue '================================================'
cmake --build "$BUILD_DIR" --target test_line_mapping -j"$JOBS"

print_blue '================================================'
print_blue 'Running line_* tests'
print_blue '================================================'
ctest --test-dir "$BUILD_DIR" --output-on-failure -R '^line_'
