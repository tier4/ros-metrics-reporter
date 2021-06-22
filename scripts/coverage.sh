#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
PACKAGE_LIST=$(colcon list --names-only)
PACKAGE_LIST_FULL=$(colcon list)
COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1 -O0"
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

TIMESTAMP=$(cat "$3")
LCOVRC=$4

# Parse exclude path
args=("$@")
argn=$#

exclude=()
for ((i=5; i <= argn; i++)); do
  exclude+=( "${args[$i-1]}" )
done

[ "$2" == "" ] && { echo "Please set output directory." ; exit 1; }
OUTPUT_DIR=${2}/lcov_result/${TIMESTAMP}

function get_package_path() {
  [ "$1" == "" ] && return 1

  echo "$PACKAGE_LIST_FULL" | grep -w "$1" | awk '{ print $2 }'
  return 0
}

function get_package_coverage() {
  [ "$1" == "" ] && return 1

  if [[ $1 == *_msgs ]]; then
    echo "Skipped $1"
    return 0
  fi

  PACKAGE_PATH=$(get_package_path "$1")"/"
  RET=$(python "$SCRIPT_DIR"/path_match.py "$PACKAGE_PATH" "${exclude[@]}")
  if [ "$RET" -eq "0" ] ; then
    echo "Match exclude path. Skipped $1"
    return 0
  fi

  # Build with correct flags
  colcon build --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" \
    -DCMAKE_C_FLAGS="$COVERAGE_FLAGS" \
    --packages-up-to $1 || { echo "Build failed." ; return 1; }

  if [ ! -d ${OUTPUT_DIR}/$1 ]; then
    mkdir -p ${OUTPUT_DIR}/$1
  fi

  # Get a zero-coverage baseline
  lcov \
    --config-file "${LCOVRC}" \
    --base-directory "${BASE_DIR}" \
    --capture \
    --directory "${BASE_DIR}/build/$1" \
    -o "${OUTPUT_DIR}/$1/lcov.base" \
    --initial || { echo "Zero baseline coverage failed."; return 1; }

  colcon test \
    --event-handlers console_cohesion+ \
    --packages-select $1 \
    --return-code-on-test-failure || { echo "Unit/integration testing failed."; return 1; }

  # Get coverage
  lcov \
    --config-file "${LCOVRC}" \
    --base-directory "${BASE_DIR}" \
    --capture \
    --directory "${BASE_DIR}/build/$1" \
    --output-file "${OUTPUT_DIR}/$1/lcov.run" || { echo "Coverage generation failed."; return 1; }

  # Return if lcov.run is empty
  if [ ! -s ${OUTPUT_DIR}/$1/lcov.run ]; then
    echo "lcov.run is empty. Skipped $1"
    return 0
  fi

  # Combine zero-coverage with coverage information.
  lcov \
    --config-file "${LCOVRC}" \
    -a "${OUTPUT_DIR}/$1/lcov.base" \
    -a "${OUTPUT_DIR}/$1/lcov.run" \
    -o "${OUTPUT_DIR}/$1/lcov.total" || { echo "Coverage combination failed."; return 1; }

  # Filter test, build, and install files and generate html
  lcov --config-file "${LCOVRC}" -r "${OUTPUT_DIR}/$1/lcov.total" \
    "${BASE_DIR}/build/*" \
    "${BASE_DIR}/install/*" \
          "*/test/*" \
          "*/CMakeCCompilerId.c" \
          "*/CMakeCXXCompilerId.cpp" \
          "*_msgs/*" \
          "*/usr/*" \
          "*/opt/*" \
    -o "${OUTPUT_DIR}/$1/lcov.total.filtered" || { echo "Filtering failed."; return 1; }

  genhtml \
    --config-file "${LCOVRC}" \
    -p "${BASE_DIR}" \
    --legend \
    --demangle-cpp \
    "${OUTPUT_DIR}/$1/lcov.total.filtered" \
    -o "${OUTPUT_DIR}/$1/" || { echo "HTML generation failed."; return 1; }

  return 0
}

# Create output directory
mkdir -p $OUTPUT_DIR

# Save timestamp
echo $TIMESTAMP > ${OUTPUT_DIR}/timestamp.txt

# Save package list
echo "$PACKAGE_LIST" > ${OUTPUT_DIR}/package_list.txt

for PACKAGE in $PACKAGE_LIST; do
  get_package_coverage $PACKAGE
done
