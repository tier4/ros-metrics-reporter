#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"

TIMESTAMP=$(cat $3)
LCOVRC=$4

[ "$2" == "" ] && { echo "Please set output directory." ; exit 1; }
OUTPUT_DIR=${2}/lcov_result/${TIMESTAMP}/all

function get_package_coverage() {
  # Build with correct flags
  colcon build --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" \
    -DCMAKE_C_FLAGS="$COVERAGE_FLAGS" || { echo "Build failed." ; return 1; }

  # Get a zero-coverage baseline
  lcov \
    --config-file "${LCOVRC}" \
    --base-directory "${BASE_DIR}" \
    --capture \
    --directory "${BASE_DIR}/build" \
    -o "${OUTPUT_DIR}/lcov.base" \
    --initial || { echo "Zero baseline coverage failed."; return 1; }

  colcon test \
    --event-handlers console_cohesion+ \
    --return-code-on-test-failure || { echo "Unit/integration testing failed."; return 1; }

  # Get coverage
  lcov \
    --config-file "${LCOVRC}" \
    --base-directory "${BASE_DIR}" \
    --capture \
    --directory "${BASE_DIR}/build" \
    --output-file "${OUTPUT_DIR}/lcov.run" || { echo "Coverage generation failed."; return 1; }

  # Return if lcov.run is empty
  if [ ! -s ${OUTPUT_DIR}/lcov.run ]; then
    echo "Skipped"
    return 0
  fi

  # Combine zero-coverage with coverage information.
  lcov \
    --config-file "${LCOVRC}" \
    -a "${OUTPUT_DIR}/lcov.base" \
    -a "${OUTPUT_DIR}/lcov.run" \
    -o "${OUTPUT_DIR}/lcov.total" || { echo "Coverage combination failed."; return 1; }

  # Filter test, build, and install files and generate html
  lcov --config-file "${LCOVRC}" -r "${OUTPUT_DIR}/lcov.total" \
    "${BASE_DIR}/build/*" \
    "${BASE_DIR}/install/*" \
          "*/test/*" \
          "*/CMakeCCompilerId.c" \
          "*/CMakeCXXCompilerId.cpp" \
          "*_msgs/*" \
          "*/usr/*" \
          "*/opt/*" \
    -o "${OUTPUT_DIR}/lcov.total.filtered" || { echo "Filtering failed."; return 1; }

  genhtml \
    --config-file "${LCOVRC}" \
    -p "${BASE_DIR}" \
    --legend \
    --demangle-cpp \
    "${OUTPUT_DIR}/lcov.total.filtered" \
    -o "${OUTPUT_DIR}/" || { echo "HTML generation failed."; return 1; }

  return 0
}

# Create output directory
mkdir -p $OUTPUT_DIR

get_package_coverage
