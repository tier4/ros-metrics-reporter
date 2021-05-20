#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
PACKAGE_LIST=$(colcon list --names-only)
COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"

# Set generated timestamp
if [ $# -eq 3 ]; then
  TIMESTAMP=$(cat $3)
else
  TIMESTAMP=$(date -u '+%Y%m%d_%H%M%S')
fi

[ "$2" == "" ] && { echo "Please set output directory." ; exit 1; }
OUTPUT_DIR=${2}/lcov/${TIMESTAMP}

function get_package_coverage() {
  [ "$1" == "" ] && return 1

  if [[ $1 == *_msgs ]] ; then
    echo "Skipped $1"
    return 0
  fi

  # Build with correct flags
  colcon build --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" \
    -DCMAKE_C_FLAGS="$COVERAGE_FLAGS" \
    --packages-up-to $1 || { echo "Build failed." ; return 1; }

  if [ ! -d ${OUTPUT_DIR}/$1 ]; then
    mkdir -p ${OUTPUT_DIR}/$1
  fi

  # Get a zero-coverage baseline
  lcov \
    --config-file .lcovrc \
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
    --config-file .lcovrc \
    --base-directory "${BASE_DIR}" \
    --capture \
    --directory "${BASE_DIR}/build/$1" \
    --output-file "${OUTPUT_DIR}/$1/lcov.run" || { echo "Coverage generation failed."; return 1; }

  # Return if lcov.run is empty
  if [ ! -s ${OUTPUT_DIR}/$1/lcov.run ]; then
    echo "Skipped $1"
    return 0
  fi

  # Combine zero-coverage with coverage information.
  lcov \
    --config-file .lcovrc \
    -a "${OUTPUT_DIR}/$1/lcov.base" \
    -a "${OUTPUT_DIR}/$1/lcov.run" \
    -o "${OUTPUT_DIR}/$1/lcov.total" || { echo "Coverage combination failed."; return 1; }

  # Filter test, build, and install files and generate html
  lcov --config-file .lcovrc -r "${OUTPUT_DIR}/$1/lcov.total" \
    "${BASE_DIR}/build/*" \
    "${BASE_DIR}/install/*" \
          "${BASE_DIR}/$1/test/*" \
          "*/CMakeCCompilerId.c" \
          "*/CMakeCXXCompilerId.cpp" \
          "*_msgs/*" \
    -o "${OUTPUT_DIR}/$1/lcov.total.filtered" || { echo "Filtering failed."; return 1; }

  genhtml \
    --config-file .lcovrc \
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
  if [ $? -eq 1 ]; then
    exit 1
  fi
done
