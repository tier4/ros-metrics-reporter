#!/bin/bash

set -e

PACKAGE_LIST=$(colcon list --names-only)
PACKAGE_LIST_FULL=$(colcon list)
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

TIMESTAMP=$(cat $7)

# Parse exclude path
args=("$@")
argn=$#

exclude=()
for ((i=5; i <= argn; i++)); do
  exclude+=( "${args[$i-1]}" )
done

[ "$2" == "" ] && { echo "Please set output directory." ; exit 1; }
OUTPUT_DIR=${2}/lizard_result/${TIMESTAMP}

function get_package_path() {
  [ "$1" == "" ] && return 1

  echo "$PACKAGE_LIST_FULL" | grep -w "$1" | awk '{ print $2 }'
  return 0
}

function exec_lizard() {
  [ "$1" == "" ] && return 1

  PACKAGE_PATH=$(get_package_path "$1")
  if python "$SCRIPT_DIR"/path_match.py "$PACKAGE_PATH" "${exclude[@]}" 2>&1 >/dev/null ; then
    echo "Match exclude path. Skipped $1"
    return 0
  fi

  if [ ! -d ${OUTPUT_DIR}/$1 ]; then
    mkdir -p ${OUTPUT_DIR}/$1
  fi

  python3 ${SCRIPT_DIR}/lizard/lizard.py \
    -l cpp \
    -l python \
    -x "*test*" \
    -x "*lizard*" \
    --CCN $2 \
    -T nloc=$3 \
    --arguments $4 \
    --html $PACKAGE_PATH > ${OUTPUT_DIR}/$1/index.html || true
}

if [ ! -d $SCRIPT_DIR/lizard ]; then
  git clone https://github.com/terryyin/lizard.git $SCRIPT_DIR/lizard
fi

for PACKAGE in $PACKAGE_LIST; do
  exec_lizard $PACKAGE $4 $5 $6
done
