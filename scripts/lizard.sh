#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
PACKAGE_LIST=$(colcon list --names-only)
PACKAGE_LIST_FULL=$(colcon list)

# Set generated timestamp
if [ -e $2 ]; then
  TIMESTAMP=$(cat $2)
else
  TIMESTAMP=$(date -u '+%Y%m%d_%H%M%S')
fi
OUTPUT_DIR=${BASE_DIR}/lizard_result/${TIMESTAMP}

function get_package_path() {
  [ "$1" == "" ] && return 1

  echo "$PACKAGE_LIST_FULL" | grep -w "$1" | awk '{ print $2 }'
  return 0
}

function exec_lizard() {
  [ "$1" == "" ] && return 1

  if [ ! -d ${OUTPUT_DIR}/$1 ]; then
    mkdir -p ${OUTPUT_DIR}/$1
  fi

  PACKAGE_PATH=$(get_package_path "$1")

  python3 lizard/lizard.py -l cpp -l python -x "*test*" -x "*lizard*" \
    --CCN 20 -T nloc=200 --arguments 6 \
    --html $PACKAGE_PATH > ${OUTPUT_DIR}/$1/index.html || true
}

git clone https://github.com/terryyin/lizard.git

for PACKAGE in $PACKAGE_LIST; do
  exec_lizard $PACKAGE
done
