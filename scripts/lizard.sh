#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
RESULT_DIR="$BASE_DIR/lizard_result"
PACKAGE_LIST=$(colcon list --names-only)
PACKAGE_LIST_FULL=$(colcon list)

function get_package_path() {
  [ "$1" == "" ] && return 1

  echo "$PACKAGE_LIST_FULL" | grep -w "$1" | awk '{ print $2 }'
  return 0
}

function exec_lizard() {
  [ "$1" == "" ] && return 1

  if [ ! -d ${RESULT_DIR}/$1 ]; then
    mkdir -p ${RESULT_DIR}/$1
  fi

  PACKAGE_PATH=$(get_package_path "$1")

  python3 lizard/lizard.py -l cpp -l python -x "*test*" -x "*lizard*" \
    --CCN 20 -T nloc=200 --arguments 6 \
    --html $PACKAGE_PATH > ${RESULT_DIR}/$1/index.html || true
}

git clone https://github.com/terryyin/lizard.git

for PACKAGE in $PACKAGE_LIST; do
  exec_lizard $PACKAGE
done
