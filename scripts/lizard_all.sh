#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1

# Set generated timestamp
if [ $# -eq 6 ]; then
  TIMESTAMP=$(cat $6)
else
  TIMESTAMP=$(date -u '+%Y%m%d_%H%M%S')
fi

[ "$2" == "" ] && { echo "Please set output directory." ; exit 1; }
OUTPUT_DIR=${2}/lizard_result/${TIMESTAMP}/all

function exec_lizard() {
  [ "$1" == "" ] && return 1

  if [ ! -d ${OUTPUT_DIR} ]; then
    mkdir -p ${OUTPUT_DIR}
  fi

  python3 lizard/lizard.py -l cpp -l python -x "*test*" -x "*lizard*" \
    --CCN $2 -T nloc=$3 --arguments $4 \
    --html $1 > ${OUTPUT_DIR}/index.html || true
}

if [ ! -d "lizard" ]; then
  git clone https://github.com/terryyin/lizard.git
fi

exec_lizard $BASE_DIR $3 $4 $5
