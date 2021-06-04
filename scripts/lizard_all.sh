#!/bin/bash

set -e

[ "$1" == "" ] && { echo "Please set base directory." ; exit 1; }
BASE_DIR=$1
ACTION_DIR=$3

# Set generated timestamp
if [ $# -eq 7 ]; then
  TIMESTAMP=$(cat $7)
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

  python3 $ACTION_DIR/lizard/lizard.py \
    -l cpp \
    -l python \
    -x "*test*" \
    -x "*lizard*" \
    --CCN $2 \
    -T nloc=$3 \
    --arguments $4 \
    --html $1 > ${OUTPUT_DIR}/index.html || true
}

if [ ! -d $ACTION_DIR/lizard ]; then
  git clone https://github.com/terryyin/lizard.git $ACTION_DIR/lizard
fi

exec_lizard $BASE_DIR $3 $4 $5
