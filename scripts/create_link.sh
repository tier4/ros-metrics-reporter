#!/bin/bash

set -e

if [ -e $2 ]; then
  rm -rf $2
fi

ln -nfrs $1 $2
