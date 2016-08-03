#!/bin/bash

QFILE="$1"

LOG=""
#LOG="1"

#MONITOR=""
MONITOR="--monitor"

PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $LOG ]; then
    LOG="--log-dir=logs --log-game --log-text --log-level 101"
fi

SYNC="--synch-mode"
if [ ! -z $MONITOR ]; then
    SYNC=""
fi

ulimit -c unlimited
./build.sh

./keepaway.py --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $SYNC $MONITOR $LOG $PORT

