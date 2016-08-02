#!/bin/bash

HIVEMODE="0"

if [ ! -z $1 ]; then
    HIVEMODE=$1
fi

MONITOR=""
MONITOR="--monitor"

FILEPERPLAYER="--file-per-player"
FILEPERPLAYER=""

LOG=""
LOG="true"

PORT="--port=`shuf -i 2000-65000 -n 1`"
HIVE="--keeper-hive $HIVEMODE"
QFILE="hive-${HIVEMODE}-Q"

if [ ! -z $LOG ]; then
    LOG="--log-dir=logs --log-game --log-text --log-level 101"
fi

if [ ! -z $FILEPERPLAYER ]; then
    QFILE="$QFILE#"
fi

SYNC="--synch-mode"
if [ ! -z $MONITOR ]; then
    SYNC=""
fi

ulimit -c unlimited
./build.sh

./keepaway.py --keeper-learn --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNC $MONITOR $FILEPERPLAYER $LOG $PORT

