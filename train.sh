#!/bin/bash

HIVE="--keeper-hive"
HIVE=""

MONITOR="--monitor"
MONITOR=""

FILEPERPLAYER="--file-per-player"
FILEPERPLAYER=""

LOG="true"
LOG=""

PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $LOG ]; then
    LOG="--log-dir=logs --log-game --log-text"
fi

FILE="Q"
if [ ! -z $HIVE ]; then
    FILE="hive-$FILE"
fi

if [ ! -z $FILEPERPLAYER ]; then
    FILE="$FILE#"
fi

SYNC="--synch-mode"
if [ ! -z $MONITOR ]; then
    SYNC=""
fi

DIR=`pwd`
cd player
make clean
make
cd $DIR

./tools/killserver
killall rcssmonitor

./keepaway.py --keeper-learn --keeper-policy=learn --keeper-output=$FILE --keeper-input=$FILE $HIVE $SYNC $MONITOR $FILEPERPLAYER $LOG $PORT

