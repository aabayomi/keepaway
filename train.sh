#!/bin/bash

HIVEMODE="0"
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""

while getopts  "h:l:fms" flag; do
    case "$flag" in
        h) HIVEMODE=$OPTARG;;
        f) FULLSTATE="--fullstate";;
        m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $OPTARG";;
    esac
done

HIVE="--keeper-hive $HIVEMODE"
QFILE="hive${HIVEMODE}-Q"
PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $FULLSTATE ]; then
    QFILE="${QFILE}_fs"
fi

ulimit -c unlimited
./build.sh

./keepaway.py --keeper-learn --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNCH $MONITOR $FULLSTATE $LOG $PORT --label=$QFILE

