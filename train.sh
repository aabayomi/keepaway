#!/bin/bash

HIVEMODE="0"
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""
LOGLEVEL="101"
JOINTTILING=""
USECENTERP=""

while getopts  "h:lfmszu" flag; do
    case "$flag" in
        h) HIVEMODE=$OPTARG;;
        f) FULLSTATE="--fullstate";; m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $LOGLEVEL";;
        z) JOINTTILING="--joint-tiling";;
        u) USECENTERP="--use-center-position";;
    esac
done

HIVE="--keeper-hive $HIVEMODE"
QFILE="hive${HIVEMODE}-Q"
PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $FULLSTATE ]; then
    QFILE="${QFILE}_fs"
else
    QFILE="${QFILE}_nfs"
fi

if [ ! -z $JOINTTILING ]; then
    QFILE="${QFILE}_jt"
else
    QFILE="${QFILE}_njt"
fi

if [ ! -z $USECENTERP ]; then
    QFILE="${QFILE}_ucp"
else
    QFILE="${QFILE}_nucp"
fi

ulimit -c unlimited
./build.sh

./keepaway.py --keeper-learn --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNCH $MONITOR $FULLSTATE $LOG $PORT \
    $JOINTTILING --label=$QFILE

