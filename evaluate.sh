#!/bin/bash

QFILE=""
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""

while getopts  "p:q:l:fms" flag; do
    case "$flag" in
        p) POLICY=$OPTARG;;
        q) QFILE=$OPTARG;;
        f) FULLSTATE="--fullstate";;
        m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $OPTARG";;
    esac
done

PORT="--port=`shuf -i 2000-65000 -n 1`"
LABEL=$POLICY

if [ $POLICY = "learn" ] && [ ! -z $QFILE ]; then
    LABEL="${LABEL}-`basename $QFILE`"
fi

if [ ! -z $FULLSTATE ]; then
    LABEL="${LABEL}_FS"
else
    LABEL="${LABEL}_!FS"
fi

ulimit -c unlimited
./build.sh

./keepaway.py --keeper-policy=$POLICY \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $SYNCH $MONITOR $FULLSTATE $LOG $PORT --label="${LABEL}"

