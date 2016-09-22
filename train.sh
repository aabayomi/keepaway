#!/bin/bash

HIVEMODE="2"
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""
LOGLEVEL="101"
JOINTTILING=""
REMOVECENTERP=""
GAMMA="1."
BUILD="release"

while getopts  "b:h:g:lfmsjr" flag; do
    case "$flag" in
        h) HIVEMODE="$OPTARG";;
        f) FULLSTATE="--fullstate";; 
        m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $LOGLEVEL";;
        j) JOINTTILING="--joint-tiling";;
        r) REMOVECENTERP="--remove-center-position";;
        g) GAMMA="`echo $OPTARG | sed -e 's/[0]*$//g'`";;
        b) BUILD="$OPTARG";;
    esac
done

HIVE="--keeper-hive $HIVEMODE"
QFILE="Q_H${HIVEMODE}"
PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $GAMMA ]; then
    QFILE="${QFILE}_G${GAMMA}"
fi

if [ ! -z $FULLSTATE ]; then
    QFILE="${QFILE}_fs"
fi

if [ ! -z $JOINTTILING ]; then
    QFILE="${QFILE}_jt"
fi

if [ ! -z $REMOVECENTERP ]; then
    QFILE="${QFILE}_rcp"
fi

if [ $BUILD != "none" ]; then
    make $BUILD
fi

ulimit -c unlimited
./keepaway.py --keeper-learn --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNCH $MONITOR $FULLSTATE $LOG $PORT \
    $JOINTTILING $REMOVECENTERP --gamma=$GAMMA --label=$QFILE
