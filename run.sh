#!/bin/bash

HIVEMODE="0"
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""
LOGLEVEL="101"
JOINTTILING=""
GAMMA="1."
BUILD="release"
LEARNING="--keeper-learn --keeper-policy=learned"
QFILE2=""

while getopts  "b:h:g:q:lfmsjn" flag; do
    case "$flag" in
        h) HIVEMODE="$OPTARG";;
        f) FULLSTATE="--fullstate";; 
        m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $LOGLEVEL";;
        j) JOINTTILING="--joint-tiling";;
        g) GAMMA="`echo $OPTARG | sed -e 's/[0]*$//g'`";;
        b) BUILD="$OPTARG";;
        n) LEARNING="--keeper-policy=learned!";;
        q) QFILE2="$OPTARG";;
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

if [ ! -z $QFILE2 ]; then
    QFILE="$QFILE2"
fi

if [ $BUILD != "none" ]; then
    make $BUILD
fi

ulimit -c unlimited
./keepaway.py $LEARNING \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNCH $MONITOR $FULLSTATE $LOG $PORT \
    $JOINTTILING --gamma=$GAMMA --label=`basename $QFILE`

