#!/bin/bash

FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""
LOGLEVEL="101"
HIERARCHICALFSM=""
QLEARNING=""
GAMMA="1.0"
INITIALWEIGHT="0.5"
BUILD="release"
LEARNING="--keeper-learn --keeper-policy=learned"
LOGDIR="logs"
QFILE="pi"
QFILE2=""
MEMORYCHECK=""

while getopts  "b:g:q:I:lfmsnzMQ" flag; do
    case "$flag" in
        f) FULLSTATE="--fullstate" ;; 
        m) MONITOR="--monitor" ;;
        s) SYNCH="--synch-mode" ;;
        l) LOG="--log-dir=$LOGDIR --log-game --log-text --log-level $LOGLEVEL" ;;
        z) HIERARCHICALFSM="--hierarchical-fsm" ;;
        Q) QLEARNING="--qlearning" ;;
        g) GAMMA="`echo $OPTARG | sed -e 's/[0]*$//g'`" ;;
        I) INITIALWEIGHT="$OPTARG" ;;
        b) BUILD="$OPTARG" ;;
        n) LEARNING="--keeper-policy=learned!" ;;
        q) QFILE2="$OPTARG" ;;
        M) MEMORYCHECK="--memory-check" ;;
    esac
done

PORT="--port=`shuf -i 2000-65000 -n 1`"

if [ ! -z $GAMMA ]; then
    QFILE="${QFILE}_g${GAMMA}"
fi

if [ ! -z $INITIALWEIGHT ]; then
    QFILE="${QFILE}_w${INITIALWEIGHT}"
fi

if [ ! -z $FULLSTATE ]; then
    QFILE="${QFILE}_fs"
fi

if [ ! -z $HIERARCHICALFSM ]; then
    QFILE="${QFILE}_fsm"
fi

if [ ! -z $QLEARNING ]; then
    QFILE="${QFILE}_ql"
fi

QFILE="${QFILE}.q"

if [ ! -z $QFILE2 ]; then
    QFILE="$QFILE2" # overwrite QFILE
fi

if [ $BUILD != "none" ]; then
    make $BUILD
fi

CONSOLE_LOG="$LOGDIR/`basename $QFILE .q`.console"

ulimit -c unlimited
./keepaway.py $MEMORYCHECK $LEARNING \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $SYNCH $MONITOR $FULLSTATE $LOG $PORT \
    $HIERARCHICALFSM --gamma=$GAMMA --initial-weight=$INITIALWEIGHT \
    $QLEARNING --label=`basename $QFILE .q` 2>&1 | tee $CONSOLE_LOG

