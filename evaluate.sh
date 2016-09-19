#!/bin/bash

QFILE=""
FULLSTATE=""
MONITOR=""
SYNCH=""
LOG=""
LOGLEVEL="101"
BUILD="release"

while getopts  "b:p:q:lfms" flag; do
    case "$flag" in
        p) POLICY=$OPTARG;;
        q) QFILE=$OPTARG;;
        f) FULLSTATE="--fullstate";;
        m) MONITOR="--monitor";;
        s) SYNCH="--synch-mode";;
        l) LOG="--log-dir=logs --log-game --log-text --log-level $LOGLEVEL";;
        b) BUILD="$OPTARG";;
    esac
done

PORT="--port=`shuf -i 2000-65000 -n 1`"
LABEL="$POLICY"

if [[ "$POLICY" == learn* ]] && [ ! -z "$QFILE" ]; then
    LABEL="${LABEL}-`basename $QFILE`"
fi

if [ ! -z $FULLSTATE ]; then
    LABEL="${LABEL}_FS"
fi

if [ $BUILD != "none" ]; then
    make $BUILD
fi

ulimit -c unlimited
./keepaway.py --keeper-policy="$POLICY" \
    --keeper-output="$QFILE" --keeper-input="$QFILE" \
    "$SYNCH" "$MONITOR" "$FULLSTATE" $LOG "$PORT" --label="${LABEL}"

