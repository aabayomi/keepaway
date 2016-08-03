#!/bin/bash

source initrc

LOG="1"
MONITOR="--monitor"

POLICY="$1" # hold, hand, random or learned

QFILE=""
LABEL=`basename $POLICY`
if [ ! -z $2 ]; then
    QFILE="$2" # learned weight file
    LABEL="${LABEL}-$QFILE"
fi

./keepaway.py --keeper-policy=$POLICY \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $SYNC $MONITOR $LOG $PORT --label=${LABEL}

