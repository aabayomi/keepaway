#!/bin/bash

source initrc

HIVEMODE="0"

if [ ! -z $1 ]; then
    HIVEMODE=$1
fi

HIVE="--keeper-hive $HIVEMODE"
QFILE="hive-${HIVEMODE}-Q"

./keepaway.py --keeper-learn --keeper-policy=learn \
    --keeper-output=$QFILE --keeper-input=$QFILE \
    $HIVE $SYNC $MONITOR $LOG $PORT --label=$QFILE

