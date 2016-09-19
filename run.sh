#!/bin/bash - 
#===============================================================================
#
#          FILE: run.sh
# 
#         USAGE: ./run.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: Aijun Bai (), 
#  ORGANIZATION: 
#       CREATED: 08/01/2016 18:59
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

SLEEP="10"
FULLSTATE="-f"

exec > console.log                                                              
exec 2>&1

make clean
make release

for hive in `seq 1 2`; do
    for lookahead in `seq 1 10`; do
        gamma=`echo 1.0 - 1.0 / 2^$lookahead | bc -l`
        ./train.sh -b none -h $hive -s $FULLSTATE -g $gamma &
        sleep $SLEEP
    done

    ./train.sh -b none -h $hive -s $FULLSTATE &
done

for policy in random hand hold; do
    sleep $SLEEP
    ./evaluate.sh -b 0 -p $policy -s $FULLSTATE &
done

wait

