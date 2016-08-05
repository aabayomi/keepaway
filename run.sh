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
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 08/01/2016 18:59
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

SLEEP="15"

sh clear.sh

#for policy in random hand hold; do
#    ./evaluate.sh -p $policy -s -f &
#    sleep $SLEEP
#done

for hive in `seq 2 2`; do
    ./train.sh -h $hive -sf &
    sleep $SLEEP

    for lookahead in `seq 10 15 100`; do
        gamma=`echo 1.0 - 1.0 / $lookahead | bc -l`
        ./train.sh -h $hive -sf -g $gamma &
        sleep $SLEEP
    done
done

wait

