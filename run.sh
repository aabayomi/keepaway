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

./clear.sh

for policy in random hand hold; do
    ./evaluate.sh -p $policy -s -f &
    sleep $SLEEP
done

for hive in `seq 0 2`; do
    ./train.sh -h $hive -s -f &
    sleep $SLEEP
done

for hive in `seq 0 2`; do
    ./train.sh -h $hive -s -f -z &
    sleep $SLEEP
done

wait

