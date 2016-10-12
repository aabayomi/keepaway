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

make clean
make release

exec 1>console.log 2>&1                                                              
for initialweight in 0.125 0.25 0.5; do
    for lookahead in `seq 5 10`; do
        gamma=`echo 1.0 - 1.0 / 2^$lookahead | bc -l`
        ./train.sh -b none -sf -g $gamma -I $initialweight $* &
        sleep $SLEEP
    done
    ./train.sh -b none -sf -g 1.0 -I $initialweight $* &
done

wait

