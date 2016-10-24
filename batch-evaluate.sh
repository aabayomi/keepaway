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
make -j `nproc` release

exec 1>console.log 2>&1                                                              

for q in "$@"; do
    ./evaluate.sh -b none -q $q $* &
    sleep $SLEEP
done

wait

