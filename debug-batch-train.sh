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

make clean
make debug

for initialweight in 0.0 1.0; do
    ./train.sh -l -b none -sf -g 1.0 -I $initialweight $* &
    ./train.sh -l -z -b none -sf -g 1.0 -I $initialweight $* &
done

wait

