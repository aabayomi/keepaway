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
make -j `nproc` release

exec 1>console.log 2>&1                                                              

lambda=0.5
initialweight=0.5

./train.sh -z -b none -sf -g 1.0 -L $lambda -I $initialweight $* & #hamq
./train.sh -z -b none -sf -g 1.0 -L $lambda -I $initialweight $* -T & #hamq-int
./train.sh -b none -sf -g 1.0 -L $lambda -I $initialweight $* & #joint option learning

wait

