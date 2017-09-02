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

NPROC=`nproc`
N=`expr $NPROC / 2  - 1`

make clean
make -j `nproc` release

exec 1>console.log 2>&1                                                              

./train.sh -b none -sf -g 1.0 -L 0.5 -A 0.125 -I 0.5 $* & #hamq

gamma=1.0

for i in `seq $N`; do
    lambda=$(python -c "import random; print('{:.3f}'.format(random.uniform(0.0, 1.0)))")
    alpha=$(python -c "import random; print('{:.3f}'.format(random.uniform(0.001, 0.25)))")
    initialweight=$(python -c "import random; print('{:.3f}'.format(random.uniform(0.0, 1.0)))")

    ./train.sh -b none -sf -g $gamma -L $lambda -A $alpha -I $initialweight $* & #hamq
done

wait

