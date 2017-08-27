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

for i in `seq 4`; do
    lambda=$(python -c "import random; print(random.uniform(0.0, 1.0))")
    gamma=$(python -c "import random; print(random.uniform(0.9, 1.0))")
    initialweight=$(python -c "import random; print(random.uniform(0.0, 1.0))")

    ./train.sh -z -b none -sf -g $gamma -L $lambda -I $initialweight $* & #hamq
    ./train.sh -z -b none -sf -g $gamma -L $lambda -I $initialweight $* -T & #hamq-int
done

wait

