#!/bin/bash - 
#===============================================================================
#
#          FILE: trainall.sh
# 
#         USAGE: ./trainall.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 08/01/2016 00:27
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

./train.sh &
./train-hive.sh &

wait

