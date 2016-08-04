#!/bin/bash - 
#===============================================================================
#
#          FILE: winsum.sh
# 
#         USAGE: ./winsum.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 07/30/2016 22:54
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

WINDOW="100"
ALPHA="0.01"
COARSE="100"

cat $1 | ./winsum $WINDOW $ALPHA $COARSE > $2.out
echo "\"$2.out\" w lp lw 3, \\" >> graph.gnuplot

