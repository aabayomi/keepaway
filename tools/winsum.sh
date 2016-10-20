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
#        AUTHOR: Aijun Bai (), 
#  ORGANIZATION: 
#       CREATED: 07/30/2016 22:54
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

WINDOW="100"
ALPHA="0.01"
COARSE="`expr $WINDOW / 10`"
OUTPUT=$2

echo $WINDOW $ALPHA $COARSE

cat $1 | ./winsum $WINDOW $ALPHA $COARSE > $OUTPUT.out
echo "\"$OUTPUT.out\" w lp lw 3 t \"$OUTPUT\", \\" >> graph.gnuplot

