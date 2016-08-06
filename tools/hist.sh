#!/bin/bash - 
#===============================================================================
#
#          FILE: hist.sh
# 
#         USAGE: ./hist.sh 
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

OUTPUT=$2

cat $1 | ./hist 10.0 > $OUTPUT.hist
echo "\"$OUTPUT.hist\" w boxes t \"$OUTPUT\", \\" >> hist.gnuplot

