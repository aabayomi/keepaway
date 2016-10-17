#!/bin/bash - 
#===============================================================================
#
#          FILE: plotall.sh
# 
#         USAGE: ./plotall.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: Aijun Bai (), 
#  ORGANIZATION: 
#       CREATED: 07/31/2016 00:18
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

make clean
make

cp graph.gnuplot.template graph.gnuplot
cp hist.gnuplot.template hist.gnuplot

for var in "$@"; do
    output="`echo $var | sed -e 's/-/ /g' | awk '{$1 = ""; print $0;}' | sed -e 's/ //g'`"
    output="`basename $output .kwy`"
    output="`echo $output | sed -e 's/_/-/g' -e 's/gamma/g/g' -e 's/initialweight/w/g' -e 's/fullstate//g'`"
    ./winsum.sh $var $output
    ./hist.sh $var $output
done

gnuplot graph.gnuplot
gnuplot hist.gnuplot

evince graph.eps &
evince hist.eps &

