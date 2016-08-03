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
#        AUTHOR: YOUR NAME (), 
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
    ./winsum.sh $var `basename $var .kwy`
    ./hist.sh $var `basename $var .kwy`
done

gnuplot graph.gnuplot
gnuplot hist.gnuplot

evince graph.eps &
evince hist.eps &

