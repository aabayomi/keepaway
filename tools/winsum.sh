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

cat $1 | ./winsum 1000 0.01 > 1.out
gnuplot graph.gnuplot
evince graph.eps


