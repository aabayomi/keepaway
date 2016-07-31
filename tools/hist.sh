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
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 07/30/2016 22:54
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

cat $1 | ./hist 2.0 > 1.hist
gnuplot hist.gnuplot
gv hist.eps

