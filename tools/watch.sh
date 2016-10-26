#!/bin/bash - 
#===============================================================================
#
#          FILE: watch.sh
# 
#         USAGE: ./watch.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: Aijun Bai (), 
#  ORGANIZATION: 
#       CREATED: 10/25/2016 18:48
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

while [ true ]; do
    ./plotall.sh
    sleep 300
done

