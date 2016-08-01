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

./winsum.sh $1 &
./hist.sh $1 &

