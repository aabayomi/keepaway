#!/bin/bash - 
#===============================================================================
#
#          FILE: clear.sh
# 
#         USAGE: ./clear.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: YOUR NAME (), 
#  ORGANIZATION: 
#       CREATED: 08/01/2016 14:44
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

sh kill.sh
rm -fr hive*-Q* logs/ *.lock core core.* vgcore.* *.lock nohup.out

mkdir logs
cd player
make clean

cd ..
cd tools
make clean
