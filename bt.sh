#! /bin/bash

CMDS=`mktemp`

echo -e "bt\nq" > $CMDS

for i in `ls core.*`; do
	gdb -c $i -x $CMDS player/keepaway_player
	rm -f $i
done

rm -f $CMDS
