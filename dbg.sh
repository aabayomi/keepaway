#!/bin/bash

rm -f .gdbinit
while read LINE; do
    if [[ ${LINE:0:1} != "#" ]]; then
        echo "b $LINE" >> .gdbinit
    fi
done <breakpoints
echo "c" >> .gdbinit

cgdb -- ./player/keepaway_player -x .gdbinit `ps -o pid= -C keepaway_player`

