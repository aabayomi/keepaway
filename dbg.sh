#!/bin/bash

rm -f .gdbinit
while read LINE; do
    if [[ ${LINE:0:1} != "#" ]]; then
        echo "b $LINE" >> .gdbinit
    fi
done <breakpoints

cgdb -- ./player/keepaway_player `ps -o pid= -C keepaway_player`

