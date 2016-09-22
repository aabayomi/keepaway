#!/bin/bash

for i in `seq 1 100`; do
    kill -INT `pidof lt-rcssserver` 1>/dev/null 2>&1
    kill -INT `pidof rcssserver` 1>/dev/null 2>&1
    killall rcssserver 1>/dev/null 2>&1
    killall rcssmonitor 1>/dev/null 2>&1
    killall lt-rcssserver 1>/dev/null 2>&1
done 

