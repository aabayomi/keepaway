#!/bin/bash

kill -INT `pidof lt-rcssserver`
kill -INT `pidof rcssserver`
killall rcssserver
killall lt-rcssserver

