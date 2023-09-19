#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "$BASH_SOURCE")"; pwd)"
SCRIPT_DIR=$(readlink -f $SCRIPT_DIR)  # this reads the actual path if a symbolic directory is used

CHECK=""
while true; do
    echo waiting for rosmaster
    CHECK=$(ps -ax | grep "rosmaster" | grep 'core')
    if [ -n "$CHECK" ]; then # esci dal ciclo 
    break 
    fi
    sleep 1
done 
echo rosmaster started