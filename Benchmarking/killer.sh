#!/bin/bash

if [[ -n "$PROC_NAME" ]]; then
  echo ...
else 
  PROC_NAME=RGBD
fi 

sleep 10
PID=`pidof play`
echo "pidof rosbag player = $PID"
while [ -e /proc/$PID ]; do
    sleep 1 # sleep while rosbag is playing 
done
sleep 5

PID_PROC=$(pidof $PROC_NAME)
echo "pidof executable $PROC_NAME to kill= $PID_PROC"
kill -s SIGINT $PID_PROC
sleep 1
