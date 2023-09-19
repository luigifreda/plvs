#!/bin/bash

PROC_NAME=$1
WAIT_TIME=$2 

if [[ -n "$PROC_NAME" ]]; then
  echo ...
else 
  PROC_NAME=rgbd_tum
fi 

if [[ -n "$WAIT_TIME" ]]; then
  echo ...
else 
  WAIT_TIME=1
fi 

echo monitoring $PROC_NAME starting wait time $WAIT_TIME ...

sleep $WAIT_TIME

PID=$(pidof $PROC_NAME)
if [ "$PID" == "" ]; then 
    echo "process does not exist"
    exit -1
fi

#echo q | htop -C -p=$PID | aha --line-fix | html2text -width 999 | grep -v "F1Help" | grep -v "xml version=" | awk '$1=$1' | cut -d " " -f 9-10 > Resources.txt
echo q | htop -C | aha --line-fix | html2text -width 999 | grep -v "F1Help" | grep -v "xml version=" | awk '$1=$1' | grep $PID | cut -d " " -f 9-10 > Resources.txt
sleep 1

while [ -e /proc/$PID ]; do
    #echo q | htop -C -p=$PID | aha --line-fix | html2text -width 999 | grep -v "F1Help" | grep -v "xml version=" | awk '$1=$1' | grep $PID | cut -d " " -f 9-10 >> Resources.txt
    MEM_CPU=$(echo q | htop -C | aha --line-fix | html2text -width 999 | grep -v "F1Help\|xml version=" | awk '$1=$1' | grep $PID | grep "$USER ") 
    #echo "MEM_CPU: $MEM_CPU"
    MEM_CPU_CUT=$(echo q "$MEM_CPU" | cut -d " " -f 10-11) 
    #echo "MEM_CPU_CUT: $MEM_CPU_CUT"
    if [ "$MEM_CPU_CUT" != "" ]; then
      #echo "non-empty string"
      echo $MEM_CPU_CUT >> Resources.txt
    fi
    sleep 1
done
