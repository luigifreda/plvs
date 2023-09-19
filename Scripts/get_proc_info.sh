#!/bin/bash 


LOG_FILE_NAME=$1
if [[ -n "$LOG_FILE_NAME" ]] 
then
    echo "log file: $LOG_FILE_NAME" 
else
	LOG_FILE_NAME="proc_info.log"
fi

cat /proc/meminfo &> $LOG_FILE_NAME
sleep 1
	
while true; do
	#echo "logging"
	cat /proc/meminfo &>> $LOG_FILE_NAME
	sleep 1
done
