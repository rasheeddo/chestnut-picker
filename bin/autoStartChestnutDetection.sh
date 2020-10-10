#!/bin/bash

export DISPLAY=:0
export LOGFILE=/home/nvidia/autoStartChestnutDetection.log
cd /home/nvidia/pydarknet

while true
do	
	echo >>$LOGFILE
	echo "----------------------------------------------" >> $LOGFILE
	date >> $LOGFILE

	python3 -u chestnut-detection.py &>> $LOGFILE

	echo "something wrong with chestnut_detection.py" &>> $LOGFILE
	date >> $LOGFILE
	sleep 1

done


