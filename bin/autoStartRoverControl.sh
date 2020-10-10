#!/bin/bash

export DISPLAY=:0
export LOGFILE=/home/nvidia/autoStartRoverControl.log
cd /home/nvidia/chestnut-picker

while :
do	
	echo >>$LOGFILE
	echo "----------------------------------------------" >> $LOGFILE
	date >> $LOGFILE

	python3 -u rover_control.py &>> $LOGFILE

	echo "something wrong with rover_control.py" &>> $LOGFILE
	date >> $LOGFILE
	sleep 1

done


