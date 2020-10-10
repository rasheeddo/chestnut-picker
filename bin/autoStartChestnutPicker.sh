#!/bin/bash

export DISPLAY=:0
export LOGFILE=/home/nvidia/autoStartChestnutPicker.log
cd /home/nvidia/chestnut-picker

while :
do	
	echo >>$LOGFILE
	echo "----------------------------------------------" >> $LOGFILE
	date >> $LOGFILE

	python3 -u chestnut_picker_demo.py &>> $LOGFILE

	echo "something wrong with chestnut_picker_demo.py" &>> $LOGFILE
	date >> $LOGFILE
	sleep 1

done


