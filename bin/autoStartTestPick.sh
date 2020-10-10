#!/bin/bash

export DISPLAY=:0
export LOGFILE=/home/nvidia/autoStartTestPick.log
cd /home/nvidia/chestnut-picker

while :
do	
	echo >>$LOGFILE
	echo "----------------------------------------------" >> $LOGFILE
	date >> $LOGFILE

	python3 -u testPick_pydarknet.py &>> $LOGFILE

	echo "something wrong with testPick_pydarknet.py" &>> $LOGFILE
	date >> $LOGFILE
	sleep 1

done


