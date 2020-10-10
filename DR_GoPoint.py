from DeltaRobot import *
import time
import numpy as np

### Call DeltaRobot class ####
deltarobot = DeltaRobot()

deltarobot.RobotTorqueOn()
deltarobot.GripperTorqueOn()

################## Go to stand by position before starting  ###########################

deltarobot.GoHome()
deltarobot.GripperCheck()
time.sleep(0.5)

global XBucket, YBucket, ZBucket, grabHeight, XHome, YHome, ZHome
XBucket = 0.0
YBucket = 550.0
ZBucket = -400.0
grabHeight = -730.0  #-723.0

XHome = 0.0
YHome = 0.0
ZHome = -329.9795

x_array = np.array([223, 244, -204, -183])
y_array = np.array([-25, 208, 206, -73])

def pickAndPlaceGenerate(x_array, y_array):
	global XBucket, YBucket, ZBucket, grabHeight, XHome, YHome, ZHome
	total_points = len(x_array)*4
	print("len()", total_points)
	task_array = np.empty((total_points+1,4), dtype=float)
	print(task_array)
	j = 0
	for i in range(total_points):
		print(i)
		## Go to that point on the ground and pick
		if i%4 == 0:
			task_array[i][0] = x_array[j]
			task_array[i][1] = y_array[j]
			task_array[i][2] = grabHeight
			task_array[i][3] = True
		## Still on that point but lift higher to bucket height
		if i%4 == 1:
			task_array[i][0] = x_array[j]
			task_array[i][1] = y_array[j]
			task_array[i][2] = ZBucket
			task_array[i][3] = True
		## Go to bucket and drop
		if i%4 == 2:
			task_array[i][0] = XBucket
			task_array[i][1] = YBucket
			task_array[i][2] = ZBucket
			task_array[i][3] = False
		## Move inside to the grab field for safety
		if i%4 == 3:
			task_array[i][0] = XBucket
			task_array[i][1] = YBucket-200
			task_array[i][2] = ZBucket
			task_array[i][3] = False
			j+=1

	## Last point is bring to home to avoid it detects in the box
	task_array[i+1][0] = XHome
	task_array[i+1][1] = YHome
	task_array[i+1][2] = ZHome
	task_array[i+1][3] = False

	return task_array


Done = False
moveFinished = False
grabFinished = False
time_ms = 800
i = 0
startTime = time.time()
gripComm = False
failCount = 0

#targets = np.array([[250,250,-500],[-300,300,-500],[-150,-150,-500],[100,-100,-500]])

#task_array = np.array([[223,-25,-720, 0],[244,208,-720, 0],[-204,206,-720, 0],[-183,-73,-720, 0]])
task_array = pickAndPlaceGenerate(x_array, y_array)
# print(task_array)

while not Done:

	if not moveFinished:
		## In while loop, there must be only one GotoPointInTime
		## inside the deltarobot class, it will decide whether to go or not depends on the XYZ input is same as last input
		moveFinished, grabFinished = deltarobot.GotoPointInTime(task_array[i][0],task_array[i][1],task_array[i][2], time_ms, grip=task_array[i][3])
		#print("Done: ", Done)
		#print("Finished: ", Finished)
	else:
		## It reached the pick point, but suddenly fuck up to pick 
		if moveFinished and not grabFinished:
			i -= 1
			print("grab fail")
			moveFinished = False
			deltarobot.GripperOpen()
			failCount += 1
			if failCount == 2:
				print("failGrab ,fuck it")
				# print("i: ", i)
				# period = time.time() - startTime
				# print("period: ", period)
				startTime = time.time()
				i += 3 # we skip to go to the bucket, then just to next pick point, 3 moves is enough
				moveFinished = False
				failCount = 0
		## It reached the pick point and maybe grab success or just no need to grab, then it's ready to go to next point 
		else:
			print("i: ", i)
			# period = time.time() - startTime
			# print("period: ", period)
			# startTime = time.time()
			i += 1
			moveFinished = False


	if i == len(task_array):
		break


#deltarobot.GoHome()
print("Done!")

