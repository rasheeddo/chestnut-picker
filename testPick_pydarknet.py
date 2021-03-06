from DeltaRobot import *
import time
import struct
import socket
import select
import pickle
import numpy as np
import zmq
from apscheduler.schedulers.background import BackgroundScheduler

############################# Call DeltaRobot class #######################
deltarobot = DeltaRobot()

deltarobot.RobotTorqueOn()
deltarobot.GripperTorqueOn()
deltarobot.GoHome()
deltarobot.GripperCheck()

############################# UDP / ZMQ socket #######################
## For some reason, detection script doesn't receive ROBOT_STATUS message
## when using ZMQ. But general UDP socket is working fine

# context = zmq.Context()
# DETECT_PORT = '12345'
# detect_sock = context.socket(zmq.SUB)
# detect_sock.connect("tcp://127.0.0.1:" + DETECT_PORT)
# detect_sock.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

# ROBOT_STATUS_PORT = '6666'
# robot_sock = context.socket(zmq.PUB)
# robot_sock.bind("tcp://*:" + ROBOT_STATUS_PORT)

# ## For non-blocking better to use poller
# poller = zmq.Poller()
# poller.register(detect_sock, zmq.POLLIN)
# timeout = 1 # ms

DETECT_PORT = 12345
detect_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
detect_sock.bind(("0.0.0.0", DETECT_PORT))
detect_sock.setblocking(0)

ROBOT_STATUS_PORT = 22334
robot_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

nboxes = 0

######################## Parameters ########################
global grabHeight, XBucket, YBucket, ZBucket, XHome, YHome, ZHome, finishTime, Xlength, Ylength, cameraOffset
 
Xlength = 740.0 # This is a true length from camera view
Ylength = 550.0

grabHeight = -720.0 #-725.0  #-723.0
XBucket = 0.0
YBucket = 550.0
ZBucket = -400.0

XHome = 0.0
YHome = 0.0
ZHome = -329.9795

finishTime = 800

cameraOffset = 95.0  #95.0
roverOffset = 55.0   #on carpet 90.0 with 5rpm

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def pixelToCartesian(px,py):
	global Xlength, Ylength
	x = map(px,0,640,-Xlength/2.0, Xlength/2.0) # use map() give similar result as coor_g()
	y = map(py,0,480,Ylength/2.0,-Ylength/2.0)
	return x,y

def pickAndPlaceGenerate(x_array, y_array):
	global XBucket, YBucket, ZBucket, grabHeight, cameraOffset
	total_points = len(x_array)*4
	#print("len()", total_points)
	#_task_array = np.empty((total_points+1,4), dtype=float)
	_task_array = np.empty((total_points,4), dtype=float)
	
	k = 0
	for i in range(total_points):
		#print(i)
		## Go to that point on the ground and pick
		if i%4 == 0:
			_task_array[i][0] = x_array[k]
			_task_array[i][1] = y_array[k] + cameraOffset
			_task_array[i][2] = grabHeight
			_task_array[i][3] = True
		## Still on that point but lift higher to bucket height
		if i%4 == 1:
			_task_array[i][0] = x_array[k]
			_task_array[i][1] = y_array[k] + cameraOffset
			_task_array[i][2] = ZBucket
			_task_array[i][3] = True
		## Go to bucket and drop
		if i%4 == 2:
			_task_array[i][0] = XBucket
			_task_array[i][1] = YBucket
			_task_array[i][2] = ZBucket
			_task_array[i][3] = False
		## Move inside to the grab field for safety
		if i%4 == 3:
			_task_array[i][0] = XBucket
			_task_array[i][1] = YBucket-200
			_task_array[i][2] = ZBucket
			_task_array[i][3] = False
			k+=1

	# _task_array[i+1][0] = XHome
	# _task_array[i+1][1] = YHome
	# _task_array[i+1][2] = ZHome
	# _task_array[i+1][3] = False

	#print('_task_array', _task_array)

	return _task_array

Done = False
moveFinished = False
grabFinished = False
time_ms = 800
j = 0
startTime = time.time()
gripComm = False
failCount = 0
goPick = False
BUSY = False
once = True



while True:
	######################################## Get detection data ###########################################

	# ## In case of non-blocking process
	# try:
	# 	socks = dict(poller.poll(timeout))
	# except KeyboardInterrupt:
	# 	break

	# if detect_sock in socks:
	# 	data = detect_sock.recv(zmq.NOBLOCK)
	# 	parse_data = pickle.loads(data)
	
	try:
		data, addr = detect_sock.recvfrom(30000)
		parse_data = pickle.loads(data)
	except socket.error:
		pass
	else:

		nboxes = parse_data['nboxes']
		#print("nboxes: ", nboxes)
		#print(parse_data)
		if (nboxes > 0) :
			x_array = np.empty(nboxes,dtype=float)
			y_array = np.empty(nboxes,dtype=float)
			
			for i in range(nboxes):
				#print("i", i)
				#print("p_x %f p_y %f" %(parse_data['xc'][i],parse_data['yc'][i]))
				x_array[i], y_array[i] = pixelToCartesian(parse_data['xc'][i],parse_data['yc'][i])

			goPick = True
			if not BUSY:
				task_array = pickAndPlaceGenerate(x_array, y_array)

		else:
			nboxes = 0

		## send only once is enough 
		if once:
			robot_status_packet = pickle.dumps('READY')
			# robot_sock.send(robot_status_packet)
			robot_sock.sendto(robot_status_packet,("127.0.0.1",ROBOT_STATUS_PORT))
			once = False


	######################################## Delta Robot moving loop ###########################################
	if (goPick):

		if not moveFinished:
			## In while loop, there must be only one GotoPointInTime
			## inside the deltarobot class, it will decide whether to go or not depends on the XYZ input is same as last input
			## it will continutously enter this condition as long as it doesn't reach the point
			
			#print(task_array)
			moveFinished, grabFinished = deltarobot.GotoPointInTime(task_array[j][0],task_array[j][1],task_array[j][2], time_ms, grip=task_array[j][3])
			#print("x %f y %f z %f" %(task_array[j][0],task_array[j][1],task_array[j][2]))
			BUSY = True


		else:
			# j += 1
			# moveFinished = False
			BUSY = True
			
			## It reached the pick point, but suddenly fuck up to pick 
			if moveFinished and not grabFinished:
				## when it was fail to grap on that point (on the ground)
				## most of the time, it will realize when it lifts up from that point
				## we minus it 1 point before
				j -= 1
				print("grab fail")
				moveFinished = False
				## better to open the gripper before, otherwise it's still closed
				deltarobot.GripperOpen()
				failCount += 1
				if failCount == 2:
					print("failGrab many times, don't care")
					j += 3 # we skip to go to the bucket, then just to next pick point, 3 moves is enough
					moveFinished = False
					failCount = 0

			## It reached the pick point and maybe grab success or just no need to grab, then it's ready to go to next point 
			else:
				j += 1
				moveFinished = False

		print("j", j)
		## all points in task is finished
		## get out of this loop and enable detection again by set BUSY to False
		

		if j == len(task_array):
			#deltarobot.GoHome()
			#time.sleep(3)
			print("j == len(task_array)")
			goPick = False
			moveFinished = False
			BUSY = False
			## clear data
			xc_array = np.zeros(40,dtype=float)
			yc_array = np.zeros(40,dtype=float)
			j = 0
			once = True
			deltarobot.GoHome()
			## this will make the camera steady enough before swtich to READY and doing a detection
			## 0.8 because the velocity profile takes 800ms to complete movement
			time.sleep(0.8)		
			#task_array = None
			#break

		

		## during the robot is moving, it need to send BUSY status to let chestnut-detection knows
		## otherwise, it will also do the detection all the time and we just care only the field area
		if BUSY:
			robot_status_packet = pickle.dumps('BUSY')
			# robot_sock.send(robot_status_packet)
			robot_sock.sendto(robot_status_packet,("127.0.0.1",ROBOT_STATUS_PORT))





	