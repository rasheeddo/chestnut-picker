from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time
import pickle
import socket


ROVER_PORT = 7777
rover_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rover_sock.bind(("0.0.0.0", ROVER_PORT))
rover_sock.setblocking(0)

ROVER_STATUS_PORT = 8888
rover_status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


print("Connecting to the CUBE")
vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=921600)
print("Connection is success")

global THR_VAL

# def pushThrottle():
# 	global THR_VAL
# 	vehicle.channels.overrides['2'] = THR_VAL
# 	#print("push throttle")


# sched = BackgroundScheduler()
# sched.add_job(pushThrottle, 'interval', seconds = 1/10)

# sched.start()

THR_VAL = 1500
t1 = time.time()
send_t1 = time.time()
send_t2 = time.time()
current_mode = 'MANUAL'

while True:
	
	try:
		data, addr = rover_sock.recvfrom(1024)
		THR_VAL = pickle.loads(data)
		#print("THR_VAL: ", THR_VAL)
		t1 = time.time()
	except socket.error:
		t2 = time.time()
		#print("Not receive THR_VAL")
		if (t2 - t1 > 2.00):
			THR_VAL = 1500
		pass
	else:
		send_t1 = time.time()
		if (abs(send_t2 - send_t1) > 0.1):
			send_t2 = time.time()
			print("THR_VAL: ", THR_VAL)


	if vehicle.mode.name == 'HOLD':
		current_mode = 'HOLD'
		rover_status_packet = pickle.dumps('HOLD')
		rover_status_sock.sendto(rover_status_packet, ('127.0.0.1', ROVER_STATUS_PORT))
	else:
		current_mode = 'MANUAL'		
		rover_status_packet = pickle.dumps('RUN')
		rover_status_sock.sendto(rover_status_packet, ('127.0.0.1', ROVER_STATUS_PORT))

	if current_mode == 'MANUAL':
		vehicle.channels.overrides['2'] = THR_VAL


	

