from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time 

print("Connecting to the CUBE")
vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=921600)
print("Connection is success")

#lock = threading.Lock()

def pushThrottle():
	vehicle.channels.overrides['2'] = 1550
	print("push throttle")


sched = BackgroundScheduler()
sched.add_job(pushThrottle, 'interval', seconds = 1/10)

sched.start()

while True:
	print("do something in while loop...")
	time.sleep(0.5)
