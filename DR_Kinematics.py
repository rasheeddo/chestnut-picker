from DeltaRobot import *
import time

deltarobot = DeltaRobot()

deltarobot.RobotTorqueOff()
deltarobot.GripperTorqueOff()
#deltarobot.RobotTorqueOn()
#deltarobot.GripperTorqueOn()

#deltarobot.GoHome()
#time.sleep(2)
#deltarobot.GripperCheck()

#deltarobot.GotoPoint(150,-150,-700)

deltarobot.KinematicsCheck()

