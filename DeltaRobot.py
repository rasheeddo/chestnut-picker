import time
import numpy as np
#import pygame
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class DeltaRobot:
	def __init__(self):

		##############################################################################################################################################
		######################################################### Math Constants ###################################################################
		#############################################################################################################################################
		self.rad2deg = 180/np.pi
		self.deg2rad = np.pi/180
		self.pi = np.pi
		self.root3 = np.sqrt(3)

		#############################################################################################################################################
		####################################################### Robot's Parameters ##################################################################
		#############################################################################################################################################

		self.sb = 311.769   	#315.339 delta MK1		
		self.sp = 87.134 		#88.335 delta MK1	
		self.L = 399.147 		#293 delta MK1	 
		self.l = 683.5 			#555 delta MK1	

		self.wb = (self.root3/6)*self.sb
		self.ub = (self.root3/3)*self.sb
		self.wp = (self.root3/6)*self.sp
		self.up = (self.root3/3)*self.sp

		#############################################################################################################################################
		####################################################### Constraint Parameters ###############################################################
		#############################################################################################################################################

		# Workspace Constraint
		self.maxR = 560.0 			#300 delta MK1				
		self.minStoke = -300.0		#-300 delta MK1	
		self.maxStoke = -1000.0		#-725 delta MK1	

		## Velocity Constraint
		self.set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		self.set_V_Limit = 500       # 350 Default                  [0.229RPM]

		## Joint Constraint
		self.OFFSET_ANGLE = 3.8427      # This offset is for DELTA ROBOT MK2 only due to the joint offset between
										# proximal and distal link, it will be used in FWD and INV

		## Gripper Constraint
		self.GRIPPER_OPEN = 2624	#2624
		self.GRIPPER_OPEN_75 = 2300	#2624
		self.GRIPPER_CLOSE = 1930	#1876
		self.GRIP_MID = 2277
		self.minObjectExisted = -95		# This is current value when it grab something
										# comes from experiment, if the current is < -50 means gripper has some object
										# with foam this is -90

		self.GRIPPER_CURRENT = 250
		self.GRIPPER_Tf = 80	#150	#300
		self.GRIPPER_Ta = 40 	#75		#150
		self.GRIP_STATUS = False   # True: closed,   False: opened
		self.countMissedGrab = 0

		self.BUSY = False
		self.targetX = None
		self.targetY = None
		self.targetZ = None

		self.halfTime = 0.0
		self.startRunTime = 0.0
		self.checkRunTime = 0.0

		#############################################################################################################################################
		################################################### Servo/Config. Parameters ################################################################
		#############################################################################################################################################
		# Control table address
		self.ADDR_PRO_MODEL_NUMBER       		 = 0
		self.ADDR_PRO_DRIVE_MODE         		 = 10
		self.ADDR_PRO_OPERATING_MODE     		 = 11
		self.ADDR_PRO_CURRENT_LIMIT      		 = 38
		self.ADDR_PRO_ACCELERATION_LIMIT 		 = 40
		self.ADDR_PRO_VELOCITY_LIMIT     		 = 44
		self.ADDR_PRO_TORQUE_ENABLE      		 = 64               # Control table address is different in Dynamixel model
		self.ADDR_PRO_POSITION_D_GAIN    		 = 80
		self.ADDR_PRO_POSITION_I_GAIN    		 = 82
		self.ADDR_PRO_POSITION_P_GAIN    		 = 84
		self.ADDR_PRO_FEEDFORWARD_2nd_GAIN		 = 88
		self.ADDR_PRO_FEEDFORWARD_1st_GAIN 		 = 90
		self.ADDR_PRO_GOAL_CURRENT       		 = 102
		self.ADDR_PRO_GOAL_VELOCITY      		 = 104
		self.ADDR_PRO_PROFILE_ACCELERATION  	 = 108		# VELOCITY BASED PROFILE
		self.ADDR_PRO_PROFILE_VELOCITY   		 = 112		# VELOCITY BASED PROFILE
		self.ADDR_PRO_PROFILE_ACCELERATION_TIME  = 108		# TIME BASED PROFILE
		self.ADDR_PRO_PROFILE_TIME_SPAN			 = 112      # TIME BASED PROFILE
		self.ADDR_PRO_GOAL_POSITION      		 = 116
		self.ADDR_PRO_MOVING             		 = 122
		self.ADDR_PRO_MOVING_STATUS       		 = 123
		self.ADDR_PRO_PRESENT_CURRENT    		 = 126 
		self.ADDR_PRO_PRESENT_POSITION   		 = 132

		# Data Byte Length
		self.LEN_PRO_GOAL_POSITION       	     = 4
		self.LEN_PRO_PRESENT_POSITION            = 4
		self.LEN_PRO_GOAL_CURRENT				 = 2
		self.LEN_PRO_PRESENT_CURRENT             = 2
		self.LEN_PRO_POS_TIME                    = 12		

		# Operating Mode Number
		self.CURRENT_CONTROL                     = 0
		self.POSITION_CONTROL                    = 3 # Default
		self.CURRENT_BASED_POSITION_CONTROL      = 5

		self.TIME_BASED                          = 4
		self.VELOCITY_BASED                      = 0

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# ID
		self.DXL1_ID                      = 1                          
		self.DXL2_ID                      = 2                             
		self.DXL3_ID                      = 3                            
		self.DXL4_ID                      = 4
		self.BAUDRATE                    = 57600             # Dynamixel default self.BAUDRATE : 57600
		self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
		                                                	 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		#############################################################################################################################################
		####################################################### Port Initialization #################################################################
		#############################################################################################################################################
		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		#exit()
		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		# Set port BAUDRATE
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the BAUDRATE")
		else:
			print("Failed to change the BAUDRATE")
			print("Press any key to terminate...")
			getch()
			quit()

		# Initialize GroupSyncWrite instance for Goal Position
		self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION)

		# Initialize GroupSyncRead instace for Present Position
		self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

		# Initialize GroupSyncWrite instance for Goal Current
		self.groupSyncWriteCurrent = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_CURRENT, self.LEN_PRO_GOAL_CURRENT)

		# Initialize GroupSyncRead instace for Present Current
		self.groupSyncReadCurrent = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

		# Initialize GroupSyncRWrite instace for Time-based profile 
		self.groupSyncWritePositionInTime = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_PROFILE_ACCELERATION, self.LEN_PRO_POS_TIME)


		# Add parameter storage for Dynamixel#1 present position value
		self.dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL1_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL1_ID)
			quit()

		# Add parameter storage for Dynamixel#2 present position value
		self.dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL2_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL2_ID)
			quit()

		# Add parameter storage for Dynamixel#3 present position value
		self.dxl_addparam_result = self.groupSyncReadPosition.addParam(self.DXL3_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL3_ID)
			quit()


		# Add parameter storage for Dynamixel#1 present current value
		self.dxl_addparam_result = self.groupSyncReadCurrent.addParam(self.DXL1_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL1_ID)
			#quit()

		# Add parameter storage for Dynamixel#2 present current value
		self.dxl_addparam_result = self.groupSyncReadCurrent.addParam(self.DXL2_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL2_ID)
			#quit()

		# Add parameter storage for Dynamixel#3 present current value
		self.dxl_addparam_result = self.groupSyncReadCurrent.addParam(self.DXL3_ID)
		if self.dxl_addparam_result != True:
			print("Initialize: ERROR")
			print("[ID:%03d] groupSyncRead addparam failed" % self.DXL3_ID)
			#quit()

		#############################################################################################################################################
		################################################### Mode and Operation Setup ################################################################
		#############################################################################################################################################

		# It can be POSITION_CONTROL or CURRENT_BASED_POSITION_CONTROL
		if not self.GetOperatingMode(self.DXL1_ID) == 3:
			self.SetOperatingMode(self.DXL1_ID,self.POSITION_CONTROL)
			self.SetOperatingMode(self.DXL2_ID,self.POSITION_CONTROL)
			self.SetOperatingMode(self.DXL3_ID,self.POSITION_CONTROL)
		#self.SetOperatingMode(self.CURRENT_BASED_POSITION_CONTROL)
		# For Gripper, it is always CURRENT_BASED_POSITION_CONTROL, and already set, so doesn't need to be set again
		self.SetDrivingMode(self.DXL1_ID,self.TIME_BASED)
		self.SetDrivingMode(self.DXL2_ID,self.TIME_BASED)
		self.SetDrivingMode(self.DXL3_ID,self.TIME_BASED)
		# Goal Current Set ###
		self.SetGoalCurrentGripper(self.GRIPPER_CURRENT)

		self.SetTimeBaseProfile(self.DXL4_ID,self.GRIPPER_Tf,self.GRIPPER_Ta)   #(ID,Tf,Ta) 

		if ( (self.GetOperatingMode(self.DXL1_ID)==3) and (self.GetOperatingMode(self.DXL2_ID)==3) and (self.GetOperatingMode(self.DXL3_ID)==3) ):

			#### Normal use for any purpose ####

			# Set PID gain 
			P_Gain1 = 4000    #800 default
			I_Gain1 = 0     #0 default
			D_Gain1 = 4000   #4700 default

			P_Gain2 = 4000    #800 default
			I_Gain2 = 0     #0 default
			D_Gain2 = 4000   #4700 default

			P_Gain3 = 4000    #800 default
			I_Gain3 = 0     #0 default
			D_Gain3 = 4000   #4700 default

			P_Gain4 = 800    #800 default
			I_Gain4 = 0     #0 default
			D_Gain4 = 2000   #4700 default

			# Feedforward Set
			FF1_Gain1 = 100
			FF2_Gain1 = 50

			FF1_Gain2 = 100
			FF2_Gain2 = 50

			FF1_Gain3 = 100
			FF2_Gain3 = 50

		elif  ( (self.GetOperatingMode(self.DXL1_ID)==5) and (self.GetOperatingMode(self.DXL2_ID)==5) and (self.GetOperatingMode(self.DXL3_ID)==5) ):
			
			#### This is for dynamics model only #####

			self.SetCurrentLimit(self.DXL1_ID,1000)
			self.SetCurrentLimit(self.DXL2_ID,1000)
			self.SetCurrentLimit(self.DXL3_ID,1000)

			# Goal current shouldn't be too low or too high, the robot must have enough torque to drive itself and take some load, but not too high to damage.
			GoalCur1 = 500
			GoalCur2 = 500
			GoalCur3 = 500

			self.SetGoalCurrent(self.DXL1_ID,GoalCur1)
			self.SetGoalCurrent(self.DXL1_ID,GoalCur2)
			self.SetGoalCurrent(self.DXL1_ID,GoalCur3)

			# Set PID gain 
			P_Gain1 = 1200    #800 default
			I_Gain1 = 0     #0 default
			D_Gain1 = 5000   #4700 default

			P_Gain2 = 1200    #800 default
			I_Gain2 = 0     #0 default
			D_Gain2 = 5000   #4700 default

			P_Gain3 = 1200    #800 default
			I_Gain3 = 0     #0 default
			D_Gain3 = 5000   #4700 default

			# Feedforward Set
			FF1_Gain1 = 100
			FF2_Gain1 = 50

			FF1_Gain2 = 100
			FF2_Gain2 = 50

			FF1_Gain3 = 100
			FF2_Gain3 = 50

		else:
			print("Initialiaztion ERROR: some motor is not in the same mode as others")

		# Set PID gain 
		self.SetPID(self.DXL1_ID, P_Gain1, I_Gain1, D_Gain1)
		self.SetPID(self.DXL2_ID, P_Gain2, I_Gain2, D_Gain2)
		self.SetPID(self.DXL3_ID, P_Gain3, I_Gain3, D_Gain3)
		self.SetPID(self.DXL4_ID, P_Gain4, I_Gain4, D_Gain4)

		self.SetFFGain(self.DXL1_ID, FF1_Gain1, FF2_Gain1)
		self.SetFFGain(self.DXL2_ID, FF1_Gain2, FF2_Gain2)
		self.SetFFGain(self.DXL3_ID, FF1_Gain3, FF2_Gain3)


	def map(self, val, in_min, in_max, out_min, out_max):
                                                                          
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def ReadModelNumber(self,ID):
		# MX106 -> 321
		# XM540 -> 1120
		# XM430 -> 1020
		model_number, dxl_comm_result, dxl_error = self.self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MODEL_NUMBER)

		if model_number == 321:
			print("MX106 is on Servo ID%d" %ID)
		elif model_number == 1020:
			print("XM430 is on Servo ID%d" %ID)
		elif model_number == 1120:
			print("XM540 is on Servo ID%d" %ID)
		else:
			print("Unknown model...")

		return model_number

	def SetDrivingMode(self,ID,Base):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_DRIVE_MODE, Base)

		self.DriveMode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_DRIVE_MODE)

		if self.DriveMode == 4:
			print("Motor " + str(ID) + " is in Driving Mode : Time-Based Profile")
		elif self.DriveMode == 0:
			print("Motor " + str(ID) + " is in Driving Mode : Velocity-Based Profile")
		else:
			print("Motor " + str(ID) + " is in Driving Mode : Unknown Drive Mode...")

	def GetOperatingMode(self,ID):
		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)

		return present_mode

	def SetOperatingMode(self,ID,MODE):

		## Must set to torque off before 

		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE, MODE)

		present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_OPERATING_MODE)
		if present_mode == 0:
		    # Current (Torque) Control Mode
			print("Motor " + str(ID) + " Operating Mode is Torque Control")
		elif present_mode == 3:
		    # Position Control Mode
			print("Motor " + str(ID) + " Operating Mode is Position Control")
		elif present_mode == 5:
		    # Current-based Position Control Mode
			print("Motor " + str(ID) + " Operating Mode is Current-based Position Control")
		else:
			print("In other Mode that didn't set!")

	def SetCurrentLimit(self,ID,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 6.8779]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 2047], [Range(ampere) : 0 ~ 5.5064]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : 0 ~ 1193], [Range(ampere) : 0 ~ 3.2092]

		# UnitMX106 = 3.36 
		# UnitXM540 = 2.69
		# UnitXM430 = 2.69

		# Write ComValue
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_CURRENT_LIMIT,ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		# Read and confirm input value
		dxl_current_limit, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_CURRENT_LIMIT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		#CurLimit = dxl_current_limit*(SetUnit/1000.0)
		print("Current Limit of ID " + str(ID) + " [ComValue]: " + str(dxl_current_limit))
		#print("Current Limit 1 [Ampere] : %f" %CurLimit)

	def SetGoalCurrent(self,ID,ComValue):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]

		# user input is in ComValue unit

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_GOAL_CURRENT, ComValue)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def SetGoalCurrentGripper(self,SetCur):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_CURRENT, SetCur)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Goal Current of Gripper is " + str(SetCur))

	def ReadAllCurrent(self):
		# Syncread present current
		dxl_comm_result = self.groupSyncReadCurrent.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = self.groupSyncReadCurrent.isAvailable(self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)
		if dxl_getdata_result != True:
			print("ReadAllCurrent: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL1_ID)
			#quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupSyncReadCurrent.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)
		if dxl_getdata_result != True:
			print("ReadAllCurrent: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL2_ID)
			#quit()

		# Check if groupsyncread data of Dynamixel#3 is available
		dxl_getdata_result = self.groupSyncReadCurrent.isAvailable(self.DXL3_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)
		if dxl_getdata_result != True:
			print("ReadAllCurrent: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL3_ID)
			#quit()

		# Get Dynamixel#1 present position value
		dxl_present_current1 = self.groupSyncReadCurrent.getData(self.DXL1_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

		# Get Dynamixel#2 present position value
		dxl_present_current2 = self.groupSyncReadCurrent.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

		# Get Dynamixel#3 present position value
		dxl_present_current3 = self.groupSyncReadCurrent.getData(self.DXL3_ID, self.ADDR_PRO_PRESENT_CURRENT, self.LEN_PRO_PRESENT_CURRENT)

		com_signed_value1 = np.int16(dxl_present_current1)
		com_signed_value2 = np.int16(dxl_present_current2)
		com_signed_value3 = np.int16(dxl_present_current3)

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		print("Present current1 [ComValue_signed]: %f" %com_signed_value1)
		print("Present current2 [ComValue_signed]: %f" %com_signed_value2)
		print("Present current3 [ComValue_signed]: %f" %com_signed_value3)

		return com_signed_value1, com_signed_value2, com_signed_value3

	def ReadCurrent(self,ID):

		# Each model has difference range and unit
		# MX106 -> [Unit : 3.36mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM540 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		# XM430 -> [Unit : 2.69mA], [Range(ComValue) : -SetCurLimit ~ SetCurLimit]
		'''
		UnitMX106 = 3.36 
		UnitXM540 = 2.69
		UnitXM430 = 2.69
		'''
		dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PRESENT_CURRENT)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		com_signed_value = np.int16(dxl_present_current)

		#PresentCur = com_signed_value*(SetUnit/1000.0)
		# print("ReadCurrent of ID " + str(ID) + " [ComValue_signed]: " + str(com_signed_value))
		#print("Present current1 [Ampere]: %f" %PresentCur)

		return com_signed_value

	def FWD(self,deg1, deg2, deg3):

		deg1 = (deg1 + self.OFFSET_ANGLE)*self.deg2rad
		deg2 = (deg2 + self.OFFSET_ANGLE)*self.deg2rad
		deg3 = (deg3 + self.OFFSET_ANGLE)*self.deg2rad

		A1v = np.array([0, -self.wb - self.L*np.cos(deg1) + self.up, -self.L*np.sin(deg1)])
		A2v = np.array([(self.root3/2)*(self.wb + self.L*np.cos(deg2)) - self.sp/2, 0.5*(self.wb+ self.L*np.cos(deg2)) - self.wp, -self.L*np.sin(deg2)])
		A3v = np.array([(-self.root3/2)*(self.wb + self.L*np.cos(deg3)) + self.sp/2, 0.5*(self.wb+ self.L*np.cos(deg3)) - self.wp, -self.L*np.sin(deg3)])

		r1 = self.l
		r2 = self.l
		r3 = self.l

		x1 = A1v[0]
		y1 = A1v[1]
		z1 = A1v[2]

		x2 = A2v[0]
		y2 = A2v[1]
		z2 = A2v[2]

		x3 = A3v[0]
		y3 = A3v[1]
		z3 = A3v[2]

		#Select the method to calculate
		#Depends on the height of virtual spheres center
		#Method 1 is used when the height of z1 z2 z3 are equal
		#Method 2, 3, 4 are trying to avoid 0 division at a13 and a23

		if ((z1==z2) and (z2==z3) and (z1==z3)):
			method = 1
		elif ((z1 != z3) and (z2 != z3)):
			method = 2
		elif ((z1 != z2) and (z1 != z3)):
			method = 3
		else:
			method = 4

		if method == 1:
			zn = z1  # z1 = z2 = z3 = zn

			a = 2*(x3 - x1)
			b = 2*(y3 - y1)
			c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
			d = 2*(x3 - x2)
			e = 2*(y3 - y2)
			f = r2**2 - r3**2 - x2**2 -y2**2 + x3**2 + y3**2

			numX = c*e - b*f
			denX = a*e - b*d
			x = numX/denX
			if x < 0.000001:
				x = 0

			numY = a*f - c*d
			denY = a*e - b*d
			y = numY/denY
			if y < 0.000001:
				y = 0

			A = 1
			B = -2*zn
			C = zn**2 - r1**2 + (x-x1)**2 + (y-y1)**2

			z = [None]*2

			z[0] = (-B + np.sqrt(B**2 - 4*C))/2;
			z[1] = (-B - np.sqrt(B**2 - 4*C))/2;

			realANS = [None]*3

			if z[0] < 0: 
				realANS = np.array([x,y,z[0]])
			elif z[1] < 0:
				realANS = np.array([x,y,z[1]])
			else:
				showError = "FWD ERROR: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		elif method ==2:

			a11 = 2*(x3 - x1)
			a12 = 2*(y3 - y1)
			a13 = 2*(z3 - z1)

			a21 = 2*(x3 - x2)
			a22 = 2*(y3 - y2)
			a23 = 2*(z3 - z2)

			b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
			b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2;
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1);
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2;
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY)
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5;
			x[1] = a4*y[1] + a5;
			z[0] = a6*y[0] + a7;
			z[1] = a6*y[1] + a7;

			realANS = [None]*3

			if z[0] < 0:
				realANS = np.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = np.array([x[1],y[1],z[1]])
			else:
				showError = "FWD ERROR: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		elif method == 3:
			a11 = 2*(x1 - x2)
			a12 = 2*(y1 - y2)
			a13 = 2*(z1 - z2)

			a21 = 2*(x1 - x3)
			a22 = 2*(y1 - y3)
			a23 = 2*(z1 - z3)

			b1 = r2**2 - r1**2 - x2**2 - y2**2 - z2**2 + x1**2 + y1**2 + z1**2
			b2 = r3**2 - r1**2 - x3**2 - y3**2 - z3**2 + x1**2 + y1**2 + z1**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY);
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5
			x[1] = a4*y[1] + a5
			z[0] = a6*y[0] + a7
			z[1] = a6*y[1] + a7

			realANS = [None]*3

			if z[0] < 0: 
				realANS = np.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = np.array([x[1],y[1],z[1]])
			else:
				showError = "FWD ERROR: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		if method == 4:
			a11 = 2*(x2 - x1)
			a12 = 2*(y2 - y1)
			a13 = 2*(z2 - z1)

			a21 = 2*(x2 - x3)
			a22 = 2*(y2 - y3)
			a23 = 2*(z2 - z3)

			b1 = r1**2 - r2**2 - x1**2 - y1**2 - z1**2 + x2**2 + y2**2 + z2**2
			b2 = r3**2 - r2**2 - x3**2 - y3**2 - z3**2 + x2**2 + y2**2 + z2**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY);
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5
			x[1] = a4*y[1] + a5
			z[0] = a6*y[0] + a7
			z[1] = a6*y[1] + a7

			realANS = [None]*3

			if z[0] < 0:
				realANS = np.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = np.array([x[1],y[1],z[1]])
			else:
				showError = "FWD ERROR: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		return realANS

	def INV(self,x,y,z):

		workingR = np.sqrt(x**2 + y**2)

		a = self.wb - self.up
		b = self.sp/2 - (self.root3/2)*self.wb
		c = self.wp - 0.5*self.wb

		E1 = 2*self.L*(y + a)
		F1 = 2*z*self.L
		G1 = x**2 + y**2 + z**2 + a**2 + self.L**2 + 2*y*a - self.l**2

		E2 = -self.L*(self.root3*(x+b) + y + c)
		F2 = 2*z*self.L
		G2 = x**2 + y**2 + z**2 + b**2 + c**2 + self.L**2 + 2*(x*b + y*c) - self.l**2

		E3 = self.L*(self.root3*(x-b) - y - c)
		F3 = 2*z*self.L
		G3 = x**2 + y**2 + z**2 + b**2 + c**2 + self.L**2 + 2*(-x*b + y*c) - self.l**2;

		t1 = [None]*2
		t2 = [None]*2
		t3 = [None]*2

		t1[0] = (-F1 + np.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
		t1[1] = (-F1 - np.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
		t2[0] = (-F2 + np.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
		t2[1] = (-F2 - np.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
		t3[0] = (-F3 + np.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)
		t3[1] = (-F3 - np.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)

		theta1_1 = 2*np.arctan(t1[1]);
		theta2_1 = 2*np.arctan(t2[1]);
		theta3_1 = 2*np.arctan(t3[1]);

		deg1 = theta1_1*self.rad2deg;
		deg2 = theta2_1*self.rad2deg;
		deg3 = theta3_1*self.rad2deg;


		if isinstance(theta1_1, complex) or isinstance(theta2_1, complex) or isinstance(theta3_1, complex):
			print("INV Error: Driving angle is complex number")
			return [None]

		if workingR <= self.maxR and z > self.maxStoke and z <= self.minStoke :
			deg1 = deg1 - self.OFFSET_ANGLE
			deg2 = deg2 - self.OFFSET_ANGLE
			deg3 = deg3 - self.OFFSET_ANGLE
			#print("INV_deg1:%f" %deg1)
			#print("INV_deg2:%f" %deg2)
			#print("INV_deg3:%f" %deg3)
			return deg1, deg2, deg3
		else:
			PreAng = self.ReadAngle()
			#PreAng = self.ReadAngleNormal()
			deg1 = PreAng[0]
			deg2 = PreAng[1]
			deg3 = PreAng[2]
			#print("INV_deg1:%f" %deg1)
			#print("INV_deg2:%f" %deg2)
			#print("INV_deg3:%f" %deg3)
			print("INV ERROR: Out of working range!")
			return deg1, deg2, deg3

	def XYZOutRange(self,x,y,z):

		workingR = np.sqrt(x**2 + y**2)
		maxZ = 0.75*workingR - 1225.0

		if((workingR > self.maxR) or (z < self.maxStoke) or (z > self.minStoke) or (z < maxZ)):

			print("XYZOutRange ERROR: x or y or z value is out of range")
			WarningFlag = True

		else:
			WarningFlag = False

		return WarningFlag

	def RunServo(self,ServoDeg1,ServoDeg2,ServoDeg3):

		#ServoDeg1 = 90 + ServoDeg1  # +90 to compensate the mechanical offset setting
		#ServoDeg2 = 90 + ServoDeg2
		#ServoDeg3 = 90 + ServoDeg3

		servo_ang1 = self.map(ServoDeg1, 0.0, 360.0, 0, 4095)
		servo_ang2 = self.map(ServoDeg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(ServoDeg3, 0.0, 360.0, 0, 4095)

		dxl1_goal_position = int(servo_ang1)
		dxl2_goal_position = int(servo_ang2)
		dxl3_goal_position = int(servo_ang3)

		##### Delta Robot Safety Working Range #####

		param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]
		param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]
		param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

		# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePosition.addParam(self.DXL1_ID, param_goal_position1)
		if dxl_addparam_result != True:
			print("RunServo: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL1_ID)
			self.portHandler.closePort()
			quit()

		# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePosition.addParam(self.DXL2_ID, param_goal_position2)
		if dxl_addparam_result != True:
			print("RunServo: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL2_ID)
			self.portHandler.closePort()
			quit()

		# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePosition.addParam(self.DXL3_ID, param_goal_position3)
		if dxl_addparam_result != True:
			print("RunServo: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL3_ID)
			self.portHandler.closePort()
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWritePosition.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServo: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWritePosition.clearParam()

	def RunServoInTime(self,inputDeg1,t1_1,t3_1,inputDeg2,t1_2,t3_2,inputDeg3,t1_3,t3_3):

		#inputDeg1 = inputDeg1 + 90.0
		#inputDeg2 = inputDeg2 + 90.0
		#inputDeg3 = inputDeg3 + 90.0

		servo_ang1 = self.map(inputDeg1, 0.0, 360.0, 0, 4095)
		servo_ang2 = self.map(inputDeg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(inputDeg3, 0.0, 360.0, 0, 4095)
		dxl1_goal_position = int(servo_ang1)
		dxl2_goal_position = int(servo_ang2)
		dxl3_goal_position = int(servo_ang3)
		t1_1 = int(t1_1)
		t3_1 = int(t3_1)
		t1_2 = int(t1_2)
		t3_2 = int(t3_2)
		t1_3 = int(t1_3)
		t3_3 = int(t3_3)

		time_position1 = [DXL_LOBYTE(DXL_LOWORD(t1_1)),
						DXL_HIBYTE(DXL_LOWORD(t1_1)),
						DXL_LOBYTE(DXL_HIWORD(t1_1)), 
						DXL_HIBYTE(DXL_HIWORD(t1_1)),
						DXL_LOBYTE(DXL_LOWORD(t3_1)), 
						DXL_HIBYTE(DXL_LOWORD(t3_1)),
						DXL_LOBYTE(DXL_HIWORD(t3_1)), 
						DXL_HIBYTE(DXL_HIWORD(t3_1)),
						DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), 
						DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
						DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), 
						DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]

		time_position2 = [DXL_LOBYTE(DXL_LOWORD(t1_2)), 
						DXL_HIBYTE(DXL_LOWORD(t1_2)),
						DXL_LOBYTE(DXL_HIWORD(t1_2)), 
						DXL_HIBYTE(DXL_HIWORD(t1_2)),
						DXL_LOBYTE(DXL_LOWORD(t3_2)), 
						DXL_HIBYTE(DXL_LOWORD(t3_2)),
						DXL_LOBYTE(DXL_HIWORD(t3_2)), 
						DXL_HIBYTE(DXL_HIWORD(t3_2)),
						DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), 
						DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
						DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), 
						DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]

		time_position3 = [DXL_LOBYTE(DXL_LOWORD(t1_3)), 
						DXL_HIBYTE(DXL_LOWORD(t1_3)),
						DXL_LOBYTE(DXL_HIWORD(t1_3)), 
						DXL_HIBYTE(DXL_HIWORD(t1_3)),
						DXL_LOBYTE(DXL_LOWORD(t3_3)), 
						DXL_HIBYTE(DXL_LOWORD(t3_3)),
						DXL_LOBYTE(DXL_HIWORD(t3_3)), 
						DXL_HIBYTE(DXL_HIWORD(t3_3)),
						DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), 
						DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)),
						DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), 
						DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

		# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionInTime.addParam(self.DXL1_ID, time_position1)
		if dxl_addparam_result != True:
			print("RunServoInTime: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL1_ID)
			self.portHandler.closePort()
			quit()

		# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionInTime.addParam(self.DXL2_ID, time_position2)
		if dxl_addparam_result != True:
			print("RunServoInTime: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL2_ID)
			self.portHandler.closePort()
			quit()
		# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
		dxl_addparam_result = self.groupSyncWritePositionInTime.addParam(self.DXL3_ID, time_position3)
		if dxl_addparam_result != True:
			print("RunServoInTime: ERROR")
			print("[ID:%03d] groupSyncWrite addparam failed" %self.DXL3_ID)
			self.portHandler.closePort()
			quit()

		# Syncwrite goal position
		dxl_comm_result = self.groupSyncWritePositionInTime.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServoInTime: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWritePositionInTime.clearParam()

	def ReadAngle(self):

		# Syncread present position
		dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("ReadAngle: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL1_ID)
			#quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("ReadAngle: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL2_ID)
			#quit()

		# Check if groupsyncread data of Dynamixel#2 is available
		dxl_getdata_result = self.groupSyncReadPosition.isAvailable(self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		if dxl_getdata_result != True:
			print("ReadAngle: ERROR")
			print("[ID:%03d] groupSyncRead getdata failed" % self.DXL3_ID)
			#quit()

		# Get Dynamixel#1 present position value
		dxl_present_position1 = self.groupSyncReadPosition.getData(self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# Get Dynamixel#2 present position value
		dxl_present_position2 = self.groupSyncReadPosition.getData(self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		# Get Dynamixel#3 present position value
		dxl_present_position3 = self.groupSyncReadPosition.getData(self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)

		dxl_present_position1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngle: ERROR")
			print("ID1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()

		elif dxl_error != 0:
			print("ReadAngle: ERROR")
			print("ID1 %s" % self.packetHandler.getRxPacketError(dxl_error))
			#quit()
		
		dxl_present_position2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngle: ERROR")
			print("ID2 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()
		elif dxl_error != 0:
			print("ReadAngle: ERROR")
			print("ID2 %s" % self.packetHandler.getRxPacketError(dxl_error))
		
		dxl_present_position3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngle: ERROR")
			print("ID3 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()
		elif dxl_error != 0:
			print("ReadAngle: ERROR")
			print("ID3 %s" % self.packetHandler.getRxPacketError(dxl_error))

		deg1 = self.map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
		deg2 = self.map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
		deg3 = self.map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

		#deg1 = deg1 - 90
		#deg2 = deg2 - 90
		#deg3 = deg3 - 90

		return deg1, deg2, deg3

	def ReadAngleNormal(self):
		dxl_present_position1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngleNormal: ERROR")
			print("ID1 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()
		elif dxl_error != 0:
			print("ReadAngleNormal: ERROR")
			print("ID1 %s" % self.packetHandler.getRxPacketError(dxl_error))
			#quit()

		dxl_present_position2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngleNormal: ERROR")
			print("ID2 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()
		elif dxl_error != 0:
			print("ReadAngleNormal: ERROR")
			print("ID2 %s" % self.packetHandler.getRxPacketError(dxl_error))

		dxl_present_position3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadAngleNormal: ERROR")
			print("ID3 %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			self.portHandler.closePort()
			quit()
		elif dxl_error != 0:
			print("ReadAngleNormal: ERROR")
			print("ID3 %s" % self.packetHandler.getRxPacketError(dxl_error))

		deg1 = self.map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
		deg2 = self.map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
		deg3 = self.map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

		return deg1, deg2, deg3


	def GripperCheck(self):
		self.GripperOpen()
		time.sleep(1)
		self.GripperClose()
		time.sleep(1)
		self.GripperOpen()
		time.sleep(1)

	def ReadGripperPos(self):
		# Read and confirm input value
		dxl_present_position4, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_PRESENT_POSITION)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		#print("Gripper pose ",dxl_present_position4)
		return dxl_present_position4

	def ReadGripperCurrent(self):
		current = self.ReadCurrent(self.DXL4_ID)

		return current

	def objectOnGripper(self):
		## In case of just 3D printed fingers 			- > 1935
		## In case of 3D printed fingers + sponge foam  - > 1945
		if (self.ReadGripperPos() > 1955) or (self.ReadGripperCurrent() < self.minObjectExisted):
			return True
		else:
			return False

	def GripperClose(self):
		#servo_com4 = 3600 # Old Gripper
		servo_com4 = self.GRIPPER_CLOSE
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		if dxl_comm_result4 != COMM_SUCCESS:
			print("GripperClose: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result4))
			self.portHandler.closePort()
			quit()
		elif dxl_error4 != 0:
			print("GripperClose: ERROR")
			print("%s" % self.packetHandler.getRxPacketError(dxl_error4))
			self.portHandler.closePort()
			quit()
		#self.IsGripperStop()
		#print("Gripper closed")

	def GripperClose75(self):
		servo_com4 = self.GRIPPER_CLOSE*0.75 
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		print("Gripper closed 75%")

	def GripperOpen(self):
		#servo_com4 = 1500 # Old gripper
		servo_com4 = self.GRIPPER_OPEN
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		if dxl_comm_result4 != COMM_SUCCESS:
			print("GripperOpen: ERROR")
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result4))
			self.portHandler.closePort()
			quit()
		elif dxl_error4 != 0:
			print("GripperOpen: ERROR")
			print("%s" % self.packetHandler.getRxPacketError(dxl_error4))
			self.portHandler.closePort()
			quit()
		#self.IsGripperStop()
		#print("Gripper Opened")

	def GripperOpen75(self):
		#servo_com4 = 1500 # Old gripper
		servo_com4 = self.GRIPPER_OPEN_75
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)

	def TorqueOn(self,ID):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is enable")

	def TorqueOff(self,ID):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is disable")

	def RobotTorqueOn(self):
		self.TorqueOn(self.DXL1_ID)
		self.TorqueOn(self.DXL2_ID)
		self.TorqueOn(self.DXL3_ID)

	def RobotTorqueOff(self):
		self.TorqueOff(self.DXL1_ID)
		self.TorqueOff(self.DXL2_ID)
		self.TorqueOff(self.DXL3_ID)

	def GripperTorqueOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

	def GripperTorqueOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

	def IsMoving(self,ID):
		Moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MOVING)
		# print("ID %d  %s" %(ID, Moving))
		return Moving

	def MovingStatus(self,ID):
		MovingStat, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat > 48:
			print("Motor " + str(ID) + " is in Trapezodal Profile")
		elif MovingStat < 35 and MovingStat > 20:
			print("Motor " + str(ID) + " is in Triangular Profile")
		elif MovingStat < 20 and MovingStat > 3:
			print("Motor " + str(ID) + " is in Rectangular Profile")
		elif MovingStat < 3:
			print("Motor " + str(ID) + " is in Step Mode (No Profile)")
		else:
			print("Motor " + str(ID) + " is an UNKNOWN Profile...")

		return MovingStat

	def DeltaPos(self,pre_pos,goal_pos):

		pre_pos_pul = self.map(pre_pos,0.0,360.0,0,4095)
		pre_pos_pul = int(pre_pos_pul)
		goal_pos_pul = self.map(goal_pos,0.0,360.0,0,4095)
		goal_pos_pul = int(goal_pos_pul)

		delta_pos = abs(goal_pos_pul - pre_pos_pul)

		return delta_pos

	def SetVelocityProfile(self,ID,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile in Velocity-Base ##############################
		if self.DriveMode == 0:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))
			print("V PRFL of Motor " + str(ID) + " : " + str(set_V_PRFL))
			print("A PRFL of Motor " + str(ID) + " : " + str(set_A_PRFL))
			print("--------------------------------")
		else:
			print("V PRFL ERROR : DriveMode is invalid")

	def SetTimeBaseProfile(self,ID,set_Tf,set_Ta):
		######################### Set Velocity / Acceleration Profile in Time-Base ##############################
		if self.DriveMode == 4:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_ACCELERATION_TIME, int(set_Ta))
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, self.ADDR_PRO_PROFILE_TIME_SPAN, int(set_Tf))
			print("Finish Time of Motor " + str(ID) + " : " + str(set_Tf))
			print("Acceleration time of Motor " + str(ID) + " : " + str(set_Ta))
			print("--------------------------------")
		else:
			print("TIME PRFL ERROR : DriveMode is invalid")

	def SetPID(self,ID,set_P_Gain,set_I_Gain,set_D_Gain):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		print("Position P Gain of ID" + str(ID) + ": " + str(set_P_Gain))
		print("Position I Gain of ID" + str(ID) + ": " + str(set_I_Gain))
		print("Position D Gain of ID" + str(ID) + ": " + str(set_D_Gain))
		print("------------------------------")

	def SetFFGain(self,ID,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		print("Feedforward 1st Gain of ID " + str(ID) + ": " + str(set_FF1_Gain))
		print("Feedforward 2nd Gain of ID " + str(ID) + ": " + str(set_FF2_Gain))
		print("------------------------------") 

	def GoHome(self):
		pos1 = 135.0
		pos2 = 135.0
		pos3 = 135.0
		
		t1 = 400
		t3 = 800

		self.RunServoInTime(pos1,t1,t3,pos2,t1,t3,pos3,t1,t3)
		#self.IsAllStop()


	def GoCircle(self,Radius,Height):
		### This function haven't tested with Delta Robot MK2
		zeta = [None]*360
		X = [None]*360
		Y1 = [None]*180
		Y2 = [None]*180
		Y = [None]*360
		DEG1 = [None]*360
		DEG2 = [None]*360
		DEG3 = [None]*360

		## Generate X,Y coordinate for circle
		for i in range(1,361):
		    zeta[i-1] = i*self.deg2rad
		    X[i-1] = Radius*np.cos(zeta[i-1])

		for i in range(1,(len(X)/2)+1):
		    Y1[i-1] = np.sqrt(Radius**2 - X[i-1]**2)

		j = 0
		for i in range((len(X)/2)+1,len(X)+1):
		    Y2[j] = -np.sqrt(Radius**2 - X[j]**2)
		    j = j+1

		Y = np.concatenate((Y1,Y2))

		## Generae servo angle data set according to X,Y coordinate
		i = 0
		increment = 5

		while(i<360):
			x = X[i]
			y = Y[i]
			z = Height   

			DEG = self.INV(x,y,z)
			DEG1[i] = DEG[0] + 180.0
			DEG2[i] = DEG[1] + 180.0
			DEG3[i] = DEG[2] + 180.0


			i = i + 1

		## Run servo in 359 points left
		K = 0
		while K < 360:

			self.RunServo(DEG1[K],DEG2[K],DEG3[K])

			# make a delay for starting point
			if K == 0:
				time.sleep(1.5)

			K = K + increment

		## Finish the whole circle
		self.RunServo(DEG1[359],DEG2[359],DEG3[359])

	def GoSquare(self, SideLength, Height):
		### This function haven't tested with Delta Robot MK2
		xLim = SideLength/2
		yLim = xLim

		step_divider = 20
		step = xLim/step_divider
		x = list()
		y = list()

		# first line quadrant 1,2
		i = xLim
		while i >= -xLim:
			x.append(i)
			y.append(yLim)
			# update
			i = i - step

		# second line quadrant 2,3
		j = yLim
		while j >= -yLim:
			x.append(-xLim)
			y.append(j)
			# update
			j = j - step

		# thrid line quadrant 3,4
		k = -xLim
		while k <= xLim:
			x.append(k)
			y.append(-yLim)
			# update
			k = k + step

		#fourth line quadrant 4,1
		l = -yLim
		while l <= yLim:
			x.append(xLim)
			y.append(l)
			# update
			l = l + step

		size = len(x)

		z = [Height]*size
		DEG1 = [None]*size
		DEG2 = [None]*size
		DEG3 = [None]*size
		m = 0
		# Pre calculate Inverse Kinematics
		for m in range(0,len(x)):
			DEG = self.INV(x[m],y[m],z[m])
			DEG1[m] = DEG[0]
			DEG2[m] = DEG[1]
			DEG3[m] = DEG[2]

		## Run servo
		for K in range(0,size):

			self.RunServo(DEG1[K],DEG2[K],DEG3[K])

			# make a delay for starting point
			if K == 0:
				time.sleep(1.5)

	def GetXYZ(self):
		ReadAng = self.ReadAngle()
		# ReadAng = self.ReadAngleNormal()
		robotAngle1 = ReadAng[0] - 180.0
		robotAngle2 = ReadAng[1] - 180.0
		robotAngle3 = ReadAng[2] - 180.0
		XYZ = self.FWD(robotAngle1,robotAngle2,robotAngle3)
		x = XYZ[0]
		y = XYZ[1]
		z = XYZ[2]

		return x,y,z, robotAngle1,robotAngle2, robotAngle3

	def GetRobotAngle(self):
		servoAngle = self.ReadAngle()
		robotAngle1 = servoAngle[0] - 180.0
		robotAngle2 = servoAngle[1] - 180.0
		robotAngle3 = servoAngle[2] - 180.0

		return robotAngle1, robotAngle2, robotAngle3

	def Stop(self):
		StopAng = self.ReadAngle()
		self.RunServo(StopAng[0],StopAng[1],StopAng[2])

	def GotoPoint(self,x,y,z):

		WARN = self.XYZOutRange(x,y,z)
		if not WARN:
			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]

			GoalPos = self.INV(x,y,z)
			GoalPos1 = GoalPos[0] + 180.0
			GoalPos2 = GoalPos[1] + 180.0
			GoalPos3 = GoalPos[2] + 180.0

			DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
			DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
			DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
			if DelPos1 == 0 and DelPos2 == 0 and DelPos3 == 0:
				return

			# Time Base Profile
    		# Finish time
			t3_1 = 1000
			t3_2 = 1000
			t3_3 = 1000
			t1_1 = t3_1/2
			t1_2 = t3_2/2
			t1_3 = t3_3/2
			self.RunServoInTime(GoalPos1,t1_1,t3_1,GoalPos2,t1_2,t3_2,GoalPos3,t1_2,t3_2)

			# Bypass stop motion
			self.IsAllStop()
			
		else:
			print("ERROR GotoPoint: Out of workspace")

	def GotoPointInTime(self,x,y,z,finishedTime, grip):

		#print("checkSameTarget")
		samePoint = self.checkSameTarget(x,y,z)
		#print("XYZOutRange")
		WARN = self.XYZOutRange(x,y,z)
		if not WARN:
			if not samePoint:

				GoalPos = self.INV(x, y, z)
				GoalPos1 = GoalPos[0] + 180.0
				GoalPos2 = GoalPos[1] + 180.0
				GoalPos3 = GoalPos[2] + 180.0

				## Update latest target that received
				self.targetX = x
				self.targetY = y
				self.targetZ = z

			
				# Time Base Profile
	    		# Finish time
				t3_1 = finishedTime
				t3_2 = finishedTime
				t3_3 = finishedTime
				t1_1 = t3_1/2
				t1_2 = t3_2/2
				t1_3 = t3_3/2
				self.halfTime = finishedTime/2000  # second
				#print("RunServoInTime")
				self.RunServoInTime(GoalPos1,t1_1,t3_1,GoalPos2,t1_2,t3_2,GoalPos3,t1_2,t3_2)
				self.startRunTime = time.time()
				#time.sleep(finishedTime/2000)

				return False, False# to let main loop knows that it doesn't reach target point

				# Bypass stop motion
				#self.IsAllStop()

			else:
				#print("GotoPointInTime")
				#print("reachTarget?")
				oneIsDone = not self.IsMoving(1)
				twoIsDone = not self.IsMoving(2)
				threeIsDone = not self.IsMoving(3)
				self.checkRunTime = time.time()
				period = (self.checkRunTime - self.startRunTime)
				#print("period ", period)
				if ((period > self.halfTime) and oneIsDone and twoIsDone and threeIsDone):
				#if (self.reachedTarget()):
					#print("GotoPointInTime: reachedTarget %f %f %f" %(self.targetX, self.targetY, self.targetZ))
					if grip:
						#print("GripperClose")
						self.GripperClose()
						# return True, True
						#print("objectOnGripper")
						if (self.objectOnGripper()):
							#print("GotoPointInTime: GripperClose done")
							return True, True
						else:
							## It missed grabbling the target, then move it slightly lower
							# self.GotoPointInTime(self.targetX , self.targetY , self.targetZ-2, finishedTime, grip=True)
							# print("adjust target Z ")
							# self.countMissedGrab += 1
							# if self.countMissedGrab == 3:
							# 	## if it missed too many times, then fuck it!!
							# 	print("Fine, skip this one")
							# 	self.countMissedGrab = 0
							# 	return True
							#print("Missied grab")
							return True, False
					else:
						#print("GripperOpen75")
						self.GripperOpen75()
						#print("open")
						return True, True
				else:
					return False, False

		else:
			print("ERROR GotoPointInTime: Out of workspace")
			return False, False

	def GotoPointInTimePause(self,x,y,z,finishedTime):

		WARN = self.XYZOutRange(x,y,z)
		if not WARN:
			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]

			GoalPos = self.INV(x,y,z)
			GoalPos1 = GoalPos[0] + 180.0
			GoalPos2 = GoalPos[1] + 180.0
			GoalPos3 = GoalPos[2] + 180.0

			DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
			DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
			DelPos3 = self.DeltaPos(PreAng3,GoalPos3)
			if DelPos1 == 0 and DelPos2 == 0 and DelPos3 == 0:
				return

			# Time Base Profile
    		# Finish time
			t3_1 = time
			t3_2 = time
			t3_3 = time
			t1_1 = t3_1/2
			t1_2 = t3_2/2
			t1_3 = t3_3/2
			self.RunServoInTime(GoalPos1,t1_1,t3_1,GoalPos2,t1_2,t3_2,GoalPos3,t1_2,t3_2)

			# Bypass stop motion
			self.IsAllStop()
			
		else:
			print("ERROR GotoPoint: Out of workspace")

	def checkSameTarget(self, x, y, z):
		if self.targetX == x and self.targetY == y and self.targetZ == z:
			
			# print("checkSameTarget:")
			# print("x", x)
			# print("y", y)
			# print("z", z)
			return True
		else:
			return False

	def reachedTarget(self):

		# Check the difference between latest target point and current point
		XYZ = self.GetXYZ()
		delX = abs(self.targetX - XYZ[0])
		delY = abs(self.targetY - XYZ[1])
		delZ = abs(self.targetZ - XYZ[2])
		# print("delX", delX)
		# print("delY", delY)
		# print("delZ", delZ)

		## 10mm -> 1cm tolerance to decide it reached target or not is kind of reasonable
		if (delX < 5.0) and (delY < 5.0) and (delZ < 5.0):
			
			return True
		else: 
			return False
	

	def IsAllStop(self):
		MovingFlag = True
		# This delay would make sure that the moving detection loop will not run too fast than actual motion
		time.sleep(0.4)
		while MovingFlag:
			Move1 = self.IsMoving(self.DXL1_ID)
			Move2 = self.IsMoving(self.DXL2_ID)
			Move3 = self.IsMoving(self.DXL3_ID)

			if Move1 == 0 and Move2 == 0 and Move3 == 0:
				MovingFlag = False
				#print("IsAllStop: All servo are stop")

	def IsBusy(self):
		Move1 = self.IsMoving(self.DXL1_ID)
		Move2 = self.IsMoving(self.DXL2_ID)
		Move3 = self.IsMoving(self.DXL3_ID)

		if Move1 or Move2 or Move3:
			self.BUSY = True
		else:
			self.BUSY = False

	def IsGripperStop(self):
		MovingFlag = True
		time.sleep(0.4)
		while MovingFlag:
			Move4 = self.IsMoving(self.DXL4_ID)

			if Move4 == 0:
				MovingFlag = False

	def KinematicsCheck(self):
		try:
			while True:

				XYZ = self.GetXYZ()
				print("===      Present XYZ from FWD      ===")
				print("X: %f" %XYZ[0])
				print("Y: %f" %XYZ[1])
				print("Z: %f" %XYZ[2])
				print("===          Robot Angle         ===")
				print("RobotAng1: %f" %XYZ[3])
				print("RobotAng2: %f" %XYZ[4])
				print("RobotAng3: %f" %XYZ[5])
				X = XYZ[0]
				Y = XYZ[1]
				Z = XYZ[2]
				RobotAng1 = XYZ[3]
				RobotAng2 = XYZ[4]
				RobotAng3 = XYZ[5]
				INVDEG = self.INV(X,Y,Z)
				print("===     Present Angle from INV     ===")
				print("Deg1: %f" %INVDEG[0])
				print("Deg2: %f" %INVDEG[1])
				print("Deg3: %f" %INVDEG[2])
				print("===  Error Angle from calculation  ===")
				errorANG1 = RobotAng1 - INVDEG[0]
				errorANG2 = RobotAng2 - INVDEG[1]
				errorANG3 = RobotAng3 - INVDEG[2]
				print("Error Deg1: %f" %errorANG1)
				print("Error Deg2: %f" %errorANG2)
				print("Error Deg3: %f" %errorANG3)
				print("-------------------------------------")
				time.sleep(0.5)
		except(KeyboardInterrupt, SystemExit):
			print("End program...")
'''
	def DeltaJogLinear(self):
		LinearIncrement = 50
		waitForStart = True
		print("----------------------------------------------------------------------------------")
		print("-----------------------Press Start Button to Jog Linear!--------------------------")
		print("----------------------------------------------------------------------------------")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				startJog = True
			time.sleep(0.1)

		time.sleep(1)
		print("Full Robot is ready to move")
		print("Delta Robot Jog Linear Started...")

		self.SetProfile1(60,15)
		self.SetProfile2(60,15)
		self.SetProfile3(60,15)

		while startJog:

			############ Receive Value From Joy Stick All The Time ###############
			Buttons = self.getButton()
			Z_Btn = Buttons[1] #B
			X_Btn = Buttons[2] #X
			Y_Btn = Buttons[3] #Y
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			Continue_Btn = Buttons[8] #Logiccool
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			########## Reset Jogging and Back to home position ##################

			if Continue_Btn == 1:
				self.DeltaGoHome()

			###################### Exit the program ###########################

			if Back_Btn == 1:
				self.DeltaGoHome()
				print("----------------------------------------------------------------------------------")
				print("-----------------------Exit the Delta Robot Linear Jog Mode-----------------------")
				print("----------------------------------------------------------------------------------")
				break

			if GripOpen_Btn == 1:
				self.GripperOpen()
				time.sleep(0.5)

			if GripClose_Btn == 1:
				self.GripperClose()
				time.sleep(0.5)


			################### Move in X direction #####################
			OnceTrig = True
			while X_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				X_Btn = Buttons[2] #X
				#JogDirX = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1
				Axes = self.getAxis()
				Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				#Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.FWD(PreDeg1,PreDeg2,PreDeg3)
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn


				if abs(Ax0) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.FWD(Deg1,Deg2,Deg3)
					ReadX = ReadXYZ[0]
					#ReadY = ReadXYZ[1]
					#ReadZ = ReadXYZ[2]
					Xcom = ReadX + (Ax0*LinearIncrement)
					Ycom = Yconst
					Zcom = Zconst
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.INV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

			################### Move in Y direction #####################
			OnceTrig = True
			while Y_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				Y_Btn = Buttons[3] #Y
				#JogDirYZ = Hats[1] # Normal = 0, DowDir Pressed = -1, UpDir Pressed = 1
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1 

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.FWD(PreDeg1,PreDeg2,PreDeg3)
					Xconst = PreReadXYZ[0]
					Zconst = PreReadXYZ[2]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn

				if abs(Ax1) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.FWD(Deg1,Deg2,Deg3)
					#ReadX = ReadXYZ[0]
					ReadY = ReadXYZ[1]
					#ReadZ = ReadXYZ[2]
					Xcom = Xconst
					Ycom = ReadY - (Ax1*LinearIncrement)
					Zcom = Zconst
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.INV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

			################### Move in Z direction #####################
			OnceTrig = True
			while Z_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				Z_Btn = Buttons[1] #B
				#JogDirYZ = Hats[1] # Normal = 0, DowDir Pressed = -1, UpDir Pressed = 1
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1 

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.FWD(PreDeg1,PreDeg2,PreDeg3)
					Xconst = PreReadXYZ[0]
					Yconst = PreReadXYZ[1]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn


				if abs(Ax1) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.FWD(Deg1,Deg2,Deg3)
					#ReadX = ReadXYZ[0]
					#ReadY = ReadXYZ[1]
					ReadZ = ReadXYZ[2]
					Xcom = Xconst
					Ycom = Yconst
					Zcom = ReadZ - (Ax1*LinearIncrement)
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.INV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

	def DeltaTeachPoint(self):

		self.TorqueOff()
		self.TorqueGripperOn()
		time.sleep(0.1)
		self.GripperCheck()

		################## Go to stand by position before starting  ###########################

		waitForStart = True
		print("Press Start Button!")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				startTeach = True

			time.sleep(0.1)

		print("--> You can move robot freely by your hand")
		print("--> Press Logicool button to memorize the position")
		print("--> Press Analog left button for OpenGripper")
		print("--> Press Analog right button for CloseGripper")

		Mem_Ang1 = [None]*10000
		Mem_Ang2 = [None]*10000
		Mem_Ang3 = [None]*10000
		Mem_GripperStatus = [None]*10000
		GripperStatus = 1 # For open at first

		i = 0
		J = 0
		runTeach = False

		while startTeach:

			Buttons = self.getButton()
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			Memo_Btn = Buttons[8] #Logiccool
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			ReadANG = self.ReadAngle()
			ANG1 = ReadANG[0]
			ANG2 = ReadANG[1]
			ANG3 = ReadANG[2]

			if GripOpen_Btn == 1:
				self.GripperOpen()
				GripperStatus = 1

			if GripClose_Btn == 1:
				self.GripperClose()
				GripperStatus = 0

			if Memo_Btn == 1:
				ReadANG = self.ReadAngle()
				Mem_Ang1[i] = ReadANG[0]
				Mem_Ang2[i] = ReadANG[1]
				Mem_Ang3[i] = ReadANG[2]
				Mem_GripperStatus[i] = GripperStatus

				print("------------------------------")
				print("------------------------------")
				print("Mem_Ang1: %f" %Mem_Ang1[i])
				print("Mem_Ang2: %f" %Mem_Ang2[i])
				print("Mem_Ang3: %f" %Mem_Ang3[i])
				if GripperStatus == 1:
					print("Gripper Open")
				elif GripperStatus == 0:
					print("Gripper Close")
				else:
					print("No Gripper status...")
				print("------------------------------")
				print("------------------------------")

				i = i + 1
				while Memo_Btn == 1:
					Buttons = self.getButton()
					Memo_Btn = Buttons[8] #Logiccool
					#print("Release Button!")
					#time.sleep(0.5)

				print("Teach Point: %d" %i)

			if Back_Btn == 1:
				for J in range(0,i):
					print("Point: %d" %J)
					print("Mem_Ang1 = %f" %Mem_Ang1[J])
					print("Mem_Ang2 = %f" %Mem_Ang2[J])
					print("Mem_Ang3 = %f" %Mem_Ang3[J])
					print("////////////////////////////")
				startTeach = False
				waitForStart = True

		print("Press Start Button to run Teaching Point")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				runTeach = True

			time.sleep(0.1)

		while runTeach:

			self.TorqueOn()
			print("All Torque is ON")
			time.sleep(1)
			self.DeltaGoHome()
			time.sleep(3)

			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]

			for K in range(0,i):
				startTime = time.time()
				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back

				if Back_Btn == 1:
			 		break

				GoalPos1 = Mem_Ang1[K]
				GoalPos2 = Mem_Ang2[K]
				GoalPos3 = Mem_Ang3[K]

				DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = self.DeltaPos(PreAng3,GoalPos3)

				VSTD = 40
				ASTD = 8

				TRAJ = self.TrajectoryGenerationDelta(VSTD,ASTD,DelPos1,DelPos2,DelPos3)

				V1 = TRAJ[0]
				A1 = TRAJ[1]
				V2 = TRAJ[2]
				A2 = TRAJ[3]
				V3 = TRAJ[4]
				A3 = TRAJ[5]

				self.SetProfile1(V1,A1)
				self.SetProfile2(V2,A2)
				self.SetProfile3(V3,A3)

				self.RunServo(GoalPos1,GoalPos2,GoalPos3)
				startTime = time.time()

				print("Move to point %d" %K )

				MoveType1 = self.MovingStatus1()
				MoveType2 = self.MovingStatus2()
				MoveType3 = self.MovingStatus3()

				MovingFlag = True
				time.sleep(0.5)

				while MovingFlag:
					Move1 = self.IsMoving1()
					Move2 = self.IsMoving2()
					Move3 = self.IsMoving3()

					if Move1 == 0:
						endTime1 = time.time()
						period1 = endTime1 - startTime
					if Move2 == 0:
						endTime2 = time.time()
						period2 = endTime2 - startTime  
					if Move3 == 0:
						endTime3 = time.time()
						period3 = endTime3 - startTime
					if Move1 == 0 and Move2 == 0 and Move3 == 0:
						MovingFlag = False
						#endTime = time.time()
						#period = endTime - startTime
						print("Period1: %f" %period1)
						print("Period2: %f" %period2) 
						print("Period3: %f" %period3)                              
						print("Finished point %d" %K)

				if Mem_GripperStatus[K] == 1:
					self.GripperOpen()
					time.sleep(0.5)
					print("Open Gripper")
				elif Mem_GripperStatus[K] == 0:
					self.GripperClose()
					time.sleep(0.5)
					print("Close Gripper")

				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3

			waitForStartAgain = True
			print("Press start for run again")
			print("Press back to exit")
			while waitForStartAgain:
				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back
				Start_Btn = Buttons[7] #Start
				if Back_Btn == 1:
					waitForStartAgain = False
					runTeach = False
					self.DeltaGoHome()
					#TorqueOff()

				if Start_Btn == 1:
					K = 0
					waitForStartAgain = False

'''					
