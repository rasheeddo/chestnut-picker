# Chestnut-picker
This is a sample code for chestnut picker robot, and it's still in development process, some of the script you have seen here may or maynot exist in the future. This will just give a guideline of how things work.

You wil need dynamixel_sdk on you environment or in the same folder as these scripts. So please install that before.

DeltaRobot.py is the delta robot class which was implemented with Dynamixel SDK, but the inverse and forward kinematics are pretty much the same as on your text book and there are a lot of resources explaining about this.

DR_GoPoint.py is for testing the robot movement to go to the points you specified before the loop.

DR_Kinematics.py is to check the moving plate coordinates (XYZ) and the servos current angle. There is no torque apply to the robot, so it will be useful when you want to know what is the XYZ of the robot at your interested point.

testPick_pydarknet.py is to test the robot picking movement when it received the data from another detection script (which I used pydarknet for this time). The robot is on static frame, there is no rover movement.

chestnut_picker_demo.py is to use with rover_control.py and pydarknet scripts. This is the main script to run the robot and make a decision to the rover whether to conitue going or stop.

rover_control.py is to comunicate with Ardupiulot, and this time it just does RC overrides on throttle channel with constant value.
