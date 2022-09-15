"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor
import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
#Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 770


# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.11 #To be filled in with ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = MAX_SPEED #Initialize variable for left speed
vR = MAX_SPEED #Initialize variable for right speed


states = ["forward", "right_turn", "left_turn", "u_turn"]
current_state = "forward"
    
    
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()
    
    #Now we process the sensor data 
    move_right = gsr[2] < GROUND_SENSOR_THRESHOLD
    move_left = gsr[0] < GROUND_SENSOR_THRESHOLD
    #line_forward = gsr[1] < GROUND_SENSOR_THRESHOLD


    # Hints: 
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #
    
    #Insert Line Following Code Here
    
    
    
    #check which state robot needs to be in
    if all(i <= 304 for i in gsr): #if all values are inside a black line (only place this is at is finish line)
        print("Arrived at the finish line!")
        current_state = "forward"
    elif move_left:
        current_state = "left_turn"
            
    elif move_right:
        current_state = "right_turn"
        
    elif all(i >= 830 for i in gsr): #if all values in the array are above value 830 (value of white board)
        current_state = "u_turn"
    else:
        current_state = "forward"
            
       
    #the robot will always start by moving forward
    if current_state.__eq__("forward"):
        #set motors to same speed, move forward
        vL = MAX_SPEED
        vR = MAX_SPEED
            
    if current_state.__eq__("right_turn"):
        
        #Stop right wheel and set power to left wheel to 50%
        #I choose 50% because it doesnt overshoot the turn
        vL = 0.5 * MAX_SPEED
        vR = 0

    if current_state.__eq__("left_turn"):
        #Stop left wheel and turn right wheel at 50%
        vL = 0
        vR = 0.5 * MAX_SPEED
        
    if current_state.__eq__("u_turn"):
        #rotate counterclockwise till we detect line
        vL = 0
        vR = 0.5 * MAX_SPEED     
    
    #We set the actual robot motor speeds down below
     
    print('Current state: '+ str(current_state)) #Uncomment to see the current state!
    print(gsr) #Uncomment to see the ground sensor values!
    
    
    
    
    # TODO: Call update_odometry Here
    
    # Hints:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second. 
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    

    
    # TODO: Insert Loop Closure Code Here
    
    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last 
    # for best results
    
    #print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)