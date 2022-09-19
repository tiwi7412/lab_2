"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor
import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 500 #anything under this is a black line

# These are your pose values that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 2
CENTER_IDX = 1
RIGHT_IDX = 0

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.129 # TODO: To be filled in with ePuck wheel speed in m/s
#start (0,0,0) at 0 secs
#end (0.624, -6.39e-05, 5.76e-05) at 4.832 sec
#distance traveled ~0.624
#speed = distance/time = 0.624/4.832 = 0.129
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(4): robot.step(SIM_TIMESTEP)  

vL = MAX_SPEED # TODO: Initialize variable for left speed
vR = MAX_SPEED # TODO: Initialize variable for right speed

class controller:
    def __init__(self):
        self.begin = 0
        self.finishline_steps = 0
        
    def what_to_do(self):
    
        self.begin+=1
        if self.begin < 100:
            if (self.begin) < 80:
                return right_turn()
            else: 
                return forward()
                 
        if all(i <= GROUND_SENSOR_THRESHOLD for i in gsr): #means that completely covering a line
            if self.finishline_steps >= 4:
                self.finishline_steps = 0
                return finishline()
            else:
                self.finishline_steps += 1
                return forward()
            
        elif all(i >= GROUND_SENSOR_THRESHOLD for i in gsr):
            return u_turn()
            
        elif gsr[0] < GROUND_SENSOR_THRESHOLD:
            return right_turn()
            
        elif gsr[2] < GROUND_SENSOR_THRESHOLD:
            return left_turn()
            
        else:
            return forward()
              

class finishline:
    def __init__(self):
        loop_closure()
        print("Arrived at the finish line!")
    def get_vals(self): 
        return (MAX_SPEED, MAX_SPEED)

class forward:
    def __init__(self):
        self.vL = MAX_SPEED
        self.vR = MAX_SPEED
    def get_vals(self): 
        return (self.vL, self.vR)
       
class right_turn:
    def __init__(self):
        self.vL = 0.5 * MAX_SPEED
        self.vR = 0
    def get_vals(self): 
        return (self.vL, self.vR)
        
class left_turn:
    def __init__(self):
        self.vL = 0
        self.vR = 0.5 * MAX_SPEED
    def get_vals(self): 
        return (self.vL, self.vR)
        
class u_turn:
    def __init__(self):
        self.vL = 0
        self.vR = 0.5 * MAX_SPEED
    def get_vals(self): 
        return (self.vL, self.vR)
        
my_controller = controller()
def loop_closure(): #use when the robot passes over finish line
    global pose_x, pose_y, pose_theta
    pose_x = -0.183
    pose_y = -0.447 
    pose_theta = -1.57 #-pi/2
    
def update_odometry():
    global pose_x, pose_y, pose_theta
    vL_precentage = vL/MAX_SPEED
    vR_precentage = vR/MAX_SPEED
    if vR_precentage == 0: #turning right
        distance = -vL_precentage * SIM_TIMESTEP / 1000 * EPUCK_MAX_WHEEL_SPEED
        theta = math.asin(distance/EPUCK_AXLE_DIAMETER)
        pose_theta += theta
        pose_y += abs(distance/2) * math.sin(pose_theta)
        pose_x += abs(distance/2) * math.cos(pose_theta)
    elif vL_precentage == 0: #turning left
        distance = vR_precentage * SIM_TIMESTEP / 1000 * EPUCK_MAX_WHEEL_SPEED
        theta = math.asin(distance/EPUCK_AXLE_DIAMETER)
        pose_theta += theta
        pose_y += abs(distance/2) * math.sin(pose_theta)
        pose_x += abs(distance/2) * math.cos(pose_theta)
    else: #straight
        distance = vR_precentage * SIM_TIMESTEP / 1000 * EPUCK_MAX_WHEEL_SPEED
        pose_y += abs(distance) * math.sin(pose_theta)
        pose_x += abs(distance) * math.cos(pose_theta)
        
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    #print(gsr) # TODO: Uncomment to see the ground sensor values!

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
    
    # TODO: Insert Line Following Code Here                
    
    vL, vR = my_controller.what_to_do().get_vals()
    
    
    
    # TODO: Call update_odometry Here
    update_odometry()
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
    
    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
