from controller import Robot

# initialise the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#Imports for occupancy Mapping
from controller import Lidar
import math
from controller import GPS

#Environment Values Needed for Occupancy Mapping
#Height and Length of RectangleArena divided by 0.01
environmentHeight = 10000
environmentLength = 250
#Sets initial coordinates of robot
robotCoordinates = (-0.437826,0.004242735,0.0988747)
#Robots bearing from North
RobotsAngleFromNorth = (3*math.pi)/2
#bearing of first lidar scan from robots normal
LidarAngleFromRobot = (7*math.pi)/4

# 1. Setup Wheels
wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf')) 
    motor.setVelocity(0.0)
    wheels.append(motor)

# 2. Setup Arm Motors
arm_motors = []
arm_names = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5']
for name in arm_names:
    motor = robot.getDevice(name)
    motor.setVelocity(1.0) # so the arm doesn't go too fast
    arm_motors.append(motor)

# initialise grippers
finger1 = robot.getDevice('finger::left')
finger2 = robot.getDevice('finger::right')

# Verify we found them 
if finger1 is None or finger2 is None:
    print("WARNING: Could not find 'finger::left' or 'finger::right'.")
    finger1 = robot.getDevice('finger1')
    finger2 = robot.getDevice('finger2')

# max limits the grippers can open and close
if finger1 and finger2:
    finger1.setVelocity(1.0)
    finger2.setVelocity(1.0)
    finger1.setPosition(0.024) 
    finger2.setPosition(0.024)
else:
    print("Gripper motors not found. Check Scene Tree names.")


def drive(forward, right, rotate):
    
    wheels[0].setVelocity(forward - right - rotate) # Front Left
    wheels[1].setVelocity(forward + right + rotate) # Front Right
    wheels[2].setVelocity(forward + right - rotate) # Back Left
    wheels[3].setVelocity(forward - right + rotate) # Back Right

def set_arm_pose(a1, a2, a3, a4, a5):
    arm_motors[0].setPosition(a1) # Base rotation
    arm_motors[1].setPosition(a2) # Shoulder
    arm_motors[2].setPosition(a3) # Elbow
    arm_motors[3].setPosition(a4) # Wrist Pitch
    arm_motors[4].setPosition(a5) # Wrist Roll

def set_gripper(width):
    if width > 0.024: width = 0.024
    if width < 0.0: width = 0.0
    
    if finger1 and finger2:
        finger1.setPosition(width)
        finger2.setPosition(width) 
        
counter = 0

set_arm_pose(0.0, 1.1, -2.5, 1.5, 0.0) 
set_gripper(0.024) 

while robot.step(timestep) != -1:
    counter += 1
    
    if counter < 100:
        # Move Forward
        drive(4.0, 0, 0)
    elif counter < 180:
        # Slide Right (Strafe)
        drive(0, 2.0, 0)
        
    elif counter == 180:
        drive(0, 0, 0) 
        set_arm_pose(0.0, -0.6, -1.0, -0.5, 0.0)

   
    # Extends the arm into the shelf
    elif counter == 250:
        set_arm_pose(0.0, -0.6, -1.8, -0.5, 0.0)


    # grabs the book
    elif counter == 350:
        set_gripper(0.0) # Clamp shut
        print("Grabbing book...")

    # pulls out the book
    elif counter == 450:
        set_arm_pose(0.0, -0.6, -0.2, -0.5, 0.0)
        drive(-1.0, 0, 0) 

   # LIFT & SECURE (Only happens after we are clear)
    elif counter == 550:
        drive(0, 0, 0) # Stop wheels
        
        # now it is safe to lift the arm up
        set_arm_pose(0.0, 0.8, -0.5, -1.0, 0.0)
        print("Retrieval Complete.") 
    
    elif counter == 600:  
      set_arm_pose(-6.0,0.8, -1.0,-1.5,0.0) 
      
    elif counter == 650: 
      set_arm_pose (-6.0,-0.1, -1.0,-1.5,0.0) 
       
    elif counter == 700:
      set_gripper(0.024)
    
    elif counter == 750: 
       set_arm_pose (-6.0, 0.8,-1.0,-1.5,0.0) 
      
    elif counter == 800: 
        drive(-3.0, -6.0, 4.0) 
        print("rotating our robot")
   
    elif counter == 1400: 
        drive(0,0,0) 
        
    elif counter == 1550: 
        drive(6.0,0,0) 
        print("Drive towards the book box") 
    
    elif counter == 2050: 
        drive(0,0,0)
    
    elif counter == 2150: 
        set_arm_pose (-6.0,-0.2, -1.0,-1.5,0.0) 
        
    elif counter == 2200: 
        set_gripper(0.0) 
     
    elif counter == 2250: 
        set_arm_pose(-6.0,0.8, -1.0,-1.5,0.0) 
    
    elif counter == 2300:
        set_arm_pose(0.0,0.8, -1.0,-1.5,0.0) 
    
    elif counter == 2350: 
        set_arm_pose(0.0,-0.4, -1.0,-1.5,0.0)
    
    elif counter == 2400: 
        set_gripper(0.024) 
        print("release book into the box...")
    
    elif counter == 2450: 
        set_arm_pose(0.0,0.8, -1.0,-1.5,0.0) 
        print("Done :)")
    
    