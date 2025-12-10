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
    
        
# 3. Setup Lidar Scanners
lid1 = robot.getDevice("lidar1")
lid2 = robot.getDevice("lidar2")
lid1.enable(timestep)
lid2.enable(timestep)
lid1Res = lid1.getHorizontalResolution()
lid2Res = lid2.getHorizontalResolution()
#Setup gps (used to test Occupancy Map seperate from Odometry)
gps = robot.getDevice("gps")
gps.enable(timestep)

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
        
# functions to convert coordinates to array indexes       
def get_x_index(x, enviromentLength):
    approxindex = ((x/0.01) + (enviromentLength/2))
    index = math.floor(approxindex)
    return index
    
def get_y_index(y, enviromentHeight):
    approxindex = -((y/0.01) - (enviromentHeight/2))
    index = math.floor(approxindex)
    return index

# function to convert array indexes to approximate coordinates
def get_x_coordinate(x_index, environmentLength):
    approxheight = ((0.01*x_index) - (environmentHeight/0.02))
    x = approxheight+0.005
    return x
    
def get_y_coordinate(y_index, environmentHeight):
    approxheight = ((environmentHeight/0.02) - (0.01*y_index))
    y = approxheight-0.005
    return y

# function to create array of angles representing each lidar scan
def get_angle_array(robotFN, lidarFRN, lid):
    res = lid.getHorizontalResolution()
    fov = lid.getFov()
    array = []
    for i in range(res):
        # sets initial value to angle of first lidar scan
        if i == 0:
            array.append(robotFN + lidarFRN)
        else:
            # increments angle based on lidar field of view and resolution
            angle = array[i-1] + (fov / (res - 1))
            # 
            if (angle > 2*math.pi):
                array.append(angle - (2*math.pi))
            else:
                array.append(angle)
            
    return array
    
# creates an occupancy grid containing just zeros    
def create_empty_occupancy_grid(enHeight, enLength):
    array = [[0 for _ in range(enLength)] for _ in range(enHeight)]
    return array 


def update_occupancy_grid(robotx, roboty, robotFN, lidarFRN, lid, occupancyGrid):

    #print("updating Occupancy Grid....")
    res = lid.getHorizontalResolution()
    #print(res)
    scan = lid.getRangeImage()
    #print(scan)
    angle = get_angle_array(robotFN, lidarFRN, lid)
    #print(angle)
    
    for i in range(res):
        if (math.isinf(scan[i])):
            pass
            
        else:
            x = 0
            y = 0
            
            if (0 <= angle[i] < (math.pi/2)):
                x = (scan[i] * math.sin(angle[i]))
                y = (scan[i] * math.cos(angle[i]))
            elif ((math.pi/2) <= angle[i] < (math.pi)):
                x = (scan[i] * math.cos(angle[i] - (math.pi/2)))
                y = -(scan[i] * math.sin(angle[i] - (math.pi/2)))
            elif (math.pi <= angle[i] < (3*math.pi/2)):
                x = -(scan[i] * math.sin(angle[i] - math.pi))
                y = -(scan[i] * math.cos(angle[i] - math.pi))
            else:
                x = -(scan[i] * math.sin((2*math.pi) - angle[i]))
                y = (scan[i] * math.cos((2*math.pi) - angle[i]))
                                
            x = x + robotx
            y = y + roboty
                        
            index_x = get_x_index(x, len(occupancyGrid[0]))
            index_y = get_y_index(y, len(occupancyGrid))
                        
            occupancyGrid[index_y][index_x] = 1
            
    return occupancyGrid
    

        
counter = 0

set_arm_pose(0.0, 1.1, -2.5, 1.5, 0.0) 
set_gripper(0.024) 

#Creates an Occupancy Grid for each shelf level
grid1 = create_empty_occupancy_grid(environmentHeight, environmentLength)
grid2 = create_empty_occupancy_grid(environmentHeight, environmentLength)

while robot.step(timestep) != -1:
    counter += 1
    
    # Sets robotCoordinates from gps (used to test Occupancy map seperate from odometry)
    robotCoordinates = gps.getValues()
 
    # Updates each Occupancy Grid
    grid1 = update_occupancy_grid(robotCoordinates[0], robotCoordinates[1], RobotsAngleFromNorth, LidarAngleFromRobot, lid1, grid1)
    grid2 = update_occupancy_grid(robotCoordinates[0], robotCoordinates[1], RobotsAngleFromNorth, LidarAngleFromRobot, lid2, grid2)
    
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
        
        #Prints Occupancy Map (for testing)
        for y in range(environmentHeight):
            for x in range(environmentLength):
                print(grid1[y][x], end="")
            print()
    