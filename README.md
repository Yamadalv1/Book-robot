Self implemented features 

1. Odometry and localization: (do_odom)
We've implemented kinematics equations for the robot to calculate it's pose (x,y,theta), manually converted wheel encoded readings into distance travelled linearly and the changes in orientation using
the radius of the wheels and axle lengths. The robot updates it's global pose estimate at every time-step using trigonometric equations. 

2.​‍​‌‍​‍‌​‍​‌‍​‍‌ Occupancy Grid Mapping (update_occupancy_grid)
Lidar Data Processing: Handled the conversion of raw Lidar range images given in polar coordinates into Cartesian coordinates (x,y) relative to the robot by direct code.
coordinate Transformation: Based on the robot's current heading, it uses a rotation matrix to change local LIDAR points into the global map frame.
grid Logic: Developed a personalised 2D array structure in which the obstacles are divided into the cells of the grid. The functions get_x_index and get_y_index take the continuous world coordinates and return the corresponding indices of the array.

3. Mecanum Drive Kinematics (drive)

Vector Mixing: Codified the mixing logic which is necessary for Mecanum wheels. The function is given the desired Forward, Right (Strafe), and Rotate vectors and does the manual calculation of each of the four independent wheel velocities to be able to carry out the motion.

4. Finite State Machine (Task Sequencer)

Sequential Logic: The highest-level robot behavior control is done by a counter-based state machine.
Manipulation Sequence: Manually adjusted joint angles and the timing of the 5-DOF robotic arm to accomplish the following complex tasks:
Aligning with the shelf.
Extending the arm.
Gripping the book.
Retracting and stowing the object for ​‍​‌‍​‍‌​‍​‌‍​‍‌transport.

pre-programmed packages and APIs 

We employed the local Webots Python API for controlling the robot hardware:

1. Webots Controller API (controller) -
Robot: Mainly for timestep control (step) and device retrieval (getDevice).
Motor: Primarily to specify target positions (for the arm) and velocities (for the wheels).
PositionSensor: Essentially to get raw encoder values (radians) for our odometry calculations.
Lidar: Only to get the raw RangeImage array and resolution; the entire mapping logic was our own.
GPS & InertialUnit: Basically used to get the ground truth data for debugging and verifying the accuracy of our calculated odometry.

2. Python Standard Library (math) -
The library was utilized for trigonometric and geometric calculations:
sin, cos: For odometry pose updates and coordinate transformations.
pi, degrees: For angle conversions.
floor: For discretising continuous coordinates into map grid ​‍​‌‍​‍‌​‍​‌‍​‍‌indices.
