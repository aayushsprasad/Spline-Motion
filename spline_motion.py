import math
import time
from pololu_3pi_2040_robot import robot
motors = robot.Motors()
encoders = robot.Encoders()
# Function to calculate spline points
def spline_points(u, bx, by, Dax, Dbx, Dby):
    ax = 0
    ay = 0
    Day = 0
    
    u2 = u*u
    u3 = u2 * u
    
    h1 = 2 * u3 - 3*u2 + 1
    h2 = -2*u3 +3*u2
    h3 = u3 - 2*u2 + u
    h4 = u3 - u2
    
    sx = ax*h1 + bx*h2 + Dax*h3 + Dbx*h4
    sy = ay*h1 + by*h2 + Day*h3 + Dby*h4
    
    return sx, sy

# Function to calculate motion parameters
def motion_param(x, y, alpha):
    length = math.sqrt(x*x + y*y)
    Dbx = length * math.cos(alpha)
    Dby = length * math.sin(alpha)
    
    return length, Dbx, Dby

# Define spline parameters and number of points
x = 100
y = 100
alpha = math.pi / 2  # Facing towards y-axis
npoints = 10

# Calculate motion parameters
length, Dbx, Dby = motion_param(x, y, alpha)
print(length, Dbx, Dby)

# Initialize position variables
robot_x = 0
robot_y = 0
robot_orientation = 0

# Generate spline points
u_values = [i / (npoints - 1) for i in range(npoints)]
spline_points_list = [spline_points(u, x, y, length, Dbx, Dby) for u in u_values]
sx, sy = zip(*spline_points_list)

# Function to move along spline
def move_along_spline(sx, sy, npoints):
    global robot_x, robot_y, robot_orientation 
    for i in range(npoints):
        # Calculate difference between current and target points
        delta_x = sx[i] - robot_x
        delta_y = sy[i] - robot_y
        
        # Calculate distance and angle to target point
        distance = math.sqrt(delta_x**2 + delta_y**2)
        target_angle = math.atan2(delta_y, delta_x) * 180 / math.pi
        
        # Calculate angle difference
        angle_difference = target_angle - robot_orientation
        
        # Adjust robot orientation to face the target point
        rotate_robot(angle_difference)
        
        # Move the robot forward towards the target point
        forward(distance)
        
        # Update robot position and orientation
        robot_x = sx[i]
        robot_y = sy[i]
        robot_orientation = target_angle
        
        time.sleep(1)  

# Function to rotate the robot based on the angle difference
def rotate_robot(angle_difference):
    if angle_difference > 0:
        right(angle_difference)  # Rotate right for positive angle difference
    elif angle_difference < 0:
        left(-angle_difference)  # Rotate left for negative angle difference

# Function to move the robot forward based on encoder counts
def forward(encoder_counts):
    global robot_x, robot_y, motors, encoders
    encoders.get_counts(reset=True)  # Reset encoder counts
    motors.set_speeds(1000, 1000)  # Move forward
    while True:
        c = encoders.get_counts()[0]  # Get current encoder counts
        if c >= encoder_counts:  # Check if encoder count reaches the desired count
            break
        time.sleep(0.01)  # Adjust sleep duration as needed
    motors.set_speeds(0, 0)  # Stop motors

# Function to make a right turn based on angle difference
def right(angle_difference):
    # Calculate encoder counts based on the angle difference
    global robot_x, robot_y, motors, encoders
    encoder_counts = int(angle_difference / 90 * 240)
    encoders.get_counts(reset=True)  # Reset encoder counts
    motors.set_speeds(1000, -1000)  # Rotate right
    while True:
        c = encoders.get_counts()[0]  # Get current encoder counts
        if abs(c) >= encoder_counts:  # Check if encoder count reaches the desired count
            break
        time.sleep(0.01)  # Adjust sleep duration as needed
    motors.set_speeds(0, 0)  # Stop motors
    forward(700)  # Move forward after the turn

# Function to make a left turn based on angle difference
def left(angle_difference):
    # Calculate encoder counts based on the angle difference
    global robot_x, robot_y, motors, encoders
    encoder_counts = int(angle_difference / 90 * 240)
    encoders.get_counts(reset=True)  # Reset encoder counts
    motors.set_speeds(-1000, 1000)  # Rotate left
    while True:
        c = encoders.get_counts()[0]  # Get current encoder counts
        if abs(c) >= encoder_counts:  # Check if encoder count reaches the desired count
            break
        time.sleep(0.01)  # Adjust sleep duration as needed
    motors.set_speeds(0, 0)  # Stop motors
    forward(700)  # Move forward after the turn

move_along_spline(sx, sy, npoints)