import bpy
import csv
import numpy as np
from numpy import pi, cos, sin

# ===================================================================

# 0 - X euler axis
# 1 - Y euler axis
# 2 - Z euler axis

# ===================================================================

# ==  D-H table: LS3-401  ==

# Origin offsets
o = [0, 0]
# Link offsets
d = [0, 0]
# Link lengths
a = [0.225, 0.175]

# ===================================================================


def FK_dh(th):
    """
    Computation of Forward Kinematics using classic Denavit-Hartenberg (D-H) tables  

    :param th: Joints angle
    """
    
    # Initialize transformation matrix
    A = np.identity(4)
    
    for i in range(len(th)):
        # Compute individual transformation matrix
        A_x = np.array([
            [cos(th[i]), -sin(th[i]) * cos(o[i]), sin(th[i]) * sin(o[i]), a[i] * cos(th[i])],
            [sin(th[i]), cos(th[i]) * cos(o[i]), -cos(th[i]) * sin(o[i]), a[i] * sin(th[i])],
            [0, sin(o[i]), cos(o[i]), d[i]],
            [0, 0, 0, 1]
        ])

        if i == 0:
            # Assign the first transformation matrix
            A = A_x
        else:
            # Accumulate transformations
            A = A @ A_x 
    
    # Extract the position components
    return A[:2, 3]

def IK_analytical(goal, cfg):
    """
    Computation of Inverse Kinematics using trigonometric functions 

    :param goal: [x, y] position of target goal [m]
    :param cfg: 2D SCARA robot configuration 0 or 1
    :return: [theta1 [rad], theta2 [rad]]
    """    
    
    theta = np.zeros(2)
    
    # Compute theta1
    cosT_beta_numerator = (a[0]**2) + (goal[0]**2 + goal[1]**2) - (a[1]**2)
    cosT_beta_denumerator = 2 * a[0] * np.sqrt(goal[0]**2 + goal[1]**2)
    
    # Check for available solution
    if abs(cosT_beta_numerator / cosT_beta_denumerator) > 1:
        raise ValueError("Error: theta[0] out of range for theta1!")
    else:
        if cfg == 0:
            theta[0] = np.arctan2(goal[1], goal[0]) - np.arccos(cosT_beta_numerator / cosT_beta_denumerator)
        else:
            theta[0] = np.arctan2(goal[1], goal[0]) + np.arccos(cosT_beta_numerator / cosT_beta_denumerator)
    
    # Compute theta2
    cosT_alpha_numerator = (a[0]**2) + (a[1]**2) - (goal[0]**2 + goal[1]**2)
    cosT_alpha_denumerator = 2 * (a[0] * a[1])
    
    if abs(cosT_alpha_numerator / cosT_alpha_denumerator) > 1:
        raise ValueError("Error: theta[1] out of range for theta2!")
    else:
        if cfg == 0:
            theta[1] = np.pi - np.arccos(cosT_alpha_numerator / cosT_alpha_denumerator)
        else:
            theta[1] = np.arccos(cosT_alpha_numerator / cosT_alpha_denumerator) - np.pi
    
    return theta

# Function to set joint rotations to reach a given position
def go_to_position(theta):
    # Get references to the objects representing the robot's arms
    arm_1 = bpy.data.objects['arm_1']
    arm_2 = bpy.data.objects['arm_2']
    
    # Set target joint rotations
    arm_1.rotation_euler[1] = theta[0]
    arm_2.rotation_euler[2] = theta[1]

# Function to animate the robot to reach a given position
def animate_to_position(end_t):
    arm_1 = bpy.data.objects['arm_1']
    arm_2 = bpy.data.objects['arm_2']
    
    start_t = np.zeros(2)
        
    start_t[0] = arm_1.rotation_euler[1]
    start_t[1] = arm_1.rotation_euler[2]
    
    nr_pnts = 100
    a = np.zeros((nr_pnts, 2))

    # Generate intermediate positions for animation
    for i in range(2):
        a[:, i] = np.linspace(start_t[i], end_t[i], nr_pnts)
    
    # Animate through intermediate positions
    for frame in range(nr_pnts):
        arm_1.rotation_euler[1] = a[frame][0]
        arm_2.rotation_euler[2] = a[frame][1]
        
        # Insert keyframes for animation
        arm_1.keyframe_insert("rotation_euler", frame=frame)
        arm_2.keyframe_insert("rotation_euler", frame=frame)

# Main function
def main():
    fabrik = False
    
    goal = [0., -0.3]  # Target position
    cfg = 0  # Robot configuration
    
    # Calculate inverse kinematics
    theta = IK_analytical(goal, cfg)
    
    # result of FABRIK
    if fabrik:
        with open('FABRIK_result.csv', newline='') as csvfile:
            # Create a CSV reader object
            csv_reader = csv.reader(csvfile)
            
            # Read each row in the CSV file
            for th in csv_reader:
                theta = [float(th[0]), float(th[1])]
    
    
    # Calculate forward kinematics to verify solution
    xy = FK_dh(theta)

    # Print results
    print(f"IK result: configuration {cfg}: theta1 = {theta[0]:.2f}, theta2 = {theta[1]:.2f}")
    print(f"FK check result: x = {xy[0]:.2f}, y = {xy[1]:.2f}")

    # Move to home position
    go_to_position([0, 0])

    # Move to calculated position
    # go_to_position(theta)

    # Animate movement to the calculated position
    # animate_to_position(theta)
    
main()