import bpy
import numpy as np
from numpy import pi, cos, sin

# ===================================================================

# 0 - X euler axis
# 1 - Y euler axis
# 2 - Z euler axis

# ===================================================================

# ==  D-H table: LS3-401  ==

o    = [0, 0]
d    = [0, 0]
a    = [0.225, 0.175]

# ===================================================================


def FK_dh(th):
    """
    Computation of Forward Kinematics by classics D-H tables  

    :param th: joints angle
    """
    
    for i in range(len(th)):
        A_x = np.array([
            [cos(th[i]),  -sin(th[i]) * cos(o[i]),     sin(th[i]) * sin(o[i]),     a[i] * cos(th[i])], 
            [sin(th[i]),   cos(th[i]) * cos(o[i]),    -cos(th[i]) * sin(o[i]),     a[i] * sin(th[i])],
            [0       ,     sin(o[i]),                  cos(o[i]),                  d[i]             ],
            [0       ,     0,                          0,                          1                ]           
        ])

        if i == 0:
            A = A_x
        
        else:
            A = A @ A_x 
    
    return A[:2,3]

def IK_analytical(goal, cfg):
    """
    Computation of Inverse Kinematics by trigonometric functions 

    :param goal: [x, y] position of target goal [m]
    :param cfg: 2D SCARA robot configuration 0 or 1
    :return: [theta1 [rad], thata2 [rad]]
    """    
    
    theta = np.zeros(2)
    
    cosT_beta_numerator   = (a[0]**2) + (goal[0]**2 + goal[1]**2) - (a[1]**2)
    cosT_beta_denumerator = 2 * a[0] * np.sqrt(goal[0]**2 + goal[1]**2)
    
    # Check for avaible solution
    if abs(cosT_beta_numerator / cosT_beta_denumerator) > 1:
        raise ValueError("Error: theta[0] out of range for theta1!")
    
    else:
        if cfg == 0:
            theta[0] = np.arctan2(goal[1], goal[0]) - np.arccos( cosT_beta_numerator / cosT_beta_denumerator )
        
        else:
            theta[0] = np.arctan2(goal[1], goal[0]) + np.arccos( cosT_beta_numerator / cosT_beta_denumerator )
    
    cosT_alpha_numerator   = (a[0]**2) + (a[1]**2) - (goal[0]**2 + goal[1]**2)
    cosT_alpha_denumerator = 2 * (a[0] * a[1])
    
    if abs(cosT_alpha_numerator / cosT_alpha_denumerator) > 1:
        raise ValueError("Error: theta[1] out of range for theta2!")
    
    else:
        if cfg == 0:
            theta[1] = np.pi - np.arccos( cosT_alpha_numerator / cosT_alpha_denumerator )
        
        else:
            theta[1] = np.arccos( cosT_alpha_numerator / cosT_alpha_denumerator ) - np.pi
        
    
    return theta


def go_to_position(theta):
    # Get references to the objects
    arm_1 = bpy.data.objects['arm_1']
    arm_2 = bpy.data.objects['arm_2']

    arm_1.rotation_euler[1] = theta[0]
    arm_2.rotation_euler[2] = theta[1]


def animate_to_position(end_t):
    arm_1 = bpy.data.objects['arm_1']
    arm_2 = bpy.data.objects['arm_2']
    
    start_t = np.zeros(2)
        
    start_t[0] = arm_1.rotation_euler[1]
    start_t[1] = arm_1.rotation_euler[2]
    
    nr_pnts = 100
    a = np.zeros((nr_pnts, 2))

    for i in range(2):
        a[:,i] = np.linspace(start_t[i], end_t[i], nr_pnts)
    
    for frame in range(nr_pnts):
        arm_1.rotation_euler[1] = a[frame][0]
        arm_2.rotation_euler[2] = a[frame][1]
        
        arm_1.keyframe_insert("rotation_euler", frame=frame)
        arm_2.keyframe_insert("rotation_euler", frame=frame)
        
def main():
    goal = [ 0.1, -0.1 ]
    cfg = 0

    theta = IK_analytical(goal, cfg)
    xy = FK_dh(theta)

    print(f"IK result: configuration {cfg}: theta1 = {theta[0]:.2f}, theta2 = {theta[1]:.2f}")
    print(f"FK check result: x = {xy[0]:.2f}, y = {xy[1]:.2f}")

    # home position
    # go_to_position([0,0])

    # calculated position
    # go_to_position(theta)

#    animate_to_position(theta)
    
main()