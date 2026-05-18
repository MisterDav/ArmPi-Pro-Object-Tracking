# Import all of the wheel control modules.
import sys
import rospy
import math
import threading
from chassis_control.msg import *

# Import all of the arm control modules.
from ik_module import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
import time

arm_position = [0,0,0,0]
cam_rotation = 0
claw_pos = 0
cam_lock = threading.Lock()

AK = ik_transform.ArmIK()

joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=10)
set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=3)
rospy.sleep(0.5)

# Function to move the robot arm.
def arm_move_to(x, y, z):
    global arm_position, cam_rotation
    if z < -0.13 or z > 0.15: return
    if x < -0.05 or x > 0.05: return
    
    target = AK.setPitchRanges((x, y, z), -145, -180, 0)
    
    if target:
        #print(target)
        servo_data = target[1]
        theta_data = target[0]
        bus_servo_control.set_servos(joints_pub, 1, ((1, claw_pos), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        camera_angle = theta_data['theta3'] + theta_data['theta4'] + theta_data['theta5']
        cam_rotation = theta_data['theta3'] + 100
        arm_position = [x,y,z,camera_angle]
    else:
        print("No valid configuration!")
    rospy.sleep(2)

def cam_rotate_to(degrees):
    global cam_rotation
    cam_rotation = min( max( degrees, 0 ), 90 )
    degrees_conv = (cam_rotation * 450) / 90
    bus_servo_control.set_servos(joints_pub, 1, ((1, claw_pos), (2, 500), (3, degrees_conv)) )
    rospy.sleep(0.5)

def cam_rotate_by(degrees):
    global cam_rotation
    cam_rotation += degrees
    cam_rotation = min( max( cam_rotation, 0 ), 90 )
    degrees_conv = (cam_rotation * 450) / 90
    bus_servo_control.set_servos(joints_pub, 1, ((1, claw_pos), (2, 500), (3, degrees_conv)) )
    rospy.sleep(0.5)

def wheel_move_by(x, y, speed=30):

    angle = math.atan2(y, x)
    magnitude = math.sqrt(x**2 + y**2)
    degrees = (math.degrees(angle) + 180) % 360
    
    seconds = magnitude / speed
    
    set_velocity.publish(speed, degrees, 0)
    rospy.sleep(seconds)
    set_velocity.publish(0,0,0)
    
    rospy.sleep(0.5)

def claw_change(value):
    global claw_pos
    bus_servo_control.set_servos(joints_pub, 1, ((1, value),))
    claw_pos = value
    rospy.sleep(0.5)

def wheel_rotate_by(degrees, speed=0.1):
    seconds = degrees / speed
    
    set_velocity.publish(0, 0, math.radians(speed))
    rospy.sleep(abs(seconds))
    set_velocity.publish(0,0,0)
    rospy.sleep(0.5)

# Function to return all of the servos and arms to the resting position.
def stop():
    set_velocity.publish(0,0,0)
    arm_move_to(0.0, 0.12, 0.08)
    rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("arm_test")
    
    #cam_rotate_to(0)
    #rospy.sleep(2)
    #for x in range(9):
    #    cam_rotate_by(15)
    #    rospy.sleep(2)
    control.arm_move_to(0, 0.35/2 + 0.05, -0.13)
    
    
    rospy.sleep(2)
    arm_move_to(0, 0.35/2 + 0.05, -0.05)
    cam_rotate_to(40)
    claw_change(500)
    rospy.sleep(2)
    arm_move_to(0.0, 0.12, 0.08)
    rospy.sleep(2)
    claw_change(0)
    
    stop()
    
    #rospy.sleep(1)
    #set_velocity.publish(60, 0, 0)
    #rospy.sleep(0.5)
    
    #wheel_move_by(0, 45, 30)
    #wheel_move_by(0, -45, 30)
    #wheel_rotate_by(15, 10)
    #wheel_rotate_by(15, -10)
    #stop()
    #rospy.sleep(0.5)
    #set_velocity.publish(60, 270, 0)
    #rospy.sleep(0.5)
