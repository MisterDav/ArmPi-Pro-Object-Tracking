import rospy
import camera
import control
import triangulate
import threading

import cv2
import numpy as np
import math


object_dict = {
    0: "red cube",
    1: "green cube",
    2: "blue cube",
    3: "red sphere",
    4: "green sphere",
    5: "blue sphere",
    6: "red cylinder",
    7: "green cylinder",
    8: "blue cylinder"
}


TARGET_NAME = "blue sphere"
OBJECT_TYPE = list(object_dict.values()).index(TARGET_NAME)
#OBJECT_TYPE = -1

CONF_THRESHOLD = 0.625
CENTER_THRESHOLD = 50
IMG_SIZE = 640

# Checks this many times to ensure the object is stationary before
# Attempting to triangulate its position.
TRIES_TO_STATIONARY = 16

# Waits this many frames to ensure the object doesn't accidentally think the object got lost when it didn't.
REPEATS_BEFORE_LOST = 10

MOVE_SPEED = 60
TRY_GRAB_TIMER = 30

img_center = int(IMG_SIZE / 2)
track_id = -1
track_positions = [(-1,-1), (-1,-1)]
stationary_passes = 0
snapshots = []
trying_to_grab = 0


# ----------------
# HELPER FUNCTIONS
# ----------------
def get_box_by_id(id, results=None):
    global track_id
    if camera.model == None: return
    
    a = camera.model.results
    if results != None:
        a = results
    if a == None:
        #print("No results")
        return
    #print(a[0].names)
    
    with camera.model.lock:
        if a[0].boxes == None:
            #print("No boxes")
            return None
        
        try:
            for i in range(len(camera.model.results[0].boxes)):
                if a[0].boxes.id[i] == track_id:
                    #print("Found object with ID!")
                    return a[0].boxes.xywh[i]
        except:
            pass
    #print("Idk what went wrong")

def get_center_of_box(box):
    center_x = box[0]
    center_y = box[1]
    return center_x, center_y 

def save_position_of_object():
    global track_positions
    box = get_box_by_id(track_id)
    if box == None: return
    
    cx, cy = get_center_of_box(box)
    
    del track_positions[0]
    track_positions.append( (cx, cy) )
    

# ------------------
# TRACKING FUNCTIONS
# ------------------
def search_for_object(object_class=8):
    global track_id
    
    if camera.model == None: return False
    if camera.model.results == None: return False
    
    trackables = []
    
    #print(camera.model.results[0].names)
    box = camera.model.results[0].boxes
    
    with camera.model.lock:
        for i in range(len(box.cls)):
            
            try:
                index = box.cls[i]
                #print(i, index)
                #print(box)
                if object_class != index: continue					# Check if it's the object type.
                if camera.model.results[0].boxes.conf[i] < CONF_THRESHOLD: continue		# Check if it's above a certain confidence.
        
                box_size = camera.model.results[0].boxes.xywh[i][-2::]
                trackables.append((box.id[i], box_size))
            except:
                pass

    trackables.sort(key = lambda x: x[1][0] * x[1][1], reverse = True)
    
    if len(trackables) != 0:
        track_id = trackables[0][0]
        return True
    else:
        return False

CENTER_THRESHOLD = 50
def readjust_to_center(check=False, grounded=False, fine_tune=False, vertical_only=False):
    #print("Recentering")
    global track_id
    ct = CENTER_THRESHOLD
    if fine_tune: ct = int(ct / 5)
    
    box = get_box_by_id(track_id)
    if track_id == -1: return True
    if box == None:
        #print("Lost object!")
        return False 		# The object probably got lost.
    
    cx, cy = get_center_of_box(box)
    
    cond_a = img_center - ct <= cx <= img_center + ct
    cond_b = img_center - ct <= cy <= img_center + ct
    
    if cond_a and cond_b:
        return True		# Object found, already centered
    
    if grounded: trying_to_grab = TRY_GRAB_TIMER
    if not cond_a:
        dx = img_center - cx
        control.wheel_rotate_by( int(dx / 15), math.copysign(10, dx) )
    if not cond_b:
        dy = img_center - cy
        if grounded:
            if fine_tune: 
                control.wheel_move_by( 0, -int(dy / 7) , 30)
            else:
                control.wheel_move_by( 0, -int(dy / 7) , MOVE_SPEED)
    
    rospy.sleep(1)
   
    if check: return False	# Setting check to true essentially asks if the object is already centered or not.
    else: return True
    
def attempt_object_recovery():
    #print("Recovering")
    global track_id, track_positions
    
    a = track_positions
    last_velocity = (a[1][0] - a[0][0], a[1][1] - a[0][1])
    
    checks = 250
    x = 15
    for i in range( int(360 / x) ):
        control.wheel_rotate_by( x, math.copysign(10, -last_velocity[0]) )
        successful = search_for_object(OBJECT_TYPE)
        if successful:
            return True
    track_id = -1
    return False

def check_if_stationary():
    global trying_to_grab
    if track_id == -1: return False
    
    for x in range(TRIES_TO_STATIONARY):
        
        # We want to wait until the next frame is loaded.
        camera.model.processed_frame.wait()     
        camera.model.processed_frame.clear()
        
        if not readjust_to_center(check=True):
            return False
    
    trying_to_grab = TRY_GRAB_TIMER
    rospy.sleep(1)
    return True

def check_distance_from_self():
    #print("Checking distance from self")
    if not control.cam_rotation <= 0: return None
    #print(control.cam_rotation)
    if not readjust_to_center(check=True, grounded=True, fine_tune=True, vertical_only = True): return None
    y_component = 0.25 / math.tan( math.radians(control.arm_position[-1]) )
    return y_component

#def reposition_to_grab_object():
#    box = get_box_by_id(track_id)
#    if

def approximate_distance():
    #print("Approximating distance")
    camera.model.new_frame.wait()     
    camera.model.new_frame.clear()
    save_position_of_object()
    
    control.wheel_move_by(45, 0, MOVE_SPEED)
    rospy.sleep(3)
    
    camera.model.new_frame.wait()     
    camera.model.new_frame.clear()
    save_position_of_object()
   
    du = track_positions[1][0] - track_positions[0][0]
    if du < 5: du = 5
    dist = (786 * (5/2)) / du
    if dist > 100: return 0
    return dist
    
def readjust_to_bottom():
    #print("Recentering to the bottom")
    global track_id
    ct = CENTER_THRESHOLD
    
    box = get_box_by_id(track_id)
    print(box, track_id)
    if track_id == -1: return True
    if box == None:
        #print("Lost object!")
        return False 		# The object probably got lost.
    
    cx, cy = get_center_of_box(box)
    print(cy)
    cond_b = cy >= 640 - int(ct/10)
    
    if cond_b:
        return True		# Object found, already centered
    
    if not cond_b:
        dy = (640 - int(ct/10)) - cy
        control.wheel_move_by( 0, -int(dy / 15) , 15)
    
    rospy.sleep(1)
    return True

def try_to_grab():
    global trying_to_grab
    with control.cam_lock:
        #print("Trying to grab")
        y = check_distance_from_self()
        if y is None: return
        control.claw_change(0)
        control.arm_move_to(0, y/2 + 0.05, -0.05)
        control.cam_rotate_to(40)
        control.wheel_move_by(0, -80, 30)
        
        tries = 0
        LAST_RESORT = False
        rospy.sleep(0.5)
        while not readjust_to_bottom():
            #search_for_object()
            camera.model.processed_frame.wait()     
            camera.model.processed_frame.clear()
            search_for_object()
            #print("Search successful?: ", search_for_object())
            tries += 1
            if tries > 30:
                LAST_RESORT = True
                break
        
        if LAST_RESORT:
            print("Last resort :(")
            control.wheel_move_by(0, -8, 15)
        rospy.sleep(0.5)
        control.claw_change(500)
        rospy.sleep(1)
        control.arm_move_to(0.0, 0.12, 0.08)
        control.claw_change(0)
        trying_to_grab = 0

def always_adjust():
    global track_id
    global trying_to_grab
    ct = CENTER_THRESHOLD
    
    while True:
        if trying_to_grab > 0:
            trying_to_grab -= 1
            rospy.sleep(0.2)
            continue
    
        box = get_box_by_id(track_id)
        if track_id == -1: continue
        if box == None: continue 		# The object probably got lost.
    
        cx, cy = get_center_of_box(box)
        cond_b = img_center - ct <= cy <= img_center + ct

        if not cond_b:
            dy = img_center - cy
            with control.cam_lock:
                control.cam_rotate_by( int(dy / 35) )

approached = False
if __name__ == "__main__":
    
    rospy.init_node("final_model")
    threading.Thread(target=always_adjust, daemon=True).start()
    control.stop()
    camera.model = camera.init()
    lost_frames = 0
    
    #rospy.sleep(15)
    #try_to_grab()
    #exit()
    
    while True:
        image_snapshot = []
        save_position_of_object()
        
        search_for_object(OBJECT_TYPE)
        if not readjust_to_center() and track_id != -1:
            camera.model.new_frame.wait()     
            camera.model.new_frame.clear()
            lost_frames += 1 #; print("Increased lost frames")
            
            if lost_frames > REPEATS_BEFORE_LOST:
                attempt_object_recovery()#; print("Attempt object recovery")
                lost_frames = 0
        
        #print(control.cam_rotation, control.arm_position[-1])
        if not check_distance_from_self() is None:
            if check_if_stationary():
                try_to_grab()
                track_id = -1
#             attempt_object_recovery()

        if check_if_stationary() and control.cam_rotation > 0:
            dist = approximate_distance()
            print(dist)
            if dist < 0: continue
            
            if not readjust_to_center():
                camera.model.new_frame.wait()     
                camera.model.new_frame.clear()
                control.stop()
                continue
            control.wheel_move_by(0, -dist*3.5, MOVE_SPEED)
            
            #if check_if_stationary():
            #    if not pre_triangulation(): continue
            #    print(triangulation())
            #    while True: pass
