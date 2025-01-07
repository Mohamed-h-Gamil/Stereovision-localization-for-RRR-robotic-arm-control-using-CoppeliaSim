import math
import sys
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

def sysCall_init():
    sim = require('sim')
     
    global joint1
    global joint2
    global joint3
    global cap_right
    global cap_left
    global B
    global f
    global alpha
    global count
    global red_ball_world
    global l1
    global l2
    
    l1 = 0.9; l2 = 0.9
    joint1 = sim.getObject("/Cylinder/Revolute_joint")
    joint2 = sim.getObject("/Cylinder/Revolute_joint/Cylinder/Revolute_joint")
    joint3 = sim.getObject("/Cylinder/Revolute_joint/Cylinder/Revolute_joint/Cuboid/Revolute_joint")
    
    # Open both cameras
    cap_right = sim.getObject("/visionSensor[1]")                   
    cap_left =  sim.getObject("/visionSensor[0]")
    frame_right, resR = sim.getVisionSensorImg(cap_right,0, 0.0,[0, 0],[0, 0])
    frame_left, resL = sim.getVisionSensorImg(cap_left,0, 0.0,[0, 0],[0, 0])
    B = 0.2               #Distance between the cameras [m]
    f = (np.sqrt(3)/40)*100   #Camera lense's focal length [mm] (sensor_size/(2*tan(alpha/2))) = 4.330127
    alpha = 60        #Camera field of view in the horisontal plane [degrees]    
    
    #Initial values
    count = 0

def sysCall_actuation():
    # put your actuation code here
    global joint1
    global joint2
    global joint3
    global red_ball_world
    global count
    global l1
    global l2
    
    if count == 2:
        count += 1
        theta1 = np.pi/2 - angle_between_vectors([1,0], red_ball_world[:2])[0] 
        print(f"Theta 1 = {theta1}")
        sim.setJointTargetPosition(joint1, theta1)
        cylinder_coor = world_to_cylinder(red_ball_world, theta1)   # coordinates relative to joint 2
        if np.linalg.norm(cylinder_coor) > (l1+l2):
            print("ERROR: Object out of reach")
        else:
            print(f"Red_ball coordinates relative to joint 2: {cylinder_coor}")
            theta3 = get_theta3(l1, l2, cylinder_coor[0], cylinder_coor[1])
            theta2 = get_theta2(l1, l2, cylinder_coor[0], cylinder_coor[1], theta3)
            print(f"Theta 2 = {theta2}")
            print(f"Theta 3 = {theta3}")
            sim.setJointTargetPosition(joint2, theta2)
            sim.setJointTargetPosition(joint3, theta3)
    

def sysCall_sensing():
    # put your sensing code here
    global cap_right
    global cap_left
    global B
    global f
    global alpha
    global count
    global red_ball_world
    
    if count == 0:
        count += 1
        frame_right, resR = sim.getVisionSensorImg(cap_right,0, 0.0,[0, 0],[0, 0])
        frame_left, resL = sim.getVisionSensorImg(cap_left,0, 0.0,[0, 0],[0, 0])
        imgBuffer = sim.saveImage(frame_left, resL, 0, "/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/left0.png", -1)
        imgBuffer = sim.saveImage(frame_right, resR, 0, "/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/right0.png", -1)
        frame_left = cv2.imread("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/left0.png", cv2.IMREAD_COLOR)
        frame_right = cv2.imread("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/right0.png", cv2.IMREAD_COLOR)
################## CALIBRATION #########################################################
        cv_file = cv2.FileStorage()
        cv_file.open('/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/stereoMap.xml', cv2.FileStorage_READ)

        stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
        stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
        stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
        stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

        # Undistort and rectify images
        Frame_right = cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        Frame_left = cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                            
        # save the frames in buffer
        cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/test_left.png", frame_left) 
        cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/test_right.png", frame_right)
########################################################################################
        # If cannot catch any frame, break
        if resR==0 or resL==0:                    
            print("ERROR1: No image returned")

        else:
            # APPLYING HSV-FILTER:
            mask_right = add_HSV_filter(frame_right, 1)
            mask_left = add_HSV_filter(frame_left, 0)

            # Result-frames after applying HSV-filter mask
            res_right = cv2.bitwise_and(frame_right, frame_right, mask=mask_right)
            res_left = cv2.bitwise_and(frame_left, frame_left, mask=mask_left) 

            # APPLYING SHAPE RECOGNITION:
            circles_right = find_circles(frame_right, mask_right)
            circles_left  = find_circles(frame_left, mask_left)
            ################## CALCULATING BALL DEPTH #########################################################

            # If no ball can be caught in one camera show text "TRACKING LOST"
            if np.all(circles_right) == None or np.all(circles_left) == None:
                cv2.putText(frame_right, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                cv2.putText(frame_left, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

            else:
                # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
                # All formulas used to find depth is in video presentaion
                depth = find_depth(circles_right, circles_left, frame_right, frame_left, B, f, alpha)

                cv2.putText(frame_right, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_left, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_right, "Distance: " + str(round(depth,3)) + " m", (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_left, "Distance: " + str(round(depth,3)) + " m", (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                # Multiply computer value with 205.8 to get real-life depth in [cm]. The factor was found manually.
                print("Depth: ", depth, 'm')
                meter_per_pixel = depth * (np.tan((alpha/2)*np.pi/180)/(1024/2))
                real_center_x = (512-circles_left[0]) * meter_per_pixel
                real_center_y = (512-circles_left[1]) * meter_per_pixel
                print(f"red ball relative to left cam: {(real_center_x, real_center_y, depth)}")

            # Show the frames
            cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/right_frame.png", frame_right) 
            cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/left_frame.png", frame_left)
            cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/right_mask.png", mask_right) 
            cv2.imwrite("/media/gamil/Windows-SSD/FOLDER/STUDY/EJUST/Robotics/FINAL _PROJECT/images/buffer/left_mask.png", mask_left)
            
            # Camera Coordinates to World Coordinates
            red_ball_world = camera_to_world([real_center_x, real_center_y, depth, 1])
            print(f"Red_ball world coordinates: {red_ball_world}")
            count += 1


def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
def add_HSV_filter(frame, camera):

	# Blurring the frame
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # Converting RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Lower and upper limits for red ball
    l_b_r = np.array([0, 120, 70])
    u_b_r = np.array([10, 255, 255])
    l_b_l = np.array([0, 120, 70])
    u_b_l = np.array([10, 255, 255])

    if camera == 1:
        mask = cv2.inRange(hsv, l_b_r, u_b_r)
    else:
        mask = cv2.inRange(hsv, l_b_l, u_b_l)

    # Morphological Operation - Opening - Erode followed by Dilate - Remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask
    
def find_circles(frame, mask):

    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None

    # Only proceed if at least one contour was found
    if len(contours) > 0:
        # Find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)       #Finds center point
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Only proceed if the radius is greater than a minimum value
        if radius > 10:
            # Draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
            	(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 0), -1)


    return center
    
def find_depth(circle_right, circle_left, frame_right, frame_left, baseline,f, alpha):

    # CONVERT FOCAL LENGTH f FROM [mm] TO [pixel]:
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    if width_right == width_left:
        f_pixel = (width_right * 0.5) / np.tan(alpha * 0.5 * np.pi/180)

    else:
        print('Left and right camera frames do not have the same pixel width')

    x_right = circle_right[0]
    x_left = circle_left[0]

    # CALCULATE THE DISPARITY:
    disparity = x_left-x_right      #Displacement between left and right frames [pixels]

    # CALCULATE DEPTH z:
    zDepth = (baseline*f_pixel)/disparity             #Depth in [cm]

    return abs(zDepth)
    
def camera_to_world(camera_coor):
    camera_coor = np.array(camera_coor)
    # Translation vector
    t = np.array([3.450, -1.200, 2.345])

    # Euler angles in degrees
    rotz, rotx = np.radians([-90, 125])  # Convert to radians

    # Rotation matrices
    R_z = np.array([
        [np.cos(rotz), -np.sin(rotz), 0],
        [np.sin(rotz), np.cos(rotz), 0],
        [0, 0, 1]
    ])

    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(rotx), -np.sin(rotx)],
        [0, np.sin(rotx), np.cos(rotx)]
    ])

    # Combined rotation matrix
    R = R_z @ R_x 

    # Homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    world_coor = T @ camera_coor
    return world_coor


def angle_between_vectors(u, v):
    # Compute dot product
    dot_product = np.dot(u, v)
    
    # Compute magnitudes of the vectors
    magnitude_u = np.linalg.norm(u)
    magnitude_v = np.linalg.norm(v)
    
    # Calculate the cosine of the angle
    cos_theta = dot_product / (magnitude_u * magnitude_v)
    
    # To avoid numerical issues (rounding errors), clip the cosine value to the range [-1, 1]
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Calculate the angle in radians
    angle_rad = np.arccos(cos_theta)
    
    # Optionally, convert to degrees
    angle_deg = np.degrees(angle_rad)
    
    return angle_rad, angle_deg


def world_to_cylinder(world_coor, theta1):
    world_coor = np.array(world_coor)
    # Translation vector
    t = np.array([0, -1.2, 0])

    roty = np.pi/2 - theta1
    rotx = -1*np.pi/2

    # Rotation matrices
    R_y = np.array([
        [np.cos(roty), 0, np.sin(roty)],
        [0, 1, 0],
        [-np.sin(roty), 0, np.cos(roty)]
    ])
    
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(rotx), -np.sin(rotx)],
        [0, np.sin(rotx), np.cos(rotx)]
    ])

    # Combined rotation matrix
    R = R_y @ R_x 

    # Homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    cylinder_coor = T @ world_coor
    return cylinder_coor
    

def get_theta3(l1, l2, x, y):
    n = (l1**2 + l2**2)
    m = (x**2 + y**2)
    theta3 = np.arccos((m - n) / (2*l1*l2))
    return theta3

def get_theta2(l1, l2, x, y, theta3):
    a = y * (l1 + l2*np.cos(theta3))
    b = x * l2 * np.sin(theta3)
    c = x**2 + y**2
    theta2 = np.arcsin((a-b)/c)
    return theta2