import sys
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

# Functions
import HSV_filter as hsv
import shape_recognition as shape
import triangulation as tri
#import calibration as calib


# Open both cameras
cap_right = sim.getObject("/visionSensor[1]")                   
cap_left =  sim.getObject("/visionSensor[0]")   

#frame_rate = 120    #Camera frame rate (maximum at 120 fps)

B = 9               #Distance between the cameras [cm]
f = (np.sqrt(3)/40)*100   #Camera lense's focal length [mm] (sensor_size/(2*tan(alpha/2))) = 4.330127
alpha = 60        #Camera field of view in the horisontal plane [degrees]


#Initial values
count = 1

frame_right, resR = sim.getVisionSensorImg(cap_right,0, 0.0,[0, 0],[0, 0])
frame_left, resL = sim.getVisionSensorImg(cap_left,0, 0.0,[0, 0],[0, 0])

################## CALIBRATION #########################################################
# Camera parameters to undistort and rectify images
# cv_file = cv2.FileStorage()
# cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

# stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
# stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
# stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
# stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# # Undistort and rectify images
# frame_right = cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
# frame_left = cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                    
# Show the frames
cv2.imshow("frame right", frame_right) 
cv2.imshow("frame left", frame_left)

########################################################################################

# If cannot catch any frame, break
if resR==0 or resL==0:                    
    print("ERROR1: No image returned")

else:
    # APPLYING HSV-FILTER:
    mask_right = hsv.add_HSV_filter(frame_right, 1)
    mask_left = hsv.add_HSV_filter(frame_left, 0)

    # Result-frames after applying HSV-filter mask
    res_right = cv2.bitwise_and(frame_right, frame_right, mask=mask_right)
    res_left = cv2.bitwise_and(frame_left, frame_left, mask=mask_left) 

    # APPLYING SHAPE RECOGNITION:
    circles_right = shape.find_circles(frame_right, mask_right)
    circles_left  = shape.find_circles(frame_left, mask_left)

    # Hough Transforms can be used aswell or some neural network to do object detection


    ################## CALCULATING BALL DEPTH #########################################################

    # If no ball can be caught in one camera show text "TRACKING LOST"
    if np.all(circles_right) == None or np.all(circles_left) == None:
        cv2.putText(frame_right, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
        cv2.putText(frame_left, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

    else:
        # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
        # All formulas used to find depth is in video presentaion
        depth = tri.find_depth(circles_right, circles_left, frame_right, frame_left, B, f, alpha)

        cv2.putText(frame_right, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
        cv2.putText(frame_left, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
        cv2.putText(frame_right, "Distance: " + str(round(depth,3)), (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
        cv2.putText(frame_left, "Distance: " + str(round(depth,3)), (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
        # Multiply computer value with 205.8 to get real-life depth in [cm]. The factor was found manually.
        print("Depth: ", depth)                                            


    # Show the frames
    cv2.imshow("frame right", frame_right) 
    cv2.imshow("frame left", frame_left)
    cv2.imshow("mask right", mask_right) 
    cv2.imshow("mask left", mask_left)


    # Hit "q" to close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("EXIT")


# Release and destroy all windows before termination
cap_right.release()
cap_left.release()

cv2.destroyAllWindows()