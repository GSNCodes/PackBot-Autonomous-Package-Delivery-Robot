#!/usr/bin/env python3
# license removed for brevity
import argparse
import imutils
from imutils.video import VideoStream
import time
import cv2
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Twist
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global size_of_marker 
global mtx
global dist


bridge = CvBridge()
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
img_pub = rospy.Publisher('/usb_cam/image_aruco',Image, queue_size=1)

# all tags supported by cv2
ARUCO_DICT = {

	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_6X6_250"])
arucoParams = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.CharucoBoard_create(6, 4, 1, 0.8, arucoDict)
def main():
    rospy.init_node('align',anonymous = True)
    sub = rospy.Subscriber('/usb_cam/image_raw',Image,image_callback,queue_size=1)
    rospy.spin()


def image_callback(msg:Image):
    frame = bridge.imgmsg_to_cv2(msg)

    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)
    
    # # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
    #     # flatten the ArUco IDs list
        ids = ids.flatten()
    #      # loop over the detected ArUCo corners
    #     i = 0
    #     for (markerCorner, markerID) in zip(corners, ids):
    #         # extract the marker corners (which are always returned
    #         # in top-left, top-right, bottom-right, and bottom-left
    #         # order)
    #         corners = markerCorner.reshape((4, 2))
    #         (topLeft, topRight, bottomRight, bottomLeft) = corners
    #         # convert each of the (x, y)-coordinate pairs to integers
    #         topRight = (int(topRight[0]), int(topRight[1]))
    #         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    #         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    #         topLeft = (int(topLeft[0]), int(topLeft[1]))
    #         # draw the bounding box of the ArUCo detection
    #         cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
    #         cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
    #         cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
    #         cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
    #         # compute and draw the center (x, y)-coordinates of the
    #         # ArUco marker
    #         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    #         cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    #         cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
    #         # draw the ArUco marker ID on the frame
    #         cv2.putText(frame, str(markerID),
    #             (topLeft[0], topLeft[1] - 15),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.5, (0, 255, 0), 2)
    #         cv2.putText(frame, str(int(np.linalg.norm(tvecs[i]))) + "cm",
    #             (topLeft[0], topLeft[1] - 30),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.5, (0, 0, 255), 2)
            
    #         i += 1 


        # Remove the redundant dimension, for example 8x1x3 --> 8x3
        temp_vec = np.reshape(tvecs,(tvecs.shape[0],tvecs.shape[2]))

        # Get an array of all block distances
        temp_vec_norm = np.linalg.norm(temp_vec,axis = 1)

        # Find index of closest aruco and match with its ID
        closest_id = ids[np.argmin(temp_vec_norm)]
        print("Closest id:", closest_id)
        #print("The closest id is: ",closest_id, " and is ", np.min(temp_vec_norm), " cm away")
        closest_tvec = temp_vec[np.argmin(temp_vec_norm)]

        #Convert into ROS form: [x y z] --> [z -x y]
        ROS_closest_tvec = np.array([closest_tvec[2], -closest_tvec[0], closest_tvec[1]])/100
        ROS_closest_tvec_rounded = np.round(ROS_closest_tvec, decimals=2)
        x_ros = ROS_closest_tvec[0]
        y_ros = ROS_closest_tvec[1]
        
        imgMsg = bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')

        img_pub.publish(imgMsg)

        # Align
        angle_off = np.rad2deg(np.arctan2(y_ros,x_ros))
        print(angle_off)
        if angle_off > 3:
            vel = Twist()
            vel.angular.z = 0.1
            vel_pub.publish(vel)
        elif angle_off < -3:
            vel = Twist()
            vel.angular.z = -0.1
            vel_pub.publish(vel)
        else:
            vel = Twist()
            vel.angular.z = 0
            vel_pub.publish(vel)

            if x_ros > 0.15:
                vel = Twist()
                vel.linear.x = 0.1
                vel_pub.publish(vel)
            else:
                print("Dropped off!")
                vel = Twist()
                vel.linear.x = 0
                vel_pub.publish(vel)

if __name__ == '__main__':

    dist = np.load("/home/bharath/catkin_ws/src/aruco_detect/src/dist.npy")
    mtx = np.load("/home/bharath/catkin_ws/src/aruco_detect/src/mtx.npy")
    
    size_of_marker = 2.2
    main()
