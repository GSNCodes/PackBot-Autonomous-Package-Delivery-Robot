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
from geometry_msgs.msg import PoseStamped
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global size_of_marker 
global mtx
global dist
moving = None

bridge = CvBridge()
img_pub = rospy.Publisher('/usb_cam/image_aruco',Image, queue_size=5)

#Point Dictionaries

PICKUP = {
    "x_pos": -0.9,
    "y_pos": -1.9
}

DROP_1 = {
    "x_pos": -1.96,
    "y_pos": -2.26
} 

DROP_2 = {
    "x_pos": -1.57,
    "y_pos": -3.42
}

DROP_3 = {
    "x_pos": -0.54,
    "y_pos": -3.13
} 


# all tags supported by cv2
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_6X6_250"])
arucoParams = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.CharucoBoard_create(6, 4, 1, 0.8, arucoDict)

def main():
    rospy.init_node('aruco_moving',anonymous = True)
    sub = rospy.Subscriber('/usb_cam/image_raw',Image,image_callback,queue_size=1)
    rospy.spin()


def image_callback(msg:Image):
    x = 0
    global moving

    frame = bridge.imgmsg_to_cv2(msg)

    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)
    
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        i = 0
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the frame
            cv2.putText(frame, str(markerID),
                (topLeft[0], topLeft[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv2.putText(frame, str(int(np.linalg.norm(tvecs[i]))) + "cm",
                (topLeft[0], topLeft[1] - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)
            
            i += 1


    if tvecs is not None:

        # Remove the redundant dimension, for example 8x1x3 --> 8x3
        temp_vec = np.reshape(tvecs,(tvecs.shape[0],tvecs.shape[2]))

        # Get an array of all block distances
        temp_vec_norm = np.linalg.norm(temp_vec,axis = 1)

        # Find index of closest aruco and match with its ID
        closest_id = ids[np.argmin(temp_vec_norm)]

        #print("The closest id is: ",closest_id, " and is ", np.min(temp_vec_norm), " cm away")
        closest_tvec = temp_vec[np.argmin(temp_vec_norm)]

        #Convert into ROS form: [x y z] --> [z -x y]
        ROS_closest_tvec = np.array([closest_tvec[2], -closest_tvec[0], closest_tvec[1]])/100
        ROS_closest_tvec_rounded = np.round(ROS_closest_tvec, decimals=2)
        x = ROS_closest_tvec[0]
        y = ROS_closest_tvec[1]


    imgMsg = bridge.cv2_to_imgmsg(frame, encoding = 'bgr8')

    img_pub.publish(imgMsg)
    

    # If the closest block is less greater than 30 cm, the TBOT will go towards it 
    if x > 0.3 and moving == None:
        moving  = 1
        my_id = closest_id
        result1 = movebase_client(x, 0)
        if result1:
            rospy.loginfo("Package Picked up!")

        if my_id == 1:
            result2 = movebase_point(DROP_1["x_pos"],DROP_1["y_pos"])
            if result2:
                rospy.loginfo("Package Dropping Off!")

        elif my_id == 2:
            result2 = movebase_point(DROP_2["x_pos"],DROP_32"y_pos"])
            if result2:
                rospy.loginfo("Package Dropping Off!")

        elif my_id == 3:
            result2 = movebase_point(DROP_3["x_pos"],DROP_3["y_pos"])
            if result2:
                rospy.loginfo("Package Dropping Off!")

        result3 = movebase_point(PICKUP["x_pos"],PICKUP["y_pos"])
            if result3:
                rospy.loginfo("Package Dropping Off!")
                   moving = None
 

def movebase_client(x, y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    print("On the way!")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def movebase_point(x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    print("On the way!")
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':

    dist = np.load("/home/bharath/catkin_ws/src/aruco_detect/src/dist.npy")
    mtx = np.load("/home/bharath/catkin_ws/src/aruco_detect/src/mtx.npy")
    
    size_of_marker = 2.8
    main()
