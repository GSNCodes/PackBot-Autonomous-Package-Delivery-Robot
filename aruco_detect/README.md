This is a package which allows for aruco detection from a USB camera feed
and publishes the labeled image to /usb_cam/image_aruco.

1. Copy the entire aruco_detect folder into your ~/catkin_ws/src directory
2. cd ~/catkin_ws
3. catkin_make
4. Do roscore on your remote PC
5. ssh into pi and activate usb cam node
6. Open another terminal on remote PC and do:
   "rosrun aruco_detect aruco_pose2.py"  
