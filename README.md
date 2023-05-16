# CSCI-5551-PackBot

## About

This repo contains the code for our final project for CSCI-5551: Introduction to Intelligent Robotic Systems.  
The contributors are Bharath Sivaram , Quinn Koenig, and Sowmiya Govindaraj

<https://user-images.githubusercontent.com/20588623/221689471-ae83cba7-4e35-40a6-9141-d6337fea42c7.mp4>

## Objective

In this project, the Turtlebot3 will be used to drop-off small labeled blocks from a single
pick-up location to their respective drop-off points within a maze. We simulate a warehouse environment using a cardboard maze. The dropoff points are marked by ArUCo Markers. Once the robot recieves a package marked with a ArUCo marker at the pick-up point, it then navigates to the respective drop-off point by matching with the detected ArUCo ID.

## Hardware

- TurtleBot3
  - RaspberryPi 3
  - Lidar
  - DYNAMIXEL Motors
  - OpenCR Board
- Logitech USB Webcam
- Laptop

## Software

- ROS
- Python
- Ubuntu 20.04

## How to Run

- First map the enironment using SLAM by tele-operating the TurtleBot .
- Make sure to save the map using the command `rosrun map_server map_saver -f map`
- Set the pick-up and drop-off points in the map manually.
- Each location was saved within a Python dictionary, and orientation is in quaternion form.
- Each of these locations would have ArUCo markers on the walls.
- Follow the `README.md` provided within `aruco_detect` to initialize the all the required nodes.

For more details regarding the inner workings, refer to `PackBot_Report.pdf`
