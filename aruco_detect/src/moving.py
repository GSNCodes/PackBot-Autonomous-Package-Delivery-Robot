#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

PICKUP = {

    "x_pos": -0.8,
    "y_pos": -1.9,


    "z_orient": 0.3,
    "w_orient": 0.9
}

DROP_1 = {
    "x_pos": -1.73,
    "y_pos": -2.3,

    "z_orient": 1,
    "w_orient": -0.1
} 

DROP_2 = {
    "x_pos": -1.6,
    "y_pos": -3.2,

    "z_orient": -0.7,
    "w_orient": 0.8
}

DROP_3 = {
    "x_pos": -0.8,
    "y_pos": -3.2,

    "z_orient": 0.3,
    "w_orient": 1
} 


def movebase_client(x,y,z,w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    

    client.send_goal(goal)
    print("On the way!")
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:

        x = PICKUP["x_pos"]
        y = PICKUP["y_pos"]
        z = PICKUP["z_orient"]
        w = PICKUP["w_orient"]

        # = DROP_2["x_pos"]
        #y = DROP_2["y_pos"]
        #z = DROP_2["z_orient"]
        #w = DROP_2["w_orient"]
        rospy.init_node('movebase_client_py')
        result = movebase_client(x,y,z,w)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
