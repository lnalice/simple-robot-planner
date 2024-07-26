#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SimpleTraveler:
    def __init__(self):
        # Initialize ROS
        rospy.init_node('travel_node')

        # Move to target position
        """
        ** todo **
        [subscribe] msg 'get to goal' from task manager 
        [publish] msg 'I arrived!' to task manager
        """

        # Take a spin on the spot

        # Come back home (move to initial position)

    def move_action(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo('Waiting for the action server to start')
        client.wait_for_server()

        rospy.loginfo('Action server started, sending the goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # set position
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.z = 0.0

        # set orientation
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)

        rospy.loginfo('Waiting for the result')
        client.wait_for_result()

        if client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Hooray, I reached the goal')
        else:
            rospy.loginfo('The base failed to move for some reason')

if __name__ == "__main__":
    simple_traveler = SimpleTraveler()
    simple_traveler.move_action()
    rospy.spin()