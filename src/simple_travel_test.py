#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import argparse

class SimpleTraveler:
    def __init__(self, param):
        # Initialize ROS
        rospy.init_node('travel_node')

        self.robot_name = param.name

        # Move to target position
        rospy.Subscriber("/scene_manager/move_req", String, self.move_action, queue_size=1)
        self.move_res_pub = rospy.Publisher('scene_manager/move_res', String, queue_size=1)

        # Take a spin on the spot
        rospy.Subscriber("/scene_manager/ctrl_module_req", String, self.ctrl_module, queue_size=1)
        self.ctrl_module_res_pub =rospy.Publisher('/scene_manager/ctrl_module_res', String, queue_size=1)

        # Come back home (move to initial position)
        rospy.Subscriber("/scene_manager/go_home", String, self.go_home, queue_size=1)
        self.come_back_pub = rospy.Publisher('/scene_manager/come_back_home', String, queue_size=1)

    def move_action(self, req_data):

        ud_list = req_data.split(" ")
        id, pos_x, pos_y, ort_z, ort_w = ud_list

        if id != self.robot_name:
            return


        move_base_topic = self.robot_name + '/' + 'move_base'
        client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)

        rospy.loginfo('Waiting for the action server to start')
    
        client.wait_for_server()

        rospy.loginfo('Action server started, sending the goal')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = pos_x
        goal.target_pose.pose.position.y = pos_y
        goal.target_pose.pose.position.z = 0.0

        # set orientation
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = ort_z
        goal.target_pose.pose.orientation.w = ort_w
        """
        ** todo **
        make helper 'get_goal'
        """
        client.send_goal(goal)

        rospy.loginfo('Waiting for the result')
        client.wait_for_result()

        if client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Hooray, I reached the goal')
            self.move_res_pub.publish(req_data)

        else:
            rospy.loginfo('The base failed to move for some reason')

    """
        ** todo **
        ctrl_module() spin degree using given message
        go_home() go to initial position using given message
    """
    def ctrl_module():
        pass

    def go_home(): #i think i don't need it,,
        pass

parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-n', '--name', required=True, help='Robot Name (multi-robot)') # ex) tb3_1
args = parser.parse_args()

if __name__ == "__main__":
    simple_traveler = SimpleTraveler(param=args)
    #simple_traveler.move_action() #test
    rospy.spin()