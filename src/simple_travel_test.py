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

        self.robot_name = param.name
        self.total_robot_num = 5

        rospy.init_node('travel_node'+self.robot_name)

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
        print()

        ud_list = tuple(str(req_data.data).split(" "))
        id, pos_x, pos_y, ort_z, ort_w = ud_list
        print("id"+id)

        if id != self.robot_name:
            return
        rospy.sleep(2)
        # self.move_res_pub.publish(req_data.data) #test

        # move_base_topic = self.robot_name + '/' + 'move_base'
        move_base_topic = '/move_base' # only one robot
        client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)

        rospy.loginfo('[RobotPlanner-%s] Waiting for the action server to start', id)
    
        client.wait_for_server()

        rospy.loginfo('[RobotPlanner-%s] Action server started, sending the goal', id)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = float(pos_x)
        goal.target_pose.pose.position.y = float(pos_y)
        goal.target_pose.pose.position.z = 0.0

        # set orientation
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = float(ort_z)
        goal.target_pose.pose.orientation.w = float(ort_w)
        """
        ** todo **
        make helper 'get_goal'
        """
        client.send_goal(goal)

        rospy.loginfo('[RobotPlanner-%s] Waiting for the result',id)
        client.wait_for_result()

        if client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('[RobotPlanner-%s] Hooray, I reached the goal', id)
            self.move_res_pub.publish(req_data)

        else:
            rospy.loginfo('[RobotPlanner-%s] The base failed to move for some reason', id)

    """
        ** todo **
        ctrl_module() spin degree using given message
        go_home() go to initial position using given message
    """
    def ctrl_module():
        pass

    def go_home():  #i think i don't need it,,
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