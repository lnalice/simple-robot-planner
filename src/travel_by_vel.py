#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import argparse

from publisher.cmdVelPub import moveByVel

STOP_SECONDS = 5

"""
**resource code from turtlebot3_teleop**
https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop
"""

class SimpleTraveler:
    def __init__(self, param):
        # Initialize ROS

        self.robot_name = param.name
        self.total_robot_num = 5
        self.cmd_vel_topic = 'cmd_vel' # only one robot

        self.node_name = 'robot_planner_node_' + self.robot_name
        rospy.init_node(self.node_name)

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

        ud_list = str(req_data.data).split(" ")
        req_id = ud_list[0]
        seconds = int(ud_list[1])
        lin_vel = tuple(float(e) for e in ud_list[2:5])
        ang_vel = tuple(float(e) for e in ud_list[5:])

        if req_id != self.robot_name:
            rospy.loginfo("this ID(%s) is not mine.", req_id)
            return

        rospy.loginfo("[RobotPlanner-%s] now this robot is moving...", req_id)
        try:
            moveByVel(req_id, seconds, lin_vel, ang_vel)
            self.move_res_pub.publish(req_data)

        except:
            rospy.logerr("[RobotPlanner-%s] Failed! (/cmd_vel)", req_id)

        finally:
            moveByVel(req_id, STOP_SECONDS, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

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