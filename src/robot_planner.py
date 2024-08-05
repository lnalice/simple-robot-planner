#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import argparse

from publisher.moveBasePub import moveByBase
from publisher.cmdVelPub import moveByVel

STOP_SECONDS = 5
SPIN_ONCE_SEC = 10.3
SPIN_ONCE_LIN = (0, 0, 0)
SPIN_ONCE_ANG = (0, 0, 0.60)

class Traveler:
    def __init__(self, param):
        # Initialize ROS

        self.robot_name = param.name
        self.total_robot_num = 5

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

        req_list = tuple(str(req_data.data).split(" "))
        req_id = req_list[0]

        if req_id != self.robot_name:
            rospy.loginfo("[RobotPlanner-%s] This id(%s) is not mine.", self.robot_name, req_id)
            return
        
        rospy.sleep(0.1)

        # /move_base
        if len(req_list) <= 5:
            pos_x, pos_y, ort_z, ort_w = req_list[1:]
            if moveByBase(self.robot_name, (pos_x, pos_y), (ort_z, ort_w)):
                self.move_res_pub.publish(req_data.data)
            else:
                rospy.logerr("[RobotPlanner-%s] Failed! (/move_base)", req_id)

        # /cmd_vel
        else: 
            seconds = int(req_list[1])
            lin_vel = tuple(float(e) for e in req_list[2:5])
            ang_vel = tuple(float(e) for e in req_list[5:8])
            delay_sec = float(req_list[8])

            if req_id != self.robot_name:
                rospy.loginfo("this ID(%s) is not mine.", req_id)
                return
            
            rospy.sleep(max(delay_sec-3, 0))
            rospy.loginfo("[RobotPlanner-%s] now this robot is moving...\n\n", req_id)
            rospy.sleep(3.)

            try:
                moveByVel(self.robot_name, seconds, lin_vel, ang_vel)
                self.move_res_pub.publish(req_data)

            except:
                rospy.logerr("[RobotPlanner-%s] Failed! (/cmd_vel)", req_id)

            finally:
                moveByVel(self.robot_name, STOP_SECONDS, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        
    def ctrl_module():
        pass

    def go_home():
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-n', '--name', required=True, help='Robot Name (multi-robot)') # ex) tb3_1
    args = parser.parse_args()
    
    rospy.init_node('robot_planner_node_'+ args.name)

    #rotation recovery (for localization): rotation once
    moveByVel(args.name, SPIN_ONCE_SEC, SPIN_ONCE_LIN, SPIN_ONCE_ANG)

    simple_traveler = Traveler(param=args)
    rospy.spin()