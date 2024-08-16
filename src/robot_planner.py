#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import argparse

import math

from publisher.moveBasePub import moveByBase
from publisher.cmdVelPub import moveByVel
from publisher.ctrlModulePub import ctrlByVel

STOP_SECONDS = 2
SPIN_ONCE_SEC = 10
SPIN_ONCE_LIN = (0, 0, 0)
SPIN_ONCE_ANG = (0, 0, 2*math.pi / SPIN_ONCE_SEC)
DIR_ADJUSTMENT_SEC = 3

class Traveler:
    def __init__(self, param):

        self.robot_name = param.name
        # self.robot_name = rospy.get_param("~robot_name")
        self.total_robot_num = 5

        rospy.init_node('robot_planner_node_'+ self.robot_name)

        #rotation recovery (for localization): rotation once
        moveByVel(self.robot_name, SPIN_ONCE_SEC, SPIN_ONCE_LIN, SPIN_ONCE_ANG)

        # Move to target position
        rospy.Subscriber("/scene_manager/move_req", String, self.move_action, queue_size=1)
        self.move_res_pub = rospy.Publisher('scene_manager/move_res', String, queue_size=1)

        # Take a spin on the spot
        rospy.Subscriber("/scene_manager/ctrl_module_req", String, self.ctrl_module, queue_size=1)
        self.ctrl_module_res_pub =rospy.Publisher('/scene_manager/ctrl_module_res', String, queue_size=1)

        # Come back home (move to initial position)
        rospy.Subscriber("/scene_manager/go_home", String, self.go_home, queue_size=1)
        self.come_back_pub = rospy.Publisher('/scene_manager/come_back_home', String, queue_size=1)

        rospy.loginfo('[RobotPlanner-%s] I\'m ready!', self.robot_name)

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
            
            rospy.sleep(int(delay_sec))
            rospy.loginfo("[RobotPlanner-%s] now this robot is moving...\n\n", req_id)

            try:
                moveByVel(self.robot_name, seconds, lin_vel, ang_vel)
                # direction adjustment using cmd_vel
                if lin_vel[0] != 0 and ang_vel[-1] != 0:
                    _ang_vel = (-ang_vel[-1] * seconds) / DIR_ADJUSTMENT_SEC
                    moveByVel(self.robot_name, DIR_ADJUSTMENT_SEC, (0.0, 0.0, 0.0), (0.0, 0.0, _ang_vel))

            except:
                rospy.logerr("[RobotPlanner-%s] Failed! (/cmd_vel)", req_id)

            finally:
                moveByVel(self.robot_name, STOP_SECONDS, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                self.move_res_pub.publish(req_data)
        
    def ctrl_module(self, req_data):

        req_list = tuple(str(req_data.data).split(" "))
        req_id, ctrl_sec, ctrl_vel, delay_sec = req_list

        if req_id != self.robot_name:
            rospy.loginfo("[RobotPlanner-%s] This ID(%s) is not mine.", self.robot_name, req_id)
            return
        
        rospy.sleep(0.1)

        rospy.sleep(int(delay_sec))
        rospy.loginfo("[RobotPlanner-%s] now this robot will be soon controlling the module...\n\n", req_id)

        try:
            ctrlByVel(self.robot_name, int(ctrl_sec), float(ctrl_vel))
        except:
            rospy.logerr("[RobotPlanner-%s] Failed! (/module_vel)", req_id)
        
        finally:
            ctrlByVel(self.robot_name, STOP_SECONDS, 0.0)
            self.move_res_pub.publish(req_data)
        

        self.ctrl_module_res_pub.publish(req_data)

    def go_home():
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-n', '--name', required=True, help='Robot Name (multi-robot)') # ex) tb3_1
    args = parser.parse_args()
    
    simple_traveler = Traveler(param=args)
    # simple_traveler = Traveler()
    rospy.spin()