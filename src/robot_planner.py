#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import os
import math

from publisher.cmdVelPub import moveByVel
from publisher.ctrlModulePub import ctrlByPos

from publisher.blinkLedPub import blinkLed

from helper.getLedStatus import vel2statusByte

STOP_SECONDS = 2
SPIN_ONCE_SEC = 10
SPIN_ONCE_LIN = (0, 0, 0)
SPIN_ONCE_ANG = (0, 0, 2*math.pi / SPIN_ONCE_SEC)

DIR_ADJUSTMENT_SEC = 3
COMEBACK_VEL = 0.15

class RobotPlanner:
    def __init__(self):

        self.robot_name = os.getenv('ROBOT_NAME', 'tb3_#')

        rospy.loginfo("**Robot name is... " + self.robot_name)

        rospy.init_node('robot_planner_node_'+ self.robot_name)

        # rotation recovery (for localization): rotation once
        # moveByVel(self.robot_name, SPIN_ONCE_SEC, SPIN_ONCE_LIN, SPIN_ONCE_ANG)

        # Move to target position
        rospy.Subscriber("task_manager/move_req", String, self.move_action, queue_size=1)
        self.move_res_pub = rospy.Publisher('task_manager/move_res', String, queue_size=1)

        # Take a spin on the spot
        rospy.Subscriber("task_manager/ctrl_module_req", String, self.ctrl_module, queue_size=1)
        self.ctrl_module_res_pub =rospy.Publisher('task_manager/ctrl_module_res', String, queue_size=1)

        rospy.loginfo('[RobotPlanner-%s] I\'m ready!', self.robot_name)

    """
    @goal_data seconds linX angZ delay
    """
    def move_action(self, goal_data):
        
        goal_tuple = tuple(str(goal_data.data).split(" "))

        # /cmd_vel
        seconds, linX, angZ, delay = goal_tuple
            
        if float(delay) > 0:
            rospy.logwarn("[RobotPlanner-%s] Delay time is %s.\n", self.robot_name, delay)
            rospy.sleep(int(delay))
            
        rospy.logwarn("[RobotPlanner-%s] now this robot is moving...\n", self.robot_name)
        try:
            blinkLed(self.robot_name, vel2statusByte(float(linX), float(angZ))) # blink LED
            moveByVel(self.robot_name, int(seconds), float(linX), float(angZ)) # move using cmd_vel
                
            # <recovery> direction adjustment using cmd_vel
            if float(angZ) != 0:
                _angZ = (-float(angZ) * seconds) / DIR_ADJUSTMENT_SEC
                moveByVel(self.robot_name, DIR_ADJUSTMENT_SEC, 0.0, _angZ)

        except:
            rospy.logerr("[RobotPlanner-%s] Failed! (/cmd_vel)", self.robot_name)

        finally:
            moveByVel(self.robot_name, STOP_SECONDS, 0.0, 0.0)
            self.move_res_pub.publish(goal_data)
        
    """
    @goal_data degZ degX delay
        - degZ : vertical degree
        - degX : horizon degree
    """
    def ctrl_module(self, goal_data):

        req_list = tuple(str(goal_data.data).split(" "))
        degZ, degX, delay = req_list

        rospy.sleep(0.1)

        rospy.sleep(int(delay))
        rospy.logwarn("[RobotPlanner-%s] now this robot will be soon controlling the module...\n\n", self.robot_name )

        try:
            blinkLed(self.robot_name, vel2statusByte(0,0)) # blink LED
            ctrlByPos(self.robot_name, float(degZ), float(degX)) # control module
        except:
            rospy.logerr("[RobotPlanner-%s] Failed! (/module_pos)", self.robot_name )
            ctrlByPos(self.robot_name, 0.0, 0.0)
        finally:
            self.ctrl_module_res_pub.publish(goal_data)

if __name__ == "__main__":
    robot_planner = RobotPlanner()

    rospy.spin()