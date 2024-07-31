#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import String
import argparse

"""
**resource code from turtlebot3_teleop**
https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop
"""

class SimpleTraveler:
    def __init__(self, param):
        # Initialize ROS

        self.robot_name = param.name
        self.total_robot_num = 5
        self.cmd_vel_topic = '/cmd_vel' # only one robot

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
    

    def vels(target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)
    
    def move_by_vel(self, robot_name, lin_vel: tuple, ang_vel:tuple):

        # self.cmd_vel_topic = robot_name + '/' + 'cmd_vel' #multi robot
        self.cmd_vel_topic = '/cmd_vel' # only one robot
        
        pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = lin_vel
        twist.angular.x, twist.angular.y, twist.angular.z = ang_vel

        pub.publish(twist)
        
        rospy.loginfo(self.vels(lin_vel, ang_vel))


    def move_action(self, req_data):

        ud_list = str(req_data.data).split(" ")
        id = ud_list[0]

        if id != self.robot_name:
            return
        
        lin_vel = tuple(float(e) for e in ud_list[1:4])
        ang_vel = tuple(float(e) for e in ud_list[4:])

        print("robot name is " + id)
        rospy.loginfo("[RobotPlanner-%s] now this robot is moving...", id)
        try:
            self.move_by_vel(id, lin_vel, ang_vel)

            rospy.sleep(5)

            self.move_by_vel(id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

            self.move_res_pub.publish(req_data)

        except:
            print("Failed!")

        finally:
            self.move_by_vel(id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        rospy.loginfo('[RobotPlanner-%s] Waiting for the action server to start', id)
    
        rospy.loginfo('[RobotPlanner-%s] Action server started, sending the goal', id)


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