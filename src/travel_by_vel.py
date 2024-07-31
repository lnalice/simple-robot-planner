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
    
    def move_by_vel(self, robot_name, seconds: int, lin_vel: tuple, ang_vel:tuple):

        self.cmd_vel_topic = robot_name + '/cmd_vel' #multi robot
        pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        
        start_time = rospy.Time.now().to_sec()
        while(rospy.Time.now().to_sec() - start_time < seconds):
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = lin_vel
            twist.angular.x, twist.angular.y, twist.angular.z = ang_vel
            
            rospy.sleep(0.1)

            pub.publish(twist)
            rospy.loginfo("currently:\tlinear vel %s\t angular vel %s " % (lin_vel, ang_vel))

    def move_action(self, req_data):

        ud_list = str(req_data.data).split(" ")
        id = ud_list[0]

        if id != self.robot_name:
            rospy.loginfo("this id(%s) is not mine.", id)
            return
        seconds = int(ud_list[1])
        lin_vel = tuple(float(e) for e in ud_list[2:5])
        ang_vel = tuple(float(e) for e in ud_list[5:])

        print("robot name is " + id)
        rospy.loginfo("[RobotPlanner-%s] now this robot is moving...", id)
        try:
            self.move_by_vel(id, seconds, lin_vel, ang_vel)

            self.move_by_vel(id, 10, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

            self.move_res_pub.publish(req_data)

        except:
            print("Failed!")

        # finally:
        #     self.move_by_vel(id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


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