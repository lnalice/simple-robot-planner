#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import argparse

SECONDS= 17
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-c', '--command', required=False,
        help='command [MOVE, HOME, MODULE, SCENE]')
    parser.add_argument(
        '-n', '--names', required=False, nargs='+', help='Robot Name (multi-robot)') # ex) tb3_1
    parser.add_argument(
        '-s', '--scene', required=False, help='Scene ID')
    args = parser.parse_args()
    
    rospy.init_node("test_publisher")
    
    """
    {turtlebot_name}/module_vel
    """
    # test_topic = args.name + '/module_vel'
    # pub = rospy.Publisher(test_topic, Float64, queue_size=1)

    # start_time = rospy.Time.now().to_sec()

    # t = 0
    # while(rospy.Time.now().to_sec() - start_time < SECONDS):
    #     msg = Float64(-0.03)
    #     pub.publish(msg)
    #     rospy.loginfo(msg)
    #     t += 1

    # t = 0
    # while(t <= 10):
    #     msg = Float64(0.0)
    #     pub.publish(msg)

    #     rospy.loginfo(msg)
    #     t += 1

    """
    /react/commander
    """
    test_topic = '/react/commander'
    pub = rospy.Publisher(test_topic, String, queue_size=1)

    command = args.command.upper()
    scene_id = args.scene
    IDs = ' '.join(args.names)
    
    msg = command + " " + scene_id + " " + IDs

    rospy.sleep(0.1)

    pub.publish(msg)
    rospy.loginfo("I sent topic \"%s\"", msg)