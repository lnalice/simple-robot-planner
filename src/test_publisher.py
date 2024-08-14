#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import argparse

SECONDS= 4
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-n', '--name', required=True, help='Robot Name (multi-robot)') # ex) tb3_1
    args = parser.parse_args()
    rospy.init_node("test_publisher")
    
    test_topic = args.name + '/module_vel'
    pub = rospy.Publisher(test_topic, Float64, queue_size=1)

    start_time = rospy.Time.now().to_sec()

    t = 0
    while(rospy.Time.now().to_sec() - start_time < SECONDS):
        msg = Float64(0.03)
        pub.publish(msg)
        rospy.loginfo(msg)
        t += 1

    t = 0
    while(t <= 10):
        msg = Float64(0.0)
        pub.publish(msg)

        rospy.loginfo(msg)
        t += 1