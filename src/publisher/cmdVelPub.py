import rospy
from geometry_msgs.msg import Twist

def moveByVel(robot_id: str, seconds: int, lin_vel: tuple, ang_vel:tuple) -> None:
    
    cmd_vel_topic = robot_id + '/cmd_vel' #multi robot
    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    start_time = rospy.Time.now().to_sec()

    while(rospy.Time.now().to_sec() - start_time < seconds):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = lin_vel
        twist.angular.x, twist.angular.y, twist.angular.z = ang_vel
            
        rospy.sleep(0.1)
            
        rospy.loginfo("currently:\tlinear vel %s\t angular vel %s " % (lin_vel, ang_vel))
        pub.publish(twist)