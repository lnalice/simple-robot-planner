import rospy
from geometry_msgs.msg import Twist

def moveByVel(robot_id: str, seconds: int, linX: float, angZ: float) -> None:
    
    cmd_vel_topic = 'cmd_vel'

    pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    
    start_time = rospy.Time.now().to_sec()

    lin_vel = (linX, 0.0, 0.0)
    ang_vel = (0.0, 0.0, angZ)

    while(rospy.Time.now().to_sec() - start_time < seconds):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = lin_vel
        twist.angular.x, twist.angular.y, twist.angular.z = ang_vel
            
        rospy.sleep(0.1)
            
        rospy.loginfo("currently:\tlinear vel %s\t angular vel %s " % (lin_vel, ang_vel))
        pub.publish(twist)