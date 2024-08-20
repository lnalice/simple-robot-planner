import rospy
from std_msgs.msg import Float64

def ctrlByVel(robot_id: str, seconds: int, vel_data: float) -> bool:

        ctrl_module_topic = 'module_vel'

        pub = rospy.Publisher(ctrl_module_topic, Float64, queue_size=1)
        msg = Float64(vel_data)

        start_time = rospy.Time.now().to_sec()
        
        # if vel_data != 0:
            # rospy.logwarn('[RobotPlanner-%s] Now the module will be moving', id)

        while(rospy.Time.now().to_sec() - start_time < seconds):
            pub.publish(msg)
            rospy.loginfo("currently:\tvel_data %f " %vel_data)
        
        rospy.loginfo('[RobotPlanner-%s] Hooray, I controlled the module', id)

        return True

# todo: return value