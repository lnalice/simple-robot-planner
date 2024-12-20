import rospy
from std_msgs.msg import Float64, Float64MultiArray

def ctrlByVel(robot_id: str, seconds: int, vel_data: float) -> bool:

        ctrl_module_topic = 'module_vel'

        pub = rospy.Publisher(ctrl_module_topic, Float64, queue_size=1)
        msg = Float64(vel_data)

        start_time = rospy.Time.now().to_sec()

        while(rospy.Time.now().to_sec() - start_time < seconds):
            pub.publish(msg)
            rospy.loginfo("currently:\tvel_data %f " %vel_data)
        
        rospy.loginfo('[RobotPlanner-%s] Hooray, I controlled the module', robot_id)

        return True


def ctrlByPos(robot_id: str, degZ: float, degX: float) -> bool:
      
      ctrl_module_topic = 'module_pos'

      pub = rospy.Publisher(ctrl_module_topic, Float64MultiArray, queue_size=1)

      msg = Float64MultiArray()
      msg.data = [degZ, degX]

      rospy.sleep(0.1)

      pub.publish(msg)

      rospy.loginfo('[RobotPlanner-%s] Module will be controlled using data(degree) %s', robot_id, str(msg.data))

      return True
