import rospy
from std_msgs.msg import Float64

def ctrlByVel(robot_id: str, vel_data: float, seconds: int) -> bool:

        ctrl_module_topic = robot_id + '/' + 'module_vel'

        pub = rospy.Publisher(ctrl_module_topic, Float64, queue_size=1)
        msg = Float64(vel_data)

        start_time = rospy.Time.now().to_sec()
        
        rospy.loginfo('[RobotPlanner-%s] Now the module will be moving', id)

        while(rospy.Time.now().to_sec() - start_time < seconds):
            pub.publish(msg)
            rospy.loginfo('[RobotPlanner-%s] Hooray, I controlled the module', id)
        
        pub.publish(Float64(0.0))
        # t = 0
        # while(t <= 5):
        #     pub.publish(Float64(0.0))
        #     t += 1

        # try:
        #     while(rospy.Time.now().to_sec() - start_time < seconds):
        #         pub.publish(msg)
        #     rospy.loginfo('[RobotPlanner-%s] Hooray, I controlled the module', id)

        # except:
        #      rospy.logerr('[RobotPlanner-%s] Something wrong..', id)

        # finally:
        #      t = 0
        #      while(t <= 10):
        #           msg = Float64(0.0)
        #           pub.publish(msg)

        #           t += 1

        return True

# todo: return value