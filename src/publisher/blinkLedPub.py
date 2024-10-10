import rospy
from std_msgs.msg import Float64MultiArray

def blinkLed(robot_id: str, linX: float, angZ: float, deg1: float, deg2: float) -> bool:

        blink_led_topic = 'blink_led'

        pub = rospy.Publisher(blink_led_topic, Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()

        msg.data = [linX, angZ, deg1, deg2]

        rospy.sleep(0.1)

        pub.publish(msg)
        
        rospy.loginfo('[RobotPlanner-%s] I would blink LEDs', robot_id)

        return True

