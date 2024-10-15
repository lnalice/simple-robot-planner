import rospy
from std_msgs.msg import Byte

def blinkLed(robot_id: str, led_status: int) -> bool:

        blink_led_topic = 'blink_led'

        pub = rospy.Publisher(blink_led_topic, Byte, queue_size=1)
        msg = Byte()

        msg.data = led_status

        rospy.sleep(0.1)

        pub.publish(msg)
        
        rospy.loginfo('[RobotPlanner-%s] I would blink LEDs', robot_id)

        return True

