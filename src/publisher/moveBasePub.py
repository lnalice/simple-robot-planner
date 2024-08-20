import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def moveByBase(robot_id: str, pos_data: tuple, ort_data: tuple) -> bool:
        pos_x, pos_y = pos_data
        ort_z, ort_w = ort_data

        move_base_topic = 'move_base'

        client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)

        rospy.loginfo('[RobotPlanner-%s] Waiting for the action server to start', id)
    
        client.wait_for_server()

        rospy.loginfo('[RobotPlanner-%s] Action server started, sending the goal', id)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = float(pos_x)
        goal.target_pose.pose.position.y = float(pos_y)
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = float(ort_z)
        goal.target_pose.pose.orientation.w = float(ort_w)

        client.send_goal(goal)

        rospy.loginfo('[RobotPlanner-%s] Waiting for the result',id)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('[RobotPlanner-%s] The base failed to move for some reason', id)
            return False
        
        rospy.loginfo('[RobotPlanner-%s] Hooray, I reached the goal', id)

        return True