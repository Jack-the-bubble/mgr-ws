# general imports
import sys

# ros imports
import actionlib
from moveit_commander import move_group
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult
from sensor_msgs.msg import JointState

class TiagoNode:
    def __init__(self):
        self.group_name = 'arm_torso'
        self.play_ns = 'play_motion'

        rospy.init_node('tiago_interface', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.robot_state = moveit_msgs.msg.RobotState()
        self.task_rate = rospy.Rate(1)
        self.actionlib_client = actionlib.SimpleActionClient('move_base',
                                                             MoveBaseAction)
        self.actionlib_client.wait_for_server()
        self.head_client = actionlib.SimpleActionClient(self.play_ns, PlayMotionAction)
        self.head_client.wait_for_server()

        # publishers/subscribers
        self.trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    @staticmethod
    def assign_quaternion_from_euler(euler_angles):
        quat = quaternion_from_euler(*euler_angles)
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose.orientation

    def move_base(self, pos_x, pos_y, rot):
        quat = TiagoInterface.assign_quaternion_from_euler([0, 0, rot])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pos_x
        goal.target_pose.pose.position.y = pos_y
        goal.target_pose.pose.orientation = quat

        self.actionlib_client.send_goal(goal)
        wait = self.actionlib_client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available")

        else:
            print("Result:")
            print(self.actionlib_client.get_result())
            print("End of result")

    def move_hand_ik(self, pose):
        self.group.set_pose_target(pose)
        _ = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def move_joints(self, name):
        goal = PlayMotionGoal()
        goal.motion_name = name
        goal.priority = 0
        goal.skip_planning = False
        self.head_client.send_goal(goal)
        result = self.head_client.wait_for_result()
        print(result)

    def cleanup(self):
        self.group.stop()
        self.group.clear_pose_targets()
