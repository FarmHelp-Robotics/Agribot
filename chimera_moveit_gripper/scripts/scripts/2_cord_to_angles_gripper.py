#!/usr/bin/env python2

import rospy
import sys
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import message_filters
import warnings
from Transbot_Lib import Transbot

class CobotMoveit:
    # Constructor
    def __init__(self):
        rospy.init_node('manipulation_arm', anonymous=True)
        
        self.target_pose = geometry_msgs.msg.Pose()
        self.prev_target_pose = None  # Store the previous target pose for comparison

        # Initialize Transbot for the gripper
        self.bot = Transbot()
        self.servo_id = 9  # ID of the gripper joint

        # MoveIt initialization
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = "arm_group"
        self._eef_planning_group = "gripper_group"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._eef_group = moveit_commander.MoveGroupCommander(self._eef_planning_group)
        
        # Publisher for visualizing the trajectory
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
        self._execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()
        
        rospy.loginfo('\033[94m' + " >>> CobotMoveit init done." + '\033[0m')
        
    def go_to_predefined_ee_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._eef_group.set_named_target(arg_pose_name)
        plan = self._eef_group.plan()
        self._eef_group.go(wait=True)

    # for going to predefined pose for arm group
    def go_to_predefined_arm_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)
            
    def go_to_pose_incrementally(self, target_pose, increment=0.05):
        current_pose = self._group.get_current_pose().pose
        n = 0

        # Loop until reaching target pose or reaching increment count
        while n <= 2:
            dx = target_pose.position.x - current_pose.position.x
            dy = target_pose.position.y - current_pose.position.y
            dz = target_pose.position.z - current_pose.position.z
            
            # Check if within increment range
            if abs(dx) < increment and abs(dy) < increment and abs(dz) < increment:
                break  # Exit if close enough to target
            
            # Update current position incrementally
            current_pose.position.x += increment if dx > 0 else -increment
            current_pose.position.y += increment if dy > 0 else -increment
            current_pose.position.z += increment if dz > 0 else -increment
            
            # Send updated current pose to MoveIt
            self._group.set_pose_target(current_pose)
            self._group.go(wait=True)
            n += 1
           
            
        rospy.loginfo('\033[94m' + ">>> Can't reach the target " + '\033[0m')

    def callback(self, r_data):
        # Check if pose change is significant
        if self.prev_target_pose:
            delta_x = abs(r_data.position.x - self.prev_target_pose.position.x)
            delta_y = abs(r_data.position.y - self.prev_target_pose.position.y)
            delta_z = abs(r_data.position.z - self.prev_target_pose.position.z)

            percent_change_x = (delta_x / abs(self.prev_target_pose.position.x)) * 100 if self.prev_target_pose.position.x != 0 else 0
            percent_change_y = (delta_y / abs(self.prev_target_pose.position.y)) * 100 if self.prev_target_pose.position.y != 0 else 0
            percent_change_z = (delta_z / abs(self.prev_target_pose.position.z)) * 100 if self.prev_target_pose.position.z != 0 else 0

            # Check if any of the changes is less than 3%
            if percent_change_x < 3 and percent_change_y < 3 and percent_change_z < 3:
                rospy.loginfo("Change is less than 3% in all axes, no replanning.")
                return

        # Update target pose from new incoming data
        self.target_pose.position.x = r_data.position.x
        self.target_pose.position.y = r_data.position.y
        self.target_pose.position.z = r_data.position.z
        self.target_pose.orientation.x = -0.712406800344
        self.target_pose.orientation.y = -3.50805107418e-06
        self.target_pose.orientation.z = -5.30664007665e-05
        self.target_pose.orientation.w = 0.701766733321

        # Update previous target pose
        self.prev_target_pose = self.target_pose

        # Move to updated target pose
        self.move_arm()

    def move_arm(self):
        # Example sequence of actions
        
        #self.go_to_predefined_arm_pose("pose1")
        #self.go_to_predefined_ee_pose("close")
        self.go_to_pose_incrementally(self.target_pose)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class CobotMoveit Deleted." + '\033[0m')

def main():
    cobot = CobotMoveit()
    
    # Subscribe to tomato pose topic and use callback for processing
    tomato_pose = rospy.Subscriber("/tomato_pose", Pose, cobot.callback)
    rospy.spin()

if __name__ == '__main__':
    main()

