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
red_pose = []
re = None
xyz = []
quat_list = []


class CobotMoveit:
    # Constructor
    def __init__(self):
        rospy.init_node('manipulation_arm', anonymous=True)

        # init moveit commander object 
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        # init robot commander object 
        self._robot = moveit_commander.RobotCommander()
        # init planning scene interface object 
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = "arm_group"
        self._eef_planning_group = "gripper_group"
        # move group commander object 
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._eef_group = moveit_commander.MoveGroupCommander(self._eef_planning_group)
        # publish trajectory for RViz visualization 
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()  # for arm 
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> CobotMoveit init done." + '\033[0m')

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + "\nEnd-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "joint1: {}\n".format(math.degrees(list_joint_values[0])) +
                      "joint2: {}\n".format(math.degrees(list_joint_values[1])) +
                      "joint3: {}\n".format(math.degrees(list_joint_values[2])) +
                      "joint4: {}\n".format(math.degrees(list_joint_values[3])) +
                      "joint5: {}\n".format(math.degrees(list_joint_values[4])) +
                      "wrist_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')
                      
                      
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
                  

    def go_to_pose_incrementally(self, target_pose, increment=0.08):
        current_pose = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(current_pose)

        # Loop until we reach the target pose
        while not rospy.is_shutdown():
            dx = target_pose.position.x - current_pose.position.x
            dy = target_pose.position.y - current_pose.position.y
            dz = target_pose.position.z - current_pose.position.z
            
            # Check if within increment range
            if abs(dx) < increment and abs(dy) < increment and abs(dz) < increment:
                current_pose.position.x = target_pose.position.x
                current_pose.position.y = target_pose.position.y
                current_pose.position.z = target_pose.position.z
                break

            # Update current position incrementally
            if abs(dx) >= increment:
                current_pose.position.x += increment if dx > 0 else -increment
            if abs(dy) >= increment:
                current_pose.position.y += increment if dy > 0 else -increment
            if abs(dz) >= increment:
                current_pose.position.z += increment if dz > 0 else -increment
            
            # Send the current pose to MoveIt
            self._group.set_pose_target(current_pose)
            self._group.go(wait=True)
            
        rospy.loginfo('\033[94m' + ">>> Reached Target Pose:" + '\033[0m')
        rospy.loginfo(current_pose)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class CobotMoveit Deleted." + '\033[0m')

    def callback(self, r_data):
        global red_pose, re, quat_list, xyz
        
        # Transbot servo for right_finger_joint
        self.bot = Transbot()
        self.servo_id = 9  # ID of the current joint
        
        # Pose from the camera
        re = r_data
        r_x = re.position.x
        r_y = re.position.y
        r_z = re.position.z

        # Create target pose for the robot
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = r_x 
        target_pose.position.y = r_y 
        target_pose.position.z = r_z 
        target_pose.orientation.x = -0.712406800344
        target_pose.orientation.y = -3.50805107418e-06
        target_pose.orientation.z = -5.30664007665e-05
        target_pose.orientation.w = 0.701766733321
        
        self.go_to_predefined_arm_pose("pose1")
        self.bot.set_uart_servo_angle(self.servo_id, 70)
        rospy.sleep(1)
        
        # Move to the target pose incrementally
        self.go_to_pose_incrementally(target_pose)
        rospy.sleep(1)  # Adjust the sleep as needed
        self.bot.set_uart_servo_angle(self.servo_id, 144)
        rospy.sleep(3) 

def main():
    cobot = CobotMoveit()
    
    tomato_pose = rospy.Subscriber("/tomato_pose", Pose,cobot.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
