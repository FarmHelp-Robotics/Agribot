#!/usr/bin/env python2

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import message_filters
from Transbot_Lib import Transbot

class CobotMoveit:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('manipulation_arm', anonymous=True)

        # Target pose for the arm
        self.target_pose = geometry_msgs.msg.Pose()
        
        # Gripper control message
        self.msg = Float32MultiArray()
        
        # Transbot servo for right_finger_joint
        self.bot = Transbot()
        self.servo_id = 9  # ID of the servo joint for the gripper

        # MoveIt initialization
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = "arm_group"
        self._eef_planning_group = "gripper_group"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._eef_group = moveit_commander.MoveGroupCommander(self._eef_planning_group)
        
        # Publisher for gripper control
        self.gripper_pub = rospy.Publisher('/gripper_fun', Float32MultiArray, queue_size=10)

        rospy.loginfo("CobotMoveit initialized.")

    def go_to_predefined_arm_pose(self, pose_name):
        """Moves the arm to a predefined pose."""
        rospy.loginfo("Going to predefined pose: {}",pose_name)
        self._group.set_named_target(pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)

    def ask_for_plan(self):
        """Asks the user whether to plan for the target pose."""
        rospy.loginfo('\033[93m' + "Target Pose: \n Position: ({}, {}, {}) \n Orientation: ({}, {}, {}, {})".format(
            self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z,
            self.target_pose.orientation.x, self.target_pose.orientation.y, self.target_pose.orientation.z, self.target_pose.orientation.w) + '\033[0m')
        
        user_input = raw_input("Do you want to plan for the target pose? (Y/N): ")
        if user_input.upper() == 'Y':
            self.plan_and_execute()
        else:
            rospy.loginfo("Unable to plan for the target pose.")

    def plan_and_execute(self):
        """Plans and executes the motion to the target pose."""
        self._group.set_pose_target(self.target_pose)
        plan = self._group.plan()

        # Check if the plan was successful and valid
        if plan and plan.joint_trajectory.points:  # Ensure that the plan has valid points
            rospy.loginfo('\033[92m' + "Plan generated successfully. Executing..." + '\033[0m')
            success = self._group.go(wait=True)

            # Check if execution was successful
            if success:
                rospy.loginfo('\033[92m' + "Successfully moved to the target pose." + '\033[0m')
                rospy.sleep(3)
                # After moving, perform gripper control
                self.bot.set_uart_servo_angle(self.servo_id, 135)
                rospy.sleep(2)
                self._control_gripper(grip_mode=1, grip_value=2000)
                rospy.sleep(1)
                self._control_gripper(grip_mode=0, grip_value=0)
                rospy.loginfo("Gripper values published.")
            else:
                rospy.loginfo('\033[91m' + "Execution failed. No valid motion plan was executed." + '\033[0m')  # Red text for failure
        else:
            rospy.loginfo('\033[91m' + "Plan generation failed. No valid plan was found." + '\033[0m')  # Red text for failure

    def callback(self, r_data):
        """Callback to update the target pose and trigger planning."""
        # Update target pose from incoming data
        self.target_pose.position.x = r_data.position.x
        self.target_pose.position.y = r_data.position.y
        self.target_pose.position.z = r_data.position.z
        self.target_pose.orientation.x = -0.712406800344
        self.target_pose.orientation.y = -3.50805107418e-06
        self.target_pose.orientation.z = -5.30664007665e-05
        self.target_pose.orientation.w = 0.701766733321

        # Set initial servo angle and publish gripper control
        self.bot.set_uart_servo_angle(self.servo_id, 70)
        self._control_gripper(grip_mode=0, grip_value=0)
        
        # Ask the user whether to plan for the target pose
        self.ask_for_plan()

    def _control_gripper(self, grip_mode, grip_value):
        """Helper method to control the gripper."""
        self.msg.data = [grip_mode, grip_value, 0.0, 500.0]
        self.gripper_pub.publish(self.msg)

    def __del__(self):
        """Shutdown MoveIt when the object is deleted."""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("CobotMoveit object deleted.")

def main():
    cobot = CobotMoveit()
    
    # Subscribe to the tomato pose topic and process it using the callback
    rospy.Subscriber("/tomato_pose", Pose, cobot.callback, queue_size=1)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

