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

red_pose = []
re = None
xyz = []
quat_list = []


class CobotMoveit:
    # Constructor
    def __init__(self):
        rospy.init_node('manipulation_arm', anonymous=True)

        # init moveit commandr object 
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        #init robot commander object 
        self._robot = moveit_commander.RobotCommander()
        # init planning scene interface object 
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = "arm_group"
        self._eef_planning_group = "gripper_group"
        # move group commander object 
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._eef_group = moveit_commander.MoveGroupCommander(self._eef_planning_group)
        # publish trajectory fro rviz visualition 
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame() # for arm 
        self._planning_frame = self._eef_group.get_planning_frame() # for gripper
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        
         # Publisher for joint angles
        self.joint_pub = rospy.Publisher('/ik_angles', Float64MultiArray, queue_size=10)
        self.joint_angles_msg = Float64MultiArray()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> CobotMoveit init done." + '\033[0m')

    def print_pose_ee(self):
    
        global quat_list, pose_values, xyz
        
        pose_values = self._group.get_current_pose().pose
        
        x = pose_values.position.x
        y = pose_values.position.y
        z = pose_values.position.z
        
        xyz = [x, y, z]

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w
        
        quaternion_list = [q_x, q_y, q_z, q_w]
        
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        
        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "q_x: {}\n".format(q_x) +
                      "q_y: {}\n".format(q_y) +
                      "q_z: {}\n".format(q_z) +
                      "q_w: {}\n".format(q_w) +
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

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        
        self.joint_angles_msg.data = [math.degrees(angle) for angle in list_joint_values]
        self.joint_pub.publish(self.joint_angles_msg)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_predefined_ee_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._eef_group.set_named_target(arg_pose_name)
        plan = self._eef_group.plan()
        self._eef_group.go(wait=True)

    def go_to_predefined_arm_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        self._group.go(wait=True)

    def go_to_pose(self, arg_pose):
    
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        
        self.joint_angles_msg.data = [math.degrees(angle) for angle in list_joint_values]
        self.joint_pub.publish(self.joint_angles_msg)
        print("hfdslfgfd",self.joint_angles_msg)

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class CobotMoveit Deleted." + '\033[0m')

    def callback(self, r_data):
        global red_pose, re, quat_list, xyz
        
        #---------------------- tomato pose wrt to transbot base ----------------------#
        re = r_data
        r_x = re.position.x
        r_y = re.position.y
        r_z = re.position.z
        r_roll = re.orientation.x
        r_pitch = re.orientation.y
        r_yaw = re.orientation.z
        r_w = re.orientation.w
        r_pose = [r_x, r_y, r_z, r_roll, r_pitch, r_yaw, r_w]
        print("tomato_pose", r_pose)

        
        #______________giving pose to move arm towardstomato___________________#
        
        r_1 = geometry_msgs.msg.Pose()
        r_1.position.x = r_x 
        r_1.position.y = r_y 
        r_1.position.z = r_z -0.002
        r_1.orientation.x = -0.712406800344
        r_1.orientation.y = -3.50805107418e-06
        r_1.orientation.z = -5.30664007665e-05
        r_1.orientation.w = 0.701766733321
        
        r_3 = [math.radians(-84),
           math.radians(33),
           math.radians(-142),
           math.radians(29),
           math.radians(0),
           math.radians(0)]

        #__________________________execution____________________________#
        
        while not rospy.is_shutdown():
            if not sys.warnoptions:
                warnings.simplefilter("ignore")

            #self.print_pose_ee()
            #self.print_joint_angles()
            #rospy.sleep(1)
            
            # Plan the motion to the target pose
            self.go_to_predefined_arm_pose("home")
            self.go_to_predefined_ee_pose("open")
            rospy.sleep(1)
            self.go_to_pose(r_1)
            rospy.sleep(1)  # Adjust the sleep to 0.5 seconds
            #self.go_to_predefined_ee_pose("close")
            #self.set_joint_angles(r_3)
            #self.go_to_predefined_ee_pose("open")

            

def main():
    global x, y, z, roll, pitch, yaw, ww

    cobot = CobotMoveit()
    
    tomato_pose = message_filters.Subscriber("/tomato_pose", Pose)
    clbk_pose = message_filters.ApproximateTimeSynchronizer([tomato_pose], queue_size=10, slop=0.5, allow_headerless=True)
    clbk_pose.registerCallback(cobot.callback)

    rospy.spin()

if __name__ == '__main__':
    main()

