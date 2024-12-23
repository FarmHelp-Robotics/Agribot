#!/usr/bin/env python2

"""
This script obtains the joint angles of the manipulator in ROS
and sends them to the real manipulator using the `pymycobot` API.
Parameters:
    port: serial port string. Defaults to '/dev/ttyUSB0'
    baud: serial port baudrate. Defaults to 115200.
"""

import rospy
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
import math

# Joint names and offsets
JOINT_NAMES = ['joint21', 'joint32', 'joint43', 'joint54', 'joint65', 'wrist_joint']
JOINT_OFFSETS = {'joint21': 86.45, 'wrist_joint': 0}

class MyCobotController:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 115200)
        self.mc = MyCobot(port, baud)
        self.last_angles = []
        self.last_gripper_value = None
        
        # Subscribe to joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        
        # Initialize variables
        self.latest_angles = []
        self.gripper_value = None

    def joint_callback(self, data):
        joint_map = {name: idx for idx, name in enumerate(data.name)}

        # Update gripper value if 'right_finger_joint' is present
        if 'right_finger_joint' in joint_map:
            index = joint_map['right_finger_joint']
            self.gripper_value = int(round(math.degrees(data.position[index]), 2))

        # Collect and adjust joint angles
        self.latest_angles = [
            round(math.degrees(data.position[joint_map[name]]), 2) + JOINT_OFFSETS.get(name, 0)
            for name in JOINT_NAMES if name in joint_map
        ]

    def send_commands(self):
        # Send joint angles if they have changed
        if self.latest_angles != self.last_angles:
            rospy.loginfo("Sending angles: %s", self.latest_angles)
            self.mc.send_angles(self.latest_angles, 70)
            self.last_angles = list(self.latest_angles)

        # Send gripper value if it has changed
        if self.gripper_value is not None and self.gripper_value != self.last_gripper_value:
            rospy.loginfo("Sending gripper value: %d", self.gripper_value)
            self.mc.set_gripper_value(self.gripper_value, 70, 4)
            self.last_gripper_value = self.gripper_value

def main():
    rospy.init_node("hardware_control", anonymous=True)
    controller = MyCobotController()

    # Run continuously
    rate = rospy.Rate(20)  # Set the frequency (20 Hz, adjust as needed)
    while not rospy.is_shutdown():
        controller.send_commands()  # Send commands to the manipulator
        rate.sleep()  # Sleep to maintain the desired rate

if __name__ == "__main__":
    main()

