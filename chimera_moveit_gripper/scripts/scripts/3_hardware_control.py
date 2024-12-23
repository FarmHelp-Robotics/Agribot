#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray  # For the /gripper_fun topic
import serial
import time
import math

# Define the serial port where the Arduino is connected
arduino_port = '/dev/ttyUSB0'  # Update if different
baud_rate = 1000000  # Same baud rate as in the Arduino code

# Initialize serial connection to the Arduino
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

# Initialize default values for gripper parameters
grip_mode = 0
grip_value = 1000
sprayer = 0
speed = 500

# Function to convert radians to degrees
def radians_to_degrees(radians):
    return radians * (-180.0 / math.pi)

# Function to send command to Arduino in the required format
def send_servo_command(angles, grip_mode, grip_value, sprayer, speed):
    # Ensure that angles list has 5 elements
    if len(angles) == 5:
        # Append the values received from the /gripper_fun topic
        angles.extend([grip_mode, grip_value, sprayer, speed])
        
        # Construct the message in the required format
        command = "({},{},{},{},{},{},{},{}){}".format(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], angles[8])
        
        # Send the command to the Arduino
        arduino.write(command.encode())  # Send command to Arduino
        
        print("Sent command: {}".format(command))  # Print the command sent to the Arduino

# Callback function to handle the received JointState message
def joint_state_callback(msg):
    # Check if msg.position has at least 7 elements (indices 0 to 6)
    if len(msg.position) >= 7:
        # Extract joint positions for joints 21, 32, 43, 54, and 65 (indexes 2, 3, 4, 5, 6 in 0-based index)
        # Convert from radians to degrees
        angles = [int(radians_to_degrees(msg.position[2])- 86.45), 
                  int(radians_to_degrees(msg.position[3])), 
                  int(radians_to_degrees(-msg.position[4])), 
                  int(radians_to_degrees(msg.position[5])), 
                  int(radians_to_degrees(msg.position[6]))]
        
        # Send the command to the Arduino, using the latest gripper values
        send_servo_command(angles, grip_mode, grip_value, sprayer, speed)
    else:
        rospy.logwarn("Received JointState message has insufficient positions: {}".format(len(msg.position)))
        print("Positions in message:", msg.position)  # Print the received positions for debugging

# Callback function to handle the received gripper_fun message
def gripper_fun_callback(msg):
    global grip_mode, grip_value, sprayer, speed

    # Extract values from the gripper_fun message
    if len(msg.data) >= 4:
        grip_mode = int(msg.data[0])
        grip_value = int(msg.data[1])
        sprayer = int(msg.data[2])
        speed = int(msg.data[3])
        
        #rospy.loginfo("Received gripper_fun values: grip_mode={}, grip_value={}, sprayer={}, speed={}".format(grip_mode, grip_value, sprayer, speed))

# Initialize ROS node
rospy.init_node('arduino_servo_control')

# Subscribe to the joint_states topic
rospy.Subscriber("/joint_states", JointState, joint_state_callback)

# Subscribe to the gripper_fun topic
rospy.Subscriber("/gripper_fun", Float32MultiArray, gripper_fun_callback)

# Keep the node running
rospy.spin()

# Close the serial connection when done
arduino.close()

