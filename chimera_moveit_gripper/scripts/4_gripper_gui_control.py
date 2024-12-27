#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32MultiArray
import Tkinter as tk  # For Python 2, use Tkinter (with uppercase 'T')
from Transbot_Lib import Transbot

class GripperControlGUI:
    def __init__(self, root):
        # Initialize the ROS node
        rospy.init_node('gripper_control_gui', anonymous=True)
        self.pub = rospy.Publisher('/gripper_fun', Float32MultiArray, queue_size=10)

        # Initialize main window
        self.root = root
        self.root.title("EE Control GUI")
        
        # Transbot servo for right_finger_joint
        self.bot = Transbot()
        self.servo_id = 7  # ID of the current joint id 9 
        
        # Set the window size
        self.root.geometry("200x350")

        # Grip Mode (0 or 1)
        self.grip_mode = tk.IntVar(value=0)  # 0: Mode 1, 1: Mode 2
        self.grip_mode_label = tk.Label(self.root, text="End-Effector Mode P/V(0/1): 0")
        self.grip_mode_label.pack()

        self.grip_mode_slider = tk.Scale(self.root, from_=0, to=1, orient="horizontal", variable=self.grip_mode)
        self.grip_mode_slider.pack()
        self.grip_mode_slider.bind("<Motion>", self.update_grip_mode)

        # Grip Value (1000 to 3000 if grip_mode=0, 0 to 30 if grip_mode=1)
        self.grip_value = tk.IntVar(value=1000)  # Initial grip value
        self.grip_value_label = tk.Label(self.root, text="End-Effector Value: 1000")
        self.grip_value_label.pack()

        self.grip_value_slider = tk.Scale(self.root, from_=1000, to=3000, orient="horizontal", variable=self.grip_value)
        self.grip_value_slider.pack()
        self.grip_value_slider.bind("<Motion>", self.update_grip_value)
        
        # Grip Open Close 
        self.gripper = tk.IntVar(value=130)  # Initial grip value
        self.gripper_label = tk.Label(self.root, text="Gripper: 50")
        self.gripper_label.pack()

        self.gripper_slider = tk.Scale(self.root, from_=60, to=180, orient="horizontal", variable=self.gripper)
        self.gripper_slider.pack()
        self.gripper_slider.bind("<Motion>", self.update_gripper)

        # Sprayer (0 or 1)
        self.sprayer = tk.IntVar(value=0)  # 0: Off, 1: On
        self.sprayer_label = tk.Label(self.root, text="Sprayer: 0")
        self.sprayer_label.pack()

        self.sprayer_slider = tk.Scale(self.root, from_=0, to=1, orient="horizontal", variable=self.sprayer)
        self.sprayer_slider.pack()
        self.sprayer_slider.bind("<Motion>", self.update_sprayer)

        # Speed (100 to 1000)
        self.speed = tk.IntVar(value=500)  # Initial speed value
        self.speed_label = tk.Label(self.root, text="Speed: 100")
        self.speed_label.pack()

        self.speed_slider = tk.Scale(self.root, from_=4, to=1000, orient="horizontal", variable=self.speed)
        self.speed_slider.pack()
        self.speed_slider.bind("<Motion>", self.update_speed)

        # Update slider ranges based on grip_mode
        self.update_grip_value_range()

    def update_grip_mode(self, event):
        grip_mode_value = self.grip_mode.get()
        self.grip_mode_label.config(text="End-Effector Mode P/V(0/1): {}".format(grip_mode_value))
        self.update_grip_value_range()
        self.publish_values()

    def update_grip_value_range(self):
        grip_mode_value = self.grip_mode.get()
        if grip_mode_value == 1:
            # Adjust the range of grip_value slider to 0-30
            self.grip_value_slider.config(from_=0, to=2000)
        else:
            # Adjust the range of grip_value slider to 1000-3000
            self.grip_value_slider.config(from_=-180, to=180)

    def update_grip_value(self, event):
        grip_value = self.grip_value.get()
        self.grip_value_label.config(text="End-Effector Value: {}".format(grip_value))
        self.publish_values()
        
    def update_gripper(self, event):
        gripper = self.gripper.get()
        self.gripper_label.config(text="Gripper: {}".format(gripper))
        self.bot.set_uart_servo_angle(self.servo_id, gripper)
        

    def update_sprayer(self, event):
        sprayer_value = self.sprayer.get()
        self.sprayer_label.config(text="Sprayer: {}".format(sprayer_value))
        self.publish_values()

    def update_speed(self, event):
        speed_value = self.speed.get()
        self.speed_label.config(text="Speed: {}".format(speed_value))
        self.publish_values()

    def publish_values(self):
        # Collect values to publish
        grip_mode_value = self.grip_mode.get()
        grip_value = self.grip_value.get()
        sprayer_value = self.sprayer.get()
        speed_value = self.speed.get()

        # Prepare the message to publish
        msg = Float32MultiArray()
        msg.data = [float(grip_mode_value), float(grip_value), float(sprayer_value), float(speed_value)]

        # Publish the message
        self.pub.publish(msg)

# Create the main window
root = tk.Tk()

# Create the GUI application
app = GripperControlGUI(root)

# Run the Tkinter event loop
root.mainloop()
