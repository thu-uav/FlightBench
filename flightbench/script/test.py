import rospy
import time
import math
import os, sys
import numpy as np
from enum import Enum
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import AutopilotFeedback, LowLevelFeedback, Trajectory, TrajectoryPoint, ControlCommand
from mavros_msgs.msg import State

quad_name = 'air'
autopilot_state = None

class enum_autopilot_states(Enum):
    OFF=0
    START=1
    HOVER=2
    LAND=3
    EMERGENCY_LAND=4
    BREAKING=5
    GO_TO_POSE=6
    VELOCITY_CONTROL=7
    REFERENCE_CONTROL=8
    TRAJECTORY_CONTROL=9
    COMMAND_FEEDTHROUGH=10
    RC_MANUAL=11

def autopilot_feedback_callback(data):
    autopilot_state = enum_autopilot_states(data.autopilot_state) # data.autopilot_state -> uint8

scale = 5

if __name__ == "__main__":
    rospy.init_node('path_publisher', anonymous=False)
    cmd_pub = rospy.Publisher('/'+quad_name+'/autopilot/control_command_input', ControlCommand, queue_size = 2)
    autopilot_state_sub = rospy.Subscriber("/"+quad_name+"/autopilot/feedback", AutopilotFeedback, autopilot_feedback_callback,queue_size = 2)
    # set cmd msg

    cmd_msg = ControlCommand()
    cmd_msg.control_mode = cmd_msg.BODY_RATES
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.armed = True
    cmd_msg.expected_execution_time = rospy.Time(0.05)
    cmd_msg.collective_thrust = 9
    cmd_msg.bodyrates.x = 0
    cmd_msg.bodyrates.y = scale
    cmd_msg.bodyrates.z = 0
    print("before pub")
    while True:
        cmd_pub.publish(cmd_msg)
        rospy.sleep(0.01)