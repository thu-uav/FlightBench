import rospy
import time
import math
import os, sys
import numpy as np
from enum import Enum
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import AutopilotFeedback, LowLevelFeedback, Trajectory, TrajectoryPoint
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

scale = 1

if __name__ == "__main__":
    if len(sys.argv) == 2:
        rospy.init_node('path_publisher', anonymous=False)
        traj_pub = rospy.Publisher('/'+quad_name+'/autopilot/trajectory', Trajectory, queue_size = 2)
        autopilot_state_sub = rospy.Subscriber("/"+quad_name+"/autopilot/feedback", AutopilotFeedback, autopilot_feedback_callback,queue_size = 2)
        # read traj file
        traj = np.loadtxt(sys.argv[1], delimiter=',',dtype=str)
        #traj.reshape((-1,24))
        traj = traj[1:, :]
        traj = traj.astype(np.float32)
        print("traj_len; ", traj.shape)

        # set traj msg
        traj_list = []
        traj_msg = Trajectory()
        traj_msg.type = traj_msg.GENERAL
        for i in range(traj.shape[0]):
            traj_point = TrajectoryPoint()
            traj_point.time_from_start = rospy.rostime.Duration(traj[i,0]) * scale
            traj_point.pose.position.x = traj[i,1]
            traj_point.pose.position.y = traj[i,2]
            traj_point.pose.position.z = traj[i,3]
            traj_point.pose.orientation.w = traj[i,4]
            traj_point.pose.orientation.x = traj[i,5]
            traj_point.pose.orientation.y = traj[i,6]
            traj_point.pose.orientation.z = traj[i,7]
            traj_point.velocity.linear.x = traj[i, 8] / scale
            traj_point.velocity.linear.y = traj[i, 9] / scale
            traj_point.velocity.linear.z = traj[i, 10] / scale
            traj_point.velocity.angular.x = traj[i, 11] / scale
            traj_point.velocity.angular.y = traj[i, 12] / scale
            traj_point.velocity.angular.z = traj[i, 13] / scale
            traj_point.acceleration.linear.x = traj[i, 14] / scale**2
            traj_point.acceleration.linear.y = traj[i, 15] / scale**2
            traj_point.acceleration.linear.z = traj[i, 16] / scale**2
            traj_list.append(traj_point)
        traj_msg.points = traj_list
        print("pub traj len", len(traj_list))
        traj_msg.header.stamp = rospy.Time.now()
        print("before pub")
        while True:
            traj_pub.publish(traj_msg)
            rospy.sleep(0.2)
    else:
        print("arg num invalid")