import time
import os
import rospy
import numpy as np
import torch
import copy
import cv2

import matplotlib.pyplot as plt
import transforms3d
from cv_bridge import CvBridge, CvBridgeError
from quadrotor_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

from onpolicy.runner.student_trainer import Trainer

class PARosRunner():
    def __init__(self, model_dir):
        self.trainer = Trainer(data_dir="empty", eval_dir="empty", pretrain_dir=model_dir)
        rospy.init_node('learning_min_time', anonymous=True)
        self.odom_sub = rospy.Subscriber("/air/ground_truth/odometry", Odometry, self.odom_callback)
        self.img_sub = rospy.Subscriber("/air/flight_pilot/depth", Image, self.img_callback)
        self.command_pub = rospy.Publisher('/air/autopilot/control_command_input', ControlCommand, queue_size = 10)
        self.ros_odom = None
        self.ros_img = None
        self.count = 0
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/45.0), self.eval_ros_step)
        self.delay = 1.0
        self.bridge = CvBridge()
        self.pre_pub_ = rospy.Publisher('/mapping_time', Float32, queue_size = 10)
        self.planning_pub_ = rospy.Publisher('/planning_time', Float32, queue_size = 10)

    def odom_callback(self, data):
        tmp_odom = np.zeros(12, dtype=np.float32)
        quat = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z])
        R = np.array(
        [[1.0 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[0] * quat[2] + quat[1] * quat[3])],
         [2 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]), 2 * (quat[2] * quat[3] - quat[0] * quat[1])],
         [2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), 1.0 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])]])
        tmp_odom[:9] = R.reshape(9)
        mat = R
        velo_body = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        tmp_odom[9:12] = mat @ velo_body
        tmp_odom = tmp_odom.reshape(1, 12)
        self.ros_odom = copy.deepcopy(tmp_odom)
    
    def img_callback(self, data):
        start = time.time_ns()
        depth = self.bridge.imgmsg_to_cv2(data, '32FC1')
        far = depth > 4.1
        depth = np.zeros_like(depth, dtype=np.float32) * far + (~far) * depth
        dim = (112, 112)
        depth = cv2.resize(depth, dim)
        depth = depth.reshape(1, 1, 112, 112)
        self.ros_img = torch.tensor(depth).cuda()
        end = time.time_ns()
        time_msg = Float32()
        time_msg.data = (end-start) / 1e6
        self.pre_pub_.publish(time_msg)

    def eval_ros_step(self, event=None):
        if type(self.ros_odom) == type(None) or type(self.ros_img) == type(None):
            return
        self.count += 1
        if self.count < self.delay * 40.0:
            return
        start = time.time_ns()
        actions = self.trainer.net(depth = self.ros_img, imu = torch.tensor(self.ros_odom).cuda())
        actions = actions.detach().cpu().numpy()
        end = time.time_ns()
        time_msg = Float32()
        time_msg.data = (end-start) / 1e6
        self.planning_pub_.publish(time_msg)
        cmd_msg = ControlCommand()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.control_mode = cmd_msg.BODY_RATES
        cmd_msg.armed = True
        cmd_msg.expected_execution_time = rospy.Time(0.020)
        cmd_msg.collective_thrust = (np.tanh(actions[0, 0])+1) * 20.0 / 1.8
        cmd_msg.bodyrates.x = np.tanh(actions[0, 1]) * 8.0
        cmd_msg.bodyrates.y = np.tanh(actions[0, 2]) * 8.0
        cmd_msg.bodyrates.z = np.tanh(actions[0, 3]) * 3.0

        self.command_pub.publish(cmd_msg)
