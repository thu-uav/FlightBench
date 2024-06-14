import os
import sys
import rospy
import yaml
import rosbag
import numpy as np
from collections import deque
import math
import copy
import matplotlib.pyplot as plt

quad_name = "air"
sim_path = "/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/performance/position/13state.txt"
real_path = "/home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/performance/bags/ciricle_2024-05-22-14-56-23.bag"

bag = rosbag.Bag(real_path)

radius = 2.0
odom = bag.read_messages(topics=['/'+quad_name+'/t265/odom/sample'])
odom_list = []

autopilot_fb = bag.read_messages(topics=['/'+quad_name+'/autopilot/feedback'])
state_deque = deque(maxlen = 16)
start_time = 0.0
for topic, msg, t in autopilot_fb:
    state_deque.append(msg.autopilot_state)
    if(len(state_deque)>=16):
        if state_deque[15] == 9 and state_deque[14] == 2 and state_deque[0] == 9:
            start_time = t.to_sec()
            break

print(start_time)
real_o = np.zeros(3)
min_interval = np.inf
for topic, msg, t in odom:
    odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    if abs(t.to_sec() - start_time) < min_interval:
        real_o = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        min_interval = abs(t.to_sec() - start_time)
        odom_list.clear()
        odom_list.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
real_array = np.stack(odom_list) - real_o

sim_array = np.loadtxt(sim_path)
sim_o = copy.deepcopy(sim_array[0])
sim_array = sim_array-sim_o
sim_array = sim_array[:, [1,0,2]]
sim_array[:, 1]*=-1
# plt.plot(sim_array[:, 0])
# plt.show()

error_list = []
ptr_real = 0
for i in range(sim_array.shape[0]):
    dist =  np.linalg.norm(real_array[ptr_real:, :] - sim_array[i], axis = 1)
    idx = np.argmin(dist)
    # print("sim:", sim_array[i], "real:", real_array[idx], "dist:", dist[idx])
    ptr_real = copy.deepcopy(idx)
    error_list.append(dist[idx])

avg_err = sum(error_list) / len(error_list)
print("avg. position error:", avg_err)
print("relative error: %.4f" % (avg_err / radius))