import os
import sys
import rospy
import yaml
import rosbag
import numpy as np
from collections import deque
import math
import copy

quad_name = "air"

def cal_sphere(x1, y1, z1, x2, y2, z2, x3, y3, z3):
    a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2)
    b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2)
    c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2)
    d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1)

    a2 = 2 * (x2 - x1)
    b2 = 2 * (y2 - y1)
    c2 = 2 * (z2 - z1)
    d2 = x1*x1 + y1*y1 + z1*z1 - x2*x2 - y2*y2 - z2*z2

    a3 = 2 * (x3 - x1)
    b3 = 2 * (y3 - y1)
    c3 = 2 * (z3 - z1)
    d3 = x1*x1 + y1*y1 + z1*z1 - x3*x3 - y3*y3 - z3*z3

    x = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1) / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1)
    y = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1) / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1)
    z = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1) / (a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1)
    radius = math.sqrt((x1 - x)*(x1 - x) + (y1 - y)*(y1 - y) + (z1 - z)*(z1 - z))

    return radius

def quaternion_to_rotation_matrix(q):  # x, y ,z ,w
    rot_matrix = np.array(
        [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
         [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
         [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]],
        dtype=q.dtype)
    return rot_matrix

def parse_one_bag(path):
    bag = rosbag.Bag(path)

    reach_radius = 1.5
    
    start_t = None
    end_t = None
    success = False
    flying_time = None
    mapping_time = None
    planning_time = None
    projection_time = None
    energy_acc = None
    energy_jerk = None
    traj_length = None
    max_k_ = None
    avg_k_ = None
    max_speed_ = None
    reach_min_dist_ = None
    path_ = path

    def output():
        if type(traj_length) == type(None) or type(flying_time) == type(None):
            avg_v = None
        else:
            avg_v = traj_length / flying_time
        print("=== %s completed ===" % path_)
        print("flying time(s): ",flying_time, "max_v: ", max_speed_, "average_v: ", avg_v)
        print("traj length(m): ", traj_length, ", max_k: ", max_k_, ", avg_k: ", avg_k_)
        print("success: ", success, ", min reached dist: ", reach_min_dist_)
        print("mapping time(ms): ", mapping_time, ", planning time(ms): ", planning_time, ", projection time(ms): ", projection_time)
        print("energy(acc): ", energy_acc, ", energy(jerk): ", energy_jerk)
        print("=========================")
        return max_speed_, avg_v, traj_length, max_k_, avg_k_, success, reach_min_dist_, energy_acc, energy_jerk, mapping_time, planning_time, projection_time

    # get start t and end t
    start = bag.read_messages(topics=['/'+quad_name+'/flight_pilot/if_start'])
    start_deque = deque(maxlen=3)
    for topic, msg, t in start:
        start_deque.append(msg.data)
        if len(start_deque)>2:
            if start_deque[2]>0.3 and start_deque[1]<=0.3:
                start_t = t
                break
    end = bag.read_messages(topics=['/'+quad_name+'/flight_pilot/if_end'])
    end_deque = deque(maxlen=3)
    for topic, msg, t in end:
        end_deque.append(msg.data)
        if len(end_deque)>2:
            if end_deque[2]<reach_radius and end_deque[1]>=reach_radius:
                end_t = t
                break
    
    if(start_t==None or end_t == None or (end_t-start_t).to_sec() < 0):
        print("Cannot found start or end point.")
        success = False
        return output()
    else:
        flying_time = (end_t-start_t).to_sec()

    # check collision
    collision_time = copy.deepcopy(end_t)
    collision = bag.read_messages(topics=['/'+quad_name+'/flight_pilot/if_collision'])
    for topic, msg, t in collision:
        if t>start_t and t<end_t:
            if msg.data == True:
                print("collision at: ", (t-start_t).to_sec())
                success = False
                return output()
        if t>start_t and msg.data == True:
            collision_time = copy.deepcopy(t)
            # print("ct: ", (collision_time - end_t).to_sec())
            break
        else:
            collision_time = copy.deepcopy(t)
    success = True

    end = bag.read_messages(topics=['/'+quad_name+'/flight_pilot/if_end'])
    # print("start: ", start_t.to_sec(), "ct: ", collision_time.to_sec())
    for topic, msg, t in end:
        if t > start_t and t < collision_time:
            if reach_min_dist_ == None or msg.data < reach_min_dist_:
                reach_min_dist_ = msg.data

    # autopilot_fb
    traj_length = 0.0
    max_speed_ = 0.0
    max_k_ = 0.0
    avg_k_ = 0.0
    point_deque = deque(maxlen = 3)
    autopilot_fb = bag.read_messages(topics=['/'+quad_name+'/autopilot/feedback'])
    for topic, msg, t in autopilot_fb:
        point_deque.append(np.array([msg.state_estimate.pose.pose.position.x, msg.state_estimate.pose.pose.position.y, msg.state_estimate.pose.pose.position.z]))
        if t>start_t and t<end_t:
            speed = np.linalg.norm(np.array([msg.state_estimate.twist.twist.linear.x, msg.state_estimate.twist.twist.linear.y, msg.state_estimate.twist.twist.linear.z]))
            if speed>max_speed_:
                max_speed_ = copy.deepcopy(speed)
            traj_length += np.linalg.norm(point_deque[2]-point_deque[1])

            # cal curvature
            r = cal_sphere(point_deque[0][0], point_deque[0][1], point_deque[0][2], point_deque[1][0], point_deque[1][1], point_deque[1][2], point_deque[2][0], point_deque[2][1], point_deque[2][2])
            k = 1 / r
            if k > max_k_:
                max_k_ = copy.deepcopy(k)
            avg_k_ += k * np.linalg.norm(point_deque[2]-point_deque[1])
    avg_k_ /= traj_length
            



    energy_acc = 0.0
    energy_jerk = 0.0
    time_deque = deque(maxlen = 5)
    acc_deque = deque(maxlen = 5)
    imu = bag.read_messages(topics=['/'+quad_name+'/ground_truth/imu'])
    jerk_list = []
    for topic, msg, t in imu:
        time_deque.append(msg.header.stamp.to_sec())
        rot = quaternion_to_rotation_matrix(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
        acc = rot @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        acc_deque.append(acc)
        if t>start_t and t<end_t:
            acc_norm = np.linalg.norm(acc-np.array([0,0,9.8]))
            #print("t: ", t.to_sec(), "acc: ", acc-np.array([0,0,9.8]))
            energy_acc += (time_deque[4]-time_deque[3]) * acc_norm ** 2
            # print("dt: ", (time_deque[2]-time_deque[1]).to_sec())
        # cal t-2 jerk with 5 point
        if len(time_deque) == 5:
            dt = (time_deque[4] - time_deque[0]) / 5
            if time_deque[2]>start_t.to_sec() and time_deque[2]<end_t.to_sec():
                jerk = (-acc_deque[4] + 8 * acc_deque[3] - 8 * acc_deque[1] + acc_deque[0]) / (12 * dt)
                jerk_norm = np.linalg.norm(jerk)
                energy_jerk += dt * jerk_norm ** 2
    
    energy_acc = energy_acc / traj_length
    energy_jerk = energy_jerk / traj_length
    
    pt = bag.read_messages(topics=['/planning_time'])
    time_list = []
    for topic, msg, t in pt:
        if t>start_t and t< end_t:
            time_list.append(msg.data)
    if len(time_list)>1:
        planning_time = np.mean(np.array(time_list))

    mt = bag.read_messages(topics=['/mapping_time'])
    time_list = []
    for topic, msg, t in mt:
        if t>start_t and t< end_t:
            time_list.append(msg.data)
    if len(time_list)>1:
        mapping_time = np.mean(np.array(time_list))

    prt = bag.read_messages(topics=['/projection_time'])
    time_list = []
    for topic, msg, t in prt:
        if t>start_t and t< end_t:
            time_list.append(msg.data)
    if len(time_list)>1:
        projection_time = np.mean(np.array(time_list))
        
    return output()

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("\033[31mPlease use: 'python3 bag_paser_.py <bag_path> <test_case_num> <baseline_name>' to parse bags.\033[0m")
        exit(-1)
    else:
        bag_list = os.listdir(sys.argv[1])
        parse_list = []
        for name in bag_list:
            if sys.argv[2]+"_"+sys.argv[3] in name:
                parse_list.append(name)
        print("find %d packages." % len(parse_list))
        max_v_list = []
        avg_v_list = []
        traj_len_list = []
        max_kappa_list = []
        avg_kappa_list = []
        success_list = []
        min_reach_dist_list = []
        mapping_time_list = []
        planning_time_list = []
        projection_time_list = []
        acc_list = []
        jerk_list = []
        for name in parse_list:
            max_v, avg_v, traj_length, max_k, avg_k, success, reach_min_dist, energy_acc, energy_jerk, mapping_time, planning_time, projection_time = parse_one_bag(os.path.join(sys.argv[1], name))
            success_list.append(success)
            if success:
                max_v_list.append(max_v)
                avg_v_list.append(avg_v)
                traj_len_list.append(traj_length)
                max_kappa_list.append(max_k)
                avg_kappa_list.append(avg_k)
                min_reach_dist_list.append(reach_min_dist)
                if type(mapping_time) == type(None):
                    mapping_time = 0
                mapping_time_list.append(mapping_time)
                if type(planning_time) == type(None):
                    planning_time = 0
                planning_time_list.append(planning_time)
                if type(projection_time) == type(None):
                    projection_time = 0
                projection_time_list.append(projection_time)
                acc_list.append(energy_acc)
                jerk_list.append(energy_jerk)
        
        print("=== Parsing %d packages completed, %d success ===" % (len(parse_list), sum(success_list)))
        print("max_v: ", sum(max_v_list) / len(max_v_list), "average_v: ", sum(avg_v_list) / len(avg_v_list))
        print("traj length(m): ", sum(traj_len_list) / len(traj_len_list), ", max_k: ", sum(max_kappa_list) / len(max_kappa_list), ", avg_k: ", sum(avg_kappa_list) / len(avg_kappa_list))
        print("success: ", sum(success_list) / len(success_list), ", min reached dist: ", sum(min_reach_dist_list) / len(min_reach_dist_list))
        print("mapping time(ms): ", sum(mapping_time_list) / len(mapping_time_list), ", planning time(ms): ", sum(planning_time_list) / len(planning_time_list), ", projection time(ms): ", sum(projection_time_list) / len(projection_time_list))
        print("energy(acc): ", sum(acc_list) / len(acc_list), ", energy(jerk): ", sum(jerk_list) / len(jerk_list))
        print("========(std)========")
        print("average_v: ", np.std(avg_v_list))
        print("avg_k: ", np.std(avg_kappa_list))
        print("mapping time: ", np.std(mapping_time_list), ", planning time(ms): ", np.std(planning_time_list), ", projection time(ms): ", np.std(projection_time_list))
        print("acc: ", np.std(acc_list), ", jerk: ", np.std(jerk_list))
        print("=====================")
        
