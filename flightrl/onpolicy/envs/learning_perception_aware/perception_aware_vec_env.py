from onpolicy.envs.base_vec_env import BaseVecEnv
import torch
import numpy as np
from onpolicy.config import get_config
from gym import Env, spaces
import collections
import open3d as o3d
import time
import os
import sys
import copy

class PAVecEnv(BaseVecEnv):
    def __init__(self, all_args):
        super(PAVecEnv, self).__init__(all_args)
        obs_space_dict = {
            'imu': spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                              shape=(self.args.state_len * self.n_imu, ),
                              dtype="float32"),
            'wb': spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                              shape=(4, 3),
                              dtype="float32"), # waypoint boundary: four corner position of next door
            'cfp': spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                              shape=(3, ),
                              dtype="float32") # Farest CF point: x, y, z
        }
        
        self.observation_space = [spaces.Dict(obs_space_dict)]
        self.share_observation_space = [spaces.Dict(obs_space_dict)]

        self.reward_type = ("total", "progress", "diff_progress", "waypoint", "crash", "omega", "PA")

        # about gates
        self.gate_corners = np.zeros((self.args.gate_num, 4, 3), dtype=np.float32)
        self.gate_centers = np.zeros((self.args.gate_num, 3), dtype=np.float32)
        self.calculate_corners()
        self.current_gate = np.zeros(self.num_envs, dtype = int)
        self.reach_gate = None

        # about topo path
        self.end_point = None
        self.valid_vectors = []
        self.total_len = 0.0
        self.guide_path = self.read_guide_path() # a list of gate_id length, with np.array of (point_num,3) each element
        # will set end_point, total_len and valid_vectors
        self.valid_vector_num = np.zeros(int(np.ceil(self.total_len)), dtype = int)
        self.progress = np.zeros(self.num_envs, dtype = np.float32)
        self.last_progress = np.zeros(self.num_envs, dtype = np.float32)
        self.dist = np.zeros(self.num_envs, dtype = np.float32)
        self.cfp = None
        self.action_queue = collections.deque(maxlen=3)

        # about pcd
        pcd_origin = o3d.io.read_point_cloud(os.path.join(os.environ["FLIGHTMARE_PATH"], self.args.scene_path, "env-surface.ply"))
        self.pcd = pcd_origin.voxel_down_sample(voxel_size=0.15)
        self.pcd_np = np.asarray(self.pcd.points)
        self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        self.pcd_max = self.pcd.get_max_bound() - 0.15
        self.pcd_min = self.pcd.get_min_bound() + 0.15
        self.crash = None

        self.stage_two = False


        """
        args need to be add:
        - gate_num
        - gatei_x
        - gatei_y
        - gatei_z
        - gatei_angle(in deg)
        - gate_radius: default 0.5
        - scene_id: 
        - scene_path:
        """

    def reset_values(self, random = False):
        self.current_gate = np.zeros(self.num_envs, dtype = int)
        self.reach_gate = None
        self.crash = None
        self.valid_vectors = []
        self.valid_vector_num = np.zeros(int(np.ceil(self.total_len)), dtype = int)
        tmp_state = np.zeros(self.n_imu, dtype=np.float32)
        tmp_state[:3] = self.guide_path[0][0, :3]
        angle = np.arctan2(self.guide_path[0][0, 4] - self.guide_path[0][0, 1], self.guide_path[0][0, 3] - self.guide_path[0][0, 0])
        tmp_state[3] = np.cos(angle / 2) # w
        tmp_state[6] = np.sin(angle / 2)# z
        if random:
            print("random set start pos!")
            for i in range(3):
                p_disturb = np.random.uniform(-0.15, 0.15, 3)
                q_disturb = np.random.uniform(-0.06, 0.06, 4)
                v_disturb = np.random.uniform(-0.3, 0.3, 3)
                w_disturb = np.random.uniform(-0.3, 0.3, 3)
                disturb_state = copy.deepcopy(tmp_state)
                disturb_state[:3] += p_disturb
                disturb_state[3:7] += q_disturb
                disturb_state[3:7] /= np.linalg.norm(disturb_state[3:7])
                disturb_state[7:10] += v_disturb
                disturb_state[10:13] += w_disturb
                self.valid_vectors.append(disturb_state)
        self.valid_vectors.append(tmp_state)
        self.progress = np.zeros(self.num_envs, dtype = np.float32)
        self.last_progress = np.zeros(self.num_envs, dtype = np.float32)
        self.dist = np.zeros(self.num_envs, dtype = np.float32)
        self.stage_two = False
        self.cfp = None

    def setStageTwo(self):
        self.stage_two = True

    def setStageOne(self):
        self.stage_two = False

    def step(self, action):
        # input action: num_envs, num_agent, 4
        # process action
        # start = time.time()
        action_full = np.tanh(action).reshape(self.num_envs, 4).astype(np.float32)
        self.action_queue.append(action_full.copy())
        # if len(self.action_queue) < 2:
        #     action_use = self.action_queue[0]
        # else:
        #     if np.random.uniform()<0.5: #50%
        #         action_use = self.action_queue[1].copy()
        #     else:
        #         action_use = self.action_queue[0].copy()
        action_use = action_full.copy()
        self.inner.step(action_use, self._observation, self._done, self._extraInfo)
        # start = time.time()
        
        # check done and change activate ids
        done_outside = self.check_task_progress()
        done = self._done | done_outside

        # init info
        if len(self._extraInfoNames) != 0:
            info = [{'extra_info': {
                self._extraInfoNames[j]: self._extraInfo[i, j] for j in range(0, len(self._extraInfoNames))
            }, 'episode': {}} for i in self.num_envs]
        else:
            info = [{'extra_info': {}, 'episode': {}} for i in range(self.num_envs)]

        for i in range(self.num_envs):
            if done[i]:
                info[i]['extra_info']['done_position'] = copy.deepcopy(self._observation[i, :3])

        # Reset the doned envs
        start_pos = self.random_start_pos().astype(np.float32)
        self.inner.autoReset(start_pos, done)
        self.inner.autoRandomize(done)
        for i in range(self.num_envs):
            if(done[i]):
                self._observation[i,:] = start_pos[i, :]
        # start = time.time()

        self.update_queue(self._observation, if_reset = False)
        obs = self.get_obs()
        # start = time.time()

        # cal reward
        reward = self.cal_reward()

        # start = time.time()

        # procress info
        for key in self.reward_type:
            for i in range(self.num_envs):
                self.rewards[key][i].append(reward[key][i])
                if done[i]:
                    eprew = sum(self.rewards[key][i])
                    eplen = len(self.rewards[key][i])
                    # epinfo = {"r": eprew, "l": eplen}
                    info[i]['episode']['l'] = eplen
                    info[i]['episode'][key] = eprew
                    self.rewards[key][i].clear()
            
        # print("step, act env id: ", self.active_env_ids)

        return copy.deepcopy(obs), copy.deepcopy(reward["total"].reshape(-1,1,1)), copy.deepcopy(done.reshape(-1,1)), copy.deepcopy(info)
    
    def depthStep(self, action):
        # input action: num_envs, num_agent, 4
        # process action
        # start = time.time()
        action_full = np.tanh(action).reshape(self.num_envs, 4).astype(np.float32)
        self.action_queue.append(action_full.copy())
        if len(self.action_queue) < 2:
            action_use = self.action_queue[0]
        else:
            if np.random.uniform()<0.5: #50%
                action_use = self.action_queue[1].copy()
            else:
                action_use = self.action_queue[0].copy()
        imgs = np.zeros((self.num_envs, 320 * 240), dtype = np.float32)
        self.inner.depthStep(action_use, self._observation, imgs, self._done, self._extraInfo)
        imgs = imgs.reshape(self.num_envs, 240, 320, order = 'F')
        far_clip = imgs > 4.10
        imgs = np.zeros_like(imgs) * far_clip + imgs * (~far_clip)
        # print("shape: ", imgs.shape)
        # print("max: ", np.max(imgs))
        # print("min: ", np.min(imgs))
        # print("=== depth step done ===")
        # start = time.time()
        
        # check done and change activate ids
        done_outside = self.check_task_progress()
        done = self._done | done_outside

        # init info
        if len(self._extraInfoNames) != 0:
            info = [{'extra_info': {
                self._extraInfoNames[j]: self._extraInfo[i, j] for j in range(0, len(self._extraInfoNames))
            }, 'episode': {}} for i in self.num_envs]
        else:
            info = [{'extra_info': {}, 'episode': {}} for i in range(self.num_envs)]

        for i in range(self.num_envs):
            if done[i]:
                info[i]['extra_info']['done_position'] = copy.deepcopy(self._observation[i, :3])

        # Reset the doned envs
        start_pos = self.random_start_pos().astype(np.float32)
        self.inner.autoReset(start_pos, done)
        self.inner.autoRandomize(done)
        for i in range(self.num_envs):
            if(done[i]):
                self._observation[i,:] = start_pos[i, :]
        # start = time.time()

        self.update_queue(self._observation, if_reset = False)
        obs = self.get_obs()
        # start = time.time()

        # cal reward
        reward = self.cal_reward()

        # start = time.time()

        # procress info
        for key in self.reward_type:
            for i in range(self.num_envs):
                self.rewards[key][i].append(reward[key][i])
                if done[i]:
                    eprew = sum(self.rewards[key][i])
                    eplen = len(self.rewards[key][i])
                    # epinfo = {"r": eprew, "l": eplen}
                    info[i]['episode']['l'] = eplen
                    info[i]['episode'][key] = eprew
                    self.rewards[key][i].clear()
            
        # print("step, act env id: ", self.active_env_ids)

        return copy.deepcopy(obs), copy.deepcopy(reward["total"].reshape(-1,1,1)), copy.deepcopy(done.reshape(-1,1)), copy.deepcopy(info), imgs.copy()

    def ros_get_obs(self, state):
        self._observation[0, :] = state
        # check done and change activate ids
        done_outside = self.check_task_progress()
        done = copy.deepcopy(done_outside)
        print("gate: ", self.current_gate[0])
        print("progress: ", self.progress[0])
        print("last progress: ", self.last_progress[0])

        # init info
        if len(self._extraInfoNames) != 0:
            info = [{'extra_info': {
                self._extraInfoNames[j]: self._extraInfo[i, j] for j in range(0, len(self._extraInfoNames))
            }, 'episode': {}} for i in self.num_envs]
        else:
            info = [{'extra_info': {}, 'episode': {}} for i in range(self.num_envs)]

        for i in range(self.num_envs):
            if done[i]:
                info[i]['extra_info']['done_position'] = copy.deepcopy(self._observation[i, :3])

        # start = time.time()

        self.update_queue(self._observation, if_reset = False)
        obs = self.get_obs()
        # start = time.time()

        # cal reward
        reward = self.cal_reward()

        # start = time.time()

        # procress info
        for key in self.reward_type:
            for i in range(self.num_envs):
                self.rewards[key][i].append(reward[key][i])
                if done[i]:
                    eprew = sum(self.rewards[key][i])
                    eplen = len(self.rewards[key][i])
                    # epinfo = {"r": eprew, "l": eplen}
                    info[i]['episode']['l'] = eplen
                    info[i]['episode'][key] = eprew
                    self.rewards[key][i].clear()
            
        # print("step, act env id: ", self.active_env_ids)
        return copy.deepcopy(obs), copy.deepcopy(reward["total"].reshape(-1,1,1)), copy.deepcopy(done.reshape(-1,1)), copy.deepcopy(info)

    def get_obs(self): #[env0: {'imu': [...],'pcd': [...]}, ..., envi: {}]
        """
        Modify for different observation form.
        """

        obs = {}
        imu = np.zeros((self.num_envs, 1, self.args.state_len * self.n_imu), dtype = np.float32)
        wb = np.zeros((self.num_envs, 1, 4, 3), dtype = np.float32)
        cfp = np.zeros((self.num_envs, 1, 3), dtype = np.float32)
        for i in range(self.num_envs):
            each_env_imu_list = []
            for j in range(self.args.state_len):
                each_env_imu_list.append(self.state_queue[-j-1][i])
            imu[i, :, :] = np.array([np.concatenate(each_env_imu_list)])
            wb[i, :, :, :] = np.array([self.gate_corners[self.current_gate[i]]])
            cfp[i, :, :] = np.array([self.cal_cfp(i)])
        obs["imu"] = imu
        obs["wb"] = wb
        obs["cfp"] = cfp
        self.cfp = copy.deepcopy(cfp[:, 0, :])
        return obs

    def random_start_pos(self):
        # use valid vectors to get random start pose
        sample = np.random.choice(len(self.valid_vectors), size = self.num_envs)
        return np.stack(self.valid_vectors)[sample, :]

    def check_task_progress(self):
        done = np.zeros(self.num_envs, dtype = bool)
        for i in range(self.num_envs):
            flag = False
            if len(self.rewards["total"][i]) > self.args.episode_length:
                flag = True
            elif self.stage_two and len(self.rewards["total"][i]) > self.args.stage_two_episode_length:
                flag = True
            elif (self._observation[i, 0:3] > self.pcd_max).any() or (self._observation[i, 0:3] < self.pcd_min).any():
                # oob
                flag = True
            else:
                [k_, idx_, _] = self.pcd_tree.search_radius_vector_3d(self._observation[i, 0:3], 0.15)
                if k_ >=1:
                    # print("k: ", k_)
                    # crash
                    flag = True
            done[i] = flag
        self.crash = copy.deepcopy(done)

        # update curr_gate
        need_update = (np.linalg.norm(self._observation[:, :3]-self.gate_centers[self.current_gate, :], axis = 1) < self.args.gate_radius)
        # print("need_update: ", need_update)
        # print("end_point: ", self.end_point, "gate_center: ", self.gate_centers[0, :])
        self.reach_gate = copy.deepcopy(need_update * np.exp(-(need_update * np.linalg.norm(self._observation[:, :3]-self.gate_centers[self.current_gate, :], axis = 1) / self.args.gate_radius)))
        self.current_gate  = (self.current_gate + need_update.astype(int))
        reach = self.current_gate >= self.args.gate_num
        need_reset = done | reach
        self.current_gate = np.minimum(self.current_gate, self.args.gate_num-1)
        
        # update progress
        self.last_progress = copy.deepcopy(self.progress)
        for i in range (self.num_envs):
            path = self.guide_path[self.current_gate[i]]

            if self.current_gate[i]>0:
                previous_progress = 0.0
                for g in range(self.current_gate[i]):
                    for d in range(self.guide_path[g].shape[0]):
                        previous_progress+=np.linalg.norm(self.guide_path[g][d, 3:] - self.guide_path[g][d, :3])
            else:
                previous_progress = 0.0

            min_dist = np.inf
            min_idx = None
            for d in range(path.shape[0]):
                distance = np.linalg.norm(np.cross(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3])) / np.linalg.norm(path[d, :3]-path[d, 3:])
                if_in = np.dot(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3]) / np.linalg.norm(path[d, 3:]-path[d, :3]) > -1.0 and np.dot(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3]) <= np.linalg.norm(path[d, 3:]-path[d, :3]) ** 2 * 1.1
                if if_in:
                    if distance < min_dist:
                        min_dist = distance
                        min_idx = d
            progress_tmp = 0.0
            if min_idx == None:
                progress_tmp = self.last_progress[i] - previous_progress - 0.3
                min_dist = self.args.gate_radius
            elif min_idx > 0:
                for p in range(0, min_idx):
                    progress_tmp+=np.linalg.norm(path[p, 3:]-path[p, :3])
                progress_tmp+=np.dot(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3]) / np.linalg.norm(path[min_idx, 3:]-path[min_idx, :3])
                min_dist = np.linalg.norm(np.cross(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3])) / np.linalg.norm(path[min_idx, :3]-path[min_idx, 3:])
            else:
                progress_tmp=np.dot(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3]) / np.linalg.norm(path[min_idx, 3:]-path[min_idx, :3])
                min_dist = np.linalg.norm(np.cross(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3])) / np.linalg.norm(path[min_idx, :3]-path[min_idx, 3:])
            self.progress[i] = copy.deepcopy(min(progress_tmp + previous_progress, self.total_len))
            self.dist[i] = copy.deepcopy(min_dist)
            # just for check progress cal
            # if i == 0:
            #     if np.random.uniform()>0.9:
            #         print("position: ", self._observation[i, :3])
            #         print("progress: ", self.progress[i])
            #         print("dist: ", self.dist[i])

            # for debug, will not appear during training
            # if self.progress[i] >= self.total_len:
            #     print("progress: ", self.progress[i])
            #     print("position: ", self._observation[i, :3])
            #     print("min_idx: ", min_idx)
            #     print("min_dist: ", min_dist)
        
        # update valid vector and if need to be reset
        for i in range (self.num_envs):
            if not need_reset[i] and self.progress[i] < self.total_len * 0.7 and self.valid_vector_num[max(0,int(np.floor(self.progress[i])))] < self.num_envs and self.current_gate[i] == 0:
                if self.dist[i] < 0.3  and np.linalg.norm(self._observation[i, 7:10]) >= 1.0 and np.linalg.norm(self._observation[i, 7:10]) <= 2.0 and np.linalg.norm(self._observation[i, 10:13]) < 4.0:
                    if np.random.uniform()>0.8:
                        # print("add to valid vector: ", self.valid_vector_num[max(0,int(np.floor(self.progress[i])))])
                        self.valid_vector_num[max(0,int(np.floor(self.progress[i])))]+=1
                        self.valid_vectors.append(self._observation[i].astype(np.float32))
            if need_reset[i]:
                self.current_gate[i] = 0
                self.progress[i] = 0.0
                self.last_progress[i] = 0.0
                self.dist[i] = 0.0
        return need_reset
    
    def cal_reward(self):
        """
        Method to calculate reward
        return reward: np.array of (num_env, ) ftype = np.float32
        """
        kp_ = 5.0
        ks_ = 0.002
        kwp = 6.5
        kpa = 0.06
        komega = 0.02
        vmax = 2.0
        vmin = 1.0
        dmax = 0.3
        kT = 0.5

        if self.stage_two:
            dmax = 0.9
            vmax = 20.0
            vmin = 2.0
            kpa = 0.12

        rs = copy.deepcopy(self.progress)
        rp = copy.deepcopy(self.progress - self.last_progress)
        rT = copy.deepcopy(-10*self.crash)
        romega = -np.linalg.norm(self._observation[:, 10:13], axis = 1)
        rwp = copy.deepcopy(self.reach_gate)

        # cal rpa
        theta_cfp = np.arctan2(self.cfp[:, 1], self.cfp[:, 0])
        w = self._observation[:, 3]
        x = self._observation[:, 4]
        y = self._observation[:, 5]
        z = self._observation[:, 6]
        theta_quad = np.arctan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        delta_theta = np.minimum(abs(theta_cfp - theta_quad), abs(theta_cfp - theta_quad - np.sign(theta_cfp - theta_quad)*2*np.pi))
        rpa = np.exp(-delta_theta)

        if_in_vmax = np.linalg.norm(self._observation[:, 7:10], axis = 1) <= vmax
        s_vmax = np.ones(self.num_envs) * if_in_vmax + ~if_in_vmax * np.power(10, vmax - np.linalg.norm(self._observation[:, 7:10], axis = 1))
        if_in_vmin = np.linalg.norm(self._observation[:, 7:10], axis = 1) >= vmin
        # print("min of diff: ", np.min(np.linalg.norm(self._observation[:, 7:10], axis = 1) - vmin))
        # print("max of diff: ", np.max(np.linalg.norm(self._observation[:, 7:10], axis = 1) - vmin))
        s_vmin = np.ones(self.num_envs) * if_in_vmin + ~if_in_vmin * np.power(10, ( ~if_in_vmin * np.linalg.norm(self._observation[:, 7:10], axis = 1)) - vmin)
        if_in_dist = self.dist < dmax
        s_gd = np.ones(self.num_envs) * if_in_dist + ~if_in_dist * np.exp(dmax-self.dist)
        kp = kp_ * s_vmax * s_vmin * s_gd
        ks = ks_ * s_vmax * s_vmin * s_gd

        # if np.random.uniform()>0.95:
        #     print("position: ", self._observation[0, :3])
        #     print("progress: ", self.progress[0])
        #     print("dist: ", self.dist[0])
        #     print("v norm: ", np.linalg.norm(self._observation[0, 7:10]))
        #     print("rs: ", rs[0])
        #     print("ks: ", ks[0])
        # print("q: ", self._observation[0, 3:7])
        # print("cfp vec: ", self.cfp[0])
        # print("quad yaw: ", theta_quad[0])
        # print("cfp yaw: ", theta_cfp[0])
        # print("delta theta: ", delta_theta[0])
        # print("rpa: ", rpa[0])
        # print("===== print done =====")

        reward = {"total": kp * rp + ks * rs + kwp * rwp + kT * rT + komega * romega + kpa * rpa, "progress": ks * rs, "diff_progress": kp * rp, "waypoint": kwp * rwp, "crash": kT * rT, "omega": komega * romega, "PA": kpa * rpa}

        return reward
    
    def close(self):
        self.reset()

# util functions
    def calculate_corners(self):
        for i in range(self.args.gate_num):
            x = getattr(self.args, "gate"+str(i)+"_x")
            y = getattr(self.args, "gate"+str(i)+"_y")
            z = getattr(self.args, "gate"+str(i)+"_z")
            
            position = np.array([x, y, z], dtype=np.float32)
            radius = float(self.args.gate_radius)
            angle = getattr(self.args, "gate"+str(i)+"_angle") * np.pi / 180
            rot = np.array([[np.cos(angle), -np.sin(angle), 0], 
                            [np.sin(angle), np.cos(angle), 0], 
                            [0, 0, 1]], dtype = np.float32)
            self.gate_corners[i, 0, :] = position + rot @ np.array([0, radius, radius])
            self.gate_corners[i, 1, :] = position + rot @ np.array([0, radius, -radius])
            self.gate_corners[i, 2, :] = position + rot @ np.array([0, -radius, radius])
            self.gate_corners[i, 3, :] = position + rot @ np.array([0, -radius, -radius])
            self.gate_centers[i, :] = position
    
    def cal_cfp(self, i):
        path = self.guide_path[self.current_gate[i]]
        # min_dist = np.inf
        # min_idx = 0
        # for d in range(path.shape[0]):
        #     distance = np.linalg.norm(np.cross(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3])) / np.linalg.norm(path[d, :3]-path[d, 3:])
        #     if_in = np.dot(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3]) > -0.05
        #     if if_in:
        #         if distance < min_dist:
        #             min_dist = distance
        #             min_idx = d
        # min_dist = np.linalg.norm(np.cross(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3])) / np.linalg.norm(path[min_idx, :3]-path[min_idx, 3:])

        min_dist = np.inf
        min_idx = None
        for d in range(path.shape[0]):
            distance = np.linalg.norm(np.cross(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3])) / np.linalg.norm(path[d, :3]-path[d, 3:])
            if_in = np.dot(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3]) / np.linalg.norm(path[d, 3:]-path[d, :3]) > -1.0 and np.dot(self._observation[i, :3] - path[d, :3], path[d, 3:]-path[d, :3]) <= np.linalg.norm(path[d, 3:]-path[d, :3]) ** 2 * 1.1
            if if_in:
                if distance < min_dist:
                    min_dist = distance
                    min_idx = d

        if min_idx == None:
            return self.gate_centers[self.current_gate[i], :] - self._observation[i, :3]

        # start = time.time()
        progress=np.dot(self._observation[i, :3] - path[min_idx, :3], path[min_idx, 3:]-path[min_idx, :3]) / np.linalg.norm(path[min_idx, 3:]-path[min_idx, :3])
        progress = min(progress, np.linalg.norm(path[min_idx, 3:]-path[min_idx, :3]))
        next_cfp = path[min_idx, :3]+ progress / np.linalg.norm(path[min_idx, 3:]-path[min_idx, :3]) * (path[min_idx, 3:]-path[min_idx, :3])
        # print("get min dist and d: ", time.time()-start)
        # start = time.time()
        progress+=0.25
        free = True
        for d in range(min_idx, path.shape[0]):
            # in d part
            while progress / np.linalg.norm(path[d, 3:]-path[d, :3]) < 1:
                cfp_candidate = path[d, :3]+ progress / np.linalg.norm(path[d, 3:]-path[d, :3]) * (path[d, 3:]-path[d, :3])
                vector = cfp_candidate - self._observation[i, :3]
                norm = np.linalg.norm(vector)
                if norm > 4.5:
                    free = False
                    break
                check = 0.1
                while(check<norm):
                    #start_search = time.time()
                    search_center = self._observation[i, 0:3] + check / norm * vector
                    [k_, idx_, _] = self.pcd_tree.search_knn_vector_3d(search_center, 1)
                    min_dist_search = np.linalg.norm(self.pcd_np[idx_]-search_center)
                    #print("search once time: ", time.time()-start_search, ", norm: ", min_dist_search)
                    #print("search norm: ", min_dist_search)
                    # total_num += 1
                    if min_dist_search <= 0.15:
                        free = False
                        break
                    check += max(0.2, min_dist_search)
                if not free:
                    break
                else:
                    next_cfp = copy.deepcopy(cfp_candidate)
                    progress += 0.2
            progress = 0.1
            if not free:
                break

        # print("get cfp: ", time.time()-start)
        # print("search times: ", total_num)
        
        return next_cfp - self._observation[i, :3]

    def read_guide_path(self):
        # return guide_traj: a list of (gate_num, traj_num), np.array of (point_num, 3) for each element
        guide_path = []
        for i in range(self.args.gate_num):
            f_name = os.path.join(os.environ["FLIGHTMARE_PATH"], self.args.scene_path, "roadmap_shortened_unique_path"+str(i)+"_0.csv")
            if os.path.exists(f_name):
                points = np.loadtxt(f_name, delimiter=',').reshape(-1, 6)
                guide_path.append(points)
                if np.linalg.norm(self.gate_centers[i]-points[-1, 3:]) > self.args.gate_radius:
                    print("mismatch between gate in args and topo path in files")
                    exit(-1)
                for d in range(points.shape[0]):
                    self.total_len+=np.linalg.norm(points[d, :3]-points[d, 3:])
            else:
                print("no topo path to gate ", i)
                exit(-1)
        self.end_point = self.gate_centers[-1, :]
        tmp_state = np.zeros(self.n_imu, dtype=np.float32)
        tmp_state[:3] = guide_path[0][0, :3]
        angle = np.arctan2(guide_path[0][0, 4] - guide_path[0][0, 1], guide_path[0][0, 3] - guide_path[0][0, 0])
        tmp_state[3] = np.cos(angle / 2) # w
        tmp_state[6] = np.sin(angle / 2)# z
        self.valid_vectors.append(tmp_state)
        print("self.total_len: ", self.total_len)
        for i in range(self.args.gate_num):
            print("gate %d: [%f, %f, %f]: %f" % (i, self.gate_centers[i, 0], self.gate_centers[i, 1], self.gate_centers[i, 2], getattr(self.args, "gate"+str(i)+"_angle")) )
        return guide_path

def parse_args(args, parser):
    parser.add_argument("--max_input_seq_len", type=int, default=3)
    parser.add_argument("--state_len", type=int, default=1)
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--scene_id", type=int, default=3)
    parser.add_argument("--scene_path", type=str, default="scene/office")
    parser.add_argument("--vec_env_config", type=str, default="flightlib/configs/vec_env.yaml")
    parser.add_argument("--env_config", type=str, default="flightlib/configs/quadrotor_env.yaml")
    parser.add_argument("--dyn_config", type=str, default="flightlib/configs/dyn_param.yaml")
    parser.add_argument("--gate_num", type=int, default=1)
    parser.add_argument("--gate0_x", type=float, default=7.6)
    parser.add_argument("--gate0_y", type=float, default=1.8)
    parser.add_argument("--gate0_z", type=float, default=2.5)
    parser.add_argument("--gate0_angle", type=float, default=45.0)
    parser.add_argument("--gate_radius", type=float, default=0.5)
    all_args = parser.parse_known_args(args)[0]
    return all_args

def main(args): # main
    parser = get_config()
    all_args = parse_args(args, parser)
    # path = os.environ["FLIGHTMARE_PATH"] +"/flightrl/air_planner/configs/train_config.yaml"
    # config = create_settings(path = path, mode =  "train" if all_args.train else "test")
    # config.if_random_start_pos = all_args.if_random_start_pos

    env = PAVecEnv(all_args)
    env.seed(2)
    obs = env.reset()
    t = 0
    start = time.time()
    while(t<50):
        action = np.zeros((all_args.n_rollout_threads, 4), dtype = np.float32)
        action[0][0] = -0.25
        obs, rewards, dones, infos = env.step(action)
        if t == 0:
            print("obs keys: ", obs.keys())
            for key in obs.keys():
                print("obs "+key+": ", obs[key].shape)
            print("rewards: ", rewards.shape)
            print("dones: ", dones.shape)
        # print("state:", obs[0]["imu"][0])
        # print("len obs: ", len(obs), "obs imu shape: ", obs[0]["imu"].shape, "obs pcd shape: ", obs[0]["pcd"].shape)
        # print("rewards: \n", rewards)
        # print("dones: ", dones)
        # print("infos: ", infos)
        t+=1
    print("time: ", time.time()-start)

if __name__ == "__main__":
    main(sys.argv[1:])