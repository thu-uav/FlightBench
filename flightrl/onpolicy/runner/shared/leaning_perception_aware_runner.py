from collections import defaultdict, deque
from itertools import chain
import os
import time
import copy
import numpy as np
import torch
import wandb
import datetime

from onpolicy.utils.util import update_linear_schedule
from onpolicy.runner.shared.base_runner import Runner

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def _t2n(x):
    return x.detach().cpu().numpy()

class LearningPARunner(Runner):
    def __init__(self, config):
        super(LearningPARunner, self).__init__(config)
        self.env_infos = defaultdict(list)

    def run(self):
        self.warmup()

        start = time.time()
        episodes = int(self.num_env_steps) // self.episode_length // self.n_rollout_threads

        for episode in range(episodes):
            if self.use_linear_lr_decay:
                self.trainer.policy.lr_decay(episode, episodes)
            
            for step in range(self.episode_length):
                # Sample actions
                values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)

                # Obtain reward and next obs
                obs, rewards, dones, infos = self.envs.step(actions)

                data = obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic 
                
                # insert data into buffer
                self.insert(data)

            self.compute()
            train_infos = self.train()
            
            # post process
            total_num_steps = (episode + 1) * self.episode_length * self.n_rollout_threads
            
            # save model
            if (total_num_steps % self.save_interval == 0 or episode == episodes - 1):
                if np.mean(self.env_infos["rewards/crash"]) > -0.8:
                    self.save()

            # log information
            if total_num_steps % self.log_interval == 0:
                end = time.time()
                print("\n Env {} Algo {} Exp {} updates {}/{} episodes, total num timesteps {}/{}, FPS {}.\n"
                        .format(self.env_name,
                                self.algorithm_name,
                                self.experiment_name,
                                episode,
                                episodes,
                                total_num_steps,
                                self.num_env_steps,
                                int(total_num_steps / (end - start))))
                
                train_infos["average_train_episode_rewards"] = np.mean(self.buffer.rewards) * self.episode_length
                print("average train episode rewards is {}".format(train_infos["average_train_episode_rewards"]))
                self.log_train(train_infos, total_num_steps)
                if np.mean(self.env_infos["rewards/crash"]) > -1.0:
                    self.envs.setStageTwo()
                    self.env_infos["extra/stagetwo"].append(2.0)
                else:
                    self.env_infos["extra/stagetwo"].append(1.0)
                self.log_env(self.env_infos, total_num_steps)
                self.env_infos = defaultdict(list)

            # eval
            if total_num_steps % self.eval_interval == 0 and self.use_eval:
                self.eval(total_num_steps)

    def eval_once(self):
        obs = self.eval_envs.reset()
        for key in obs.keys():
            self.buffer.share_obs[key][0] = obs[key].copy()
            self.buffer.obs[key][0] = obs[key].copy()

        key_steps = []
        obs_cfp_vector = []
        obs_ori_vector = []

        for step in range(self.episode_length):
            # Sample actions
            values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)

            # Obtain reward and next obs
            # actions = np.zeros([1,1,4])
            # actions[0,0,0] = 0
            # if step > 30:
            #     actions[0,0,2] = 0.6
            # else:
            #     actions[0,0,3] = 0.2
            obs, rewards, dones, infos = self.envs.step(actions)

            data = obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic 

            if step % 10 == 0:
                key_steps.append(step)
                obs_cfp_vector.append(obs["cfp"][0,0,:])
                obs_ori_vector.append(obs["imu"][0,0,3:7])
            
            # insert data into buffer
            self.insert(data)
        
        print("total: ", self.env_infos["rewards/total"])
        print("s: ", self.env_infos["rewards/s"])
        print("progress: ", self.env_infos["rewards/progress"])
        print("omega: ", self.env_infos["rewards/omega"])
        print("wp: ", self.env_infos["rewards/waypoint"])
        print("crash: ", self.env_infos["rewards/crash"])
        time_axis = np.linspace(0.0, self.episode_length * 0.025, self.episode_length)
        plt.figure()
        plt.plot(time_axis, np.linalg.norm(self.buffer.obs["imu"][:self.episode_length, 0, 0, 7:10], axis = 1), label = "velocity norm")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("velocity norm")
        plt.savefig("velo_norm.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 0], label = "x")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("x-t")
        plt.savefig("x-t.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 7], label = "vx")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 8], label = "vy")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 9], label = "vz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("v-t")
        plt.savefig("v-t.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 1], label = "y")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("y-t")
        plt.savefig("y-t.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 2], label = "z")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("z-t")
        plt.savefig("z-t.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 3], label = "qw")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 4], label = "qx")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 5], label = "qy")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 6], label = "qz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("q-t")
        plt.savefig("q-t.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 10], label = "omegax")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 11], label = "omegay")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 12], label = "omegaz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("omega-t")
        plt.savefig("omega-t.png")

        plt.figure()
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 0]))
        plt.title("collective_thrust")
        plt.savefig("ct.png")

        plt.figure()
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 1]), label = "x")
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 2]), label = "y")
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 3]), label = "z")
        plt.legend()
        plt.title("bodyrates")
        plt.savefig("br.png")

        plt.figure()
        plt.plot(time_axis, self.buffer.rewards[:, 0, 0, 0], label = "rewards")
        plt.legend()
        plt.title("rewards")
        plt.savefig("r.png")

        fig = plt.figure(dpi = 150)
        ax = fig.add_subplot(111, projection='3d')

        # ax.set_xlim(-12, 8)
        # ax.set_ylim(0, 4.5)
        ax.set_zlim(0, 4.5)

        ax.scatter(self.buffer.obs["imu"][:self.episode_length, 0, 0, 0],self.buffer.obs["imu"][:self.episode_length, 0, 0, 1],self.buffer.obs["imu"][:self.episode_length, 0, 0, 2], c = 'b', s = 0.5, label = 'flight')
        for g in range(len(self.envs.guide_path)):
            for p in range(self.envs.guide_path[g].shape[0]):
                if g == 0 and p == 0:
                    ax.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], [self.envs.guide_path[g][p, 2], self.envs.guide_path[g][p, 5]], c = 'r', label = "guide")
                else:
                    ax.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], [self.envs.guide_path[g][p, 2], self.envs.guide_path[g][p, 5]], c = 'r')

        plt.savefig("traj.png")

        for i in range(len(key_steps)):
            step = key_steps[i]
            # cal optical axis
            ori = obs_ori_vector[i]
            rot_matrix = np.array(
                [[1.0 - 2 * (ori[2] * ori[2] + ori[3] * ori[3]), 2 * (ori[1] * ori[2] - ori[0] * ori[3]), 2 * (ori[0] * ori[2] + ori[1] * ori[3])],
                [2 * (ori[1] * ori[2] + ori[0] * ori[3]), 1.0 - 2 * (ori[1] * ori[1] + ori[3] * ori[3]), 2 * (ori[2] * ori[3] - ori[0] * ori[1])],
                [2 * (ori[1] * ori[3] - ori[0] * ori[2]), 2 * (ori[2] * ori[3] + ori[0] * ori[1]), 1.0 - 2 * (ori[1] * ori[1] + ori[2] * ori[2])]],
                dtype=ori.dtype)
            ori_vector = rot_matrix @ np.array([0.8,0,0])
            if i == 0:
                ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], ori_vector[0], ori_vector[1], ori_vector[2], color = (0,0.85,0.46,0.8), label = "orientation")
            else:
                ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], ori_vector[0], ori_vector[1], ori_vector[2], color = (0,0.85,0.46,0.7))
            # ax.quiver(0, 0,0, 3, 3,3,color=(0,1,0,0.5)) 
        ax.legend()
        plt.savefig("traj_ori.png")

        plt.figure()
        plt.scatter(self.buffer.obs["imu"][:self.episode_length, 0, 0, 0],self.buffer.obs["imu"][:self.episode_length, 0, 0, 1], c = 'b', s = 0.5, label = 'traj')
        for g in range(len(self.envs.guide_path)):
            for p in range(self.envs.guide_path[g].shape[0]):
                plt.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], c = 'r')
        plt.savefig('traj2D.png')

        for i in range(len(key_steps)):
            step = key_steps[i]
            # cal optical axis
            ori = obs_ori_vector[i]
            rot_matrix = np.array(
                [[1.0 - 2 * (ori[2] * ori[2] + ori[3] * ori[3]), 2 * (ori[1] * ori[2] - ori[0] * ori[3]), 2 * (ori[0] * ori[2] + ori[1] * ori[3])],
                [2 * (ori[1] * ori[2] + ori[0] * ori[3]), 1.0 - 2 * (ori[1] * ori[1] + ori[3] * ori[3]), 2 * (ori[2] * ori[3] - ori[0] * ori[1])],
                [2 * (ori[1] * ori[3] - ori[0] * ori[2]), 2 * (ori[2] * ori[3] + ori[0] * ori[1]), 1.0 - 2 * (ori[1] * ori[1] + ori[2] * ori[2])]],
                dtype=ori.dtype)
            ori_vector = rot_matrix @ np.array([0.8,0,0])
            if i == 0:
                plt.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], ori_vector[0], ori_vector[1], color = (0,0.85,0.46,0.8), label = "orientation")
            else:
                plt.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], ori_vector[0], ori_vector[1], color = (0,0.85,0.46,0.7))
        plt.legend()
        plt.savefig('traj2D_ori.png')

    def collect_once(self):
        print("collecting...")
        obs = self.eval_envs.reset(random = True)
        for key in obs.keys():
            self.buffer.share_obs[key][0] = obs[key].copy()
            self.buffer.obs[key][0] = obs[key].copy()
        imgs = np.zeros((1, 240, 320), dtype=np.float32)

        key_steps = []
        obs_cfp_vector = []
        obs_ori_vector = []
        date = datetime.datetime.now()
        datestr = date.strftime('%m-%d-%H-%M-%S')
        os.mkdir('../offline_data/'+datestr)

        for step in range(self.episode_length):
            # Sample actions
            values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)

            # Obtain reward and next obs
            # actions = np.zeros([1,1,4])
            # actions[0,0,0] = 0
            # if step > 30:
            #     actions[0,0,2] = 0.6
            # else:
            #     actions[0,0,3] = 0.2
            st_with_at = np.concatenate((obs['imu'][0,0,:], actions[0, 0, :]))
            np.savetxt('../offline_data/'+datestr+'/'+str(step)+'.txt', st_with_at)
            np.save('../offline_data/'+datestr+'/'+str(step)+'.npy', imgs[0])

            obs, rewards, dones, infos, imgs = self.envs.depthStep(actions)

            data = obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic 

            if step % 10 == 0:
                key_steps.append(step)
                obs_cfp_vector.append(obs["cfp"][0,0,:])
                obs_ori_vector.append(obs["imu"][0,0,3:7])
            
            # insert data into buffer
            self.insert(data)
        
        print("total: ", self.env_infos["rewards/total"])
        print("s: ", self.env_infos["rewards/s"])
        print("progress: ", self.env_infos["rewards/progress"])
        print("omega: ", self.env_infos["rewards/omega"])
        print("wp: ", self.env_infos["rewards/waypoint"])
        print("crash: ", self.env_infos["rewards/crash"])
        time_axis = np.linspace(0.0, self.episode_length * 0.025, self.episode_length)
        plt.figure()
        plt.plot(time_axis, np.linalg.norm(self.buffer.obs["imu"][:self.episode_length, 0, 0, 7:10], axis = 1), label = "velocity norm")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("velocity norm")
        plt.savefig("velo_norm.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 0], label = "x")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("x-t")
        plt.savefig("x-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 7], label = "vx")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 8], label = "vy")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 9], label = "vz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("v-t")
        plt.savefig("v-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 1], label = "y")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("y-t")
        plt.savefig("y-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 2], label = "z")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("z-t")
        plt.savefig("z-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 3], label = "qw")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 4], label = "qx")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 5], label = "qy")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 6], label = "qz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("q-t")
        plt.savefig("q-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 10], label = "omegax")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 11], label = "omegay")
        plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 12], label = "omegaz")
        plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
        plt.legend()
        plt.title("omega-t")
        plt.savefig("omega-t.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 0]))
        plt.title("collective_thrust")
        plt.savefig("ct.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 1]), label = "x")
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 2]), label = "y")
        plt.plot(time_axis, np.tanh(self.buffer.actions[:, 0, 0, 3]), label = "z")
        plt.legend()
        plt.title("bodyrates")
        plt.savefig("br.png")
        plt.close()

        plt.figure()
        plt.plot(time_axis, self.buffer.rewards[:, 0, 0, 0], label = "rewards")
        plt.legend()
        plt.title("rewards")
        plt.savefig("r.png")
        plt.close()

        fig = plt.figure(dpi = 150)
        ax = fig.add_subplot(111, projection='3d')

        # ax.set_xlim(-12, 8)
        # ax.set_ylim(0, 4.5)
        ax.set_zlim(0, 4.5)

        ax.scatter(self.buffer.obs["imu"][:self.episode_length, 0, 0, 0],self.buffer.obs["imu"][:self.episode_length, 0, 0, 1],self.buffer.obs["imu"][:self.episode_length, 0, 0, 2], c = 'b', s = 0.5, label = 'flight')
        for g in range(len(self.envs.guide_path)):
            for p in range(self.envs.guide_path[g].shape[0]):
                if g == 0 and p == 0:
                    ax.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], [self.envs.guide_path[g][p, 2], self.envs.guide_path[g][p, 5]], c = 'r', label = "guide")
                else:
                    ax.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], [self.envs.guide_path[g][p, 2], self.envs.guide_path[g][p, 5]], c = 'r')

        plt.savefig("traj.png")
        plt.close()

        for i in range(len(key_steps)):
            step = key_steps[i]
            # cal optical axis
            ori = obs_ori_vector[i]
            rot_matrix = np.array(
                [[1.0 - 2 * (ori[2] * ori[2] + ori[3] * ori[3]), 2 * (ori[1] * ori[2] - ori[0] * ori[3]), 2 * (ori[0] * ori[2] + ori[1] * ori[3])],
                [2 * (ori[1] * ori[2] + ori[0] * ori[3]), 1.0 - 2 * (ori[1] * ori[1] + ori[3] * ori[3]), 2 * (ori[2] * ori[3] - ori[0] * ori[1])],
                [2 * (ori[1] * ori[3] - ori[0] * ori[2]), 2 * (ori[2] * ori[3] + ori[0] * ori[1]), 1.0 - 2 * (ori[1] * ori[1] + ori[2] * ori[2])]],
                dtype=ori.dtype)
            ori_vector = rot_matrix @ np.array([0.8,0,0])
            if i == 0:
                ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], ori_vector[0], ori_vector[1], ori_vector[2], color = (0,0.85,0.46,0.8), label = "orientation")
            else:
                ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], ori_vector[0], ori_vector[1], ori_vector[2], color = (0,0.85,0.46,0.7))
            # ax.quiver(0, 0,0, 3, 3,3,color=(0,1,0,0.5)) 
        ax.legend()
        plt.savefig("traj_ori.png")
        plt.close()

        plt.figure()
        plt.scatter(self.buffer.obs["imu"][:self.episode_length, 0, 0, 0],self.buffer.obs["imu"][:self.episode_length, 0, 0, 1], c = 'b', s = 0.5, label = 'traj')
        for g in range(len(self.envs.guide_path)):
            for p in range(self.envs.guide_path[g].shape[0]):
                plt.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], c = 'r')
        plt.savefig('traj2D.png')
        plt.close()

        for i in range(len(key_steps)):
            step = key_steps[i]
            # cal optical axis
            ori = obs_ori_vector[i]
            rot_matrix = np.array(
                [[1.0 - 2 * (ori[2] * ori[2] + ori[3] * ori[3]), 2 * (ori[1] * ori[2] - ori[0] * ori[3]), 2 * (ori[0] * ori[2] + ori[1] * ori[3])],
                [2 * (ori[1] * ori[2] + ori[0] * ori[3]), 1.0 - 2 * (ori[1] * ori[1] + ori[3] * ori[3]), 2 * (ori[2] * ori[3] - ori[0] * ori[1])],
                [2 * (ori[1] * ori[3] - ori[0] * ori[2]), 2 * (ori[2] * ori[3] + ori[0] * ori[1]), 1.0 - 2 * (ori[1] * ori[1] + ori[2] * ori[2])]],
                dtype=ori.dtype)
            ori_vector = rot_matrix @ np.array([0.8,0,0])
            if i == 0:
                plt.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], ori_vector[0], ori_vector[1], color = (0,0.85,0.46,0.8), label = "orientation")
            else:
                plt.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], ori_vector[0], ori_vector[1], color = (0,0.85,0.46,0.7))
        plt.legend()
        plt.savefig('traj2D_ori.png')
        plt.close()

    def warmup(self):
        # reset env
        obs = self.envs.reset()

        # insert obs to buffer
        for key in obs.keys():
            self.buffer.share_obs[key][0] = obs[key].copy()
            self.buffer.obs[key][0] = obs[key].copy()
    
    @torch.no_grad()
    def collect(self, step):
        self.trainer.prep_rollout()

        obs_concate = {}
        share_obs_concate = {}
        # [n_envs, n_agents, ...] -> [n_envs*n_agents, ...]
        for key in self.buffer.obs.keys():
            obs_concate[key] = np.concatenate(self.buffer.obs[key][step])
        for key in self.buffer.share_obs.keys():
            share_obs_concate[key] = np.concatenate(self.buffer.share_obs[key][step])

        values, actions, action_log_probs, rnn_states, rnn_states_critic = self.trainer.policy.get_actions(
            obs_concate,
            share_obs_concate,
            np.concatenate(self.buffer.rnn_states[step]),
            np.concatenate(self.buffer.rnn_states_critic[step]),
            np.concatenate(self.buffer.masks[step])
        )

        # [n_envs*n_agents, ...] -> [n_envs, n_agents, ...]
        values = np.array(np.split(_t2n(values), self.n_rollout_threads))
        actions = np.array(np.split(_t2n(actions), self.n_rollout_threads))
        action_log_probs = np.array(np.split(_t2n(action_log_probs), self.n_rollout_threads))
        rnn_states = np.array(np.split(_t2n(rnn_states), self.n_rollout_threads))
        rnn_states_critic = np.array(np.split(_t2n(rnn_states_critic), self.n_rollout_threads))

        return values, actions, action_log_probs, rnn_states, rnn_states_critic

    def insert(self, data):
        obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic = data
        
        # update env_infos if done
        dones_env = np.all(dones, axis=-1)
        if np.any(dones_env):
            for done, info in zip(dones_env, infos):
                if done:
                    self.env_infos["rewards/total"].append(copy.deepcopy(info["episode"]["total"]))
                    self.env_infos["rewards/s"].append(copy.deepcopy(info["episode"]["progress"]))
                    self.env_infos["rewards/progress"].append(copy.deepcopy(info["episode"]["diff_progress"]))
                    self.env_infos["rewards/waypoint"].append(copy.deepcopy(info["episode"]["waypoint"]))
                    self.env_infos["rewards/crash"].append(copy.deepcopy(info["episode"]["crash"]))
                    self.env_infos["rewards/omega"].append(copy.deepcopy(info["episode"]["omega"]))
                    self.env_infos["rewards/PA"].append(copy.deepcopy(info["episode"]["PA"]))
                    self.env_infos["extra/episode_steps"].append(copy.deepcopy(info["episode"]["l"]))
                    self.env_infos["extra/donepos_x"].append(copy.deepcopy(info["extra_info"]["done_position"][0]))
                    self.env_infos["extra/donepos_y"].append(copy.deepcopy(info["extra_info"]["done_position"][1]))
                    self.env_infos["extra/donepos_z"].append(copy.deepcopy(info["extra_info"]["done_position"][2]))

        # reset rnn and mask args for done envs
        rnn_states[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        rnn_states_critic[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        masks[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        self.buffer.insert(
            share_obs=obs,
            obs=obs,
            rnn_states=rnn_states,
            rnn_states_critic=rnn_states_critic,
            actions=actions,
            action_log_probs=action_log_probs,
            value_preds=values,
            rewards=rewards,
            masks=masks
        )

    def log_env(self, env_infos, total_num_steps):
        for k, v in env_infos.items():
            if len(v) > 0:
                if self.use_wandb:
                    wandb.log({k: np.mean(v)}, step=total_num_steps)
                else:
                    self.writter.add_scalars(k, {k: np.mean(v)}, total_num_steps)    
