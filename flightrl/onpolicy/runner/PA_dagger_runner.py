from collections import defaultdict, deque
from itertools import chain
import os
import time
import copy
import numpy as np
import torch
import wandb
import datetime
import cv2

from onpolicy.utils.util import update_linear_schedule
from onpolicy.runner.shared.base_runner import Runner
from onpolicy.runner.student_trainer import Trainer

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def _t2n(x):
    return x.detach().cpu().numpy()

class PADaggerRunner(Runner):
    def __init__(self, config):
        super(PADaggerRunner, self).__init__(config)
        self.env_infos = defaultdict(list)
        self.vision_policy = Trainer(data_dir = self.all_args.data_dir, eval_dir = self.all_args.eval_dir, pretrain_dir=self.all_args.pretrain_dir)
        self.beta = 0.0

    def run(self):
        start = time.time()
        episodes = int(self.num_env_steps) // self.episode_length // self.n_rollout_threads

        self.beta = 0.05

        device_student = self.vision_policy.device

        for episode in range(episodes):
            if self.use_linear_lr_decay:
                self.trainer.policy.lr_decay(episode, episodes)
            
            print("==== start episode", episode, "====")

            obs = self.envs.reset(random = True)

            # insert obs to buffer
            for key in obs.keys():
                self.buffer.share_obs[key][0] = obs[key].copy()
                self.buffer.obs[key][0] = obs[key].copy()

            imgs_square = np.zeros((1, 1, 112, 112), dtype = np.float32)
            imgs = np.zeros((1, 240, 320), dtype=np.float32)
            date = datetime.datetime.now()
            datestr = date.strftime('%m-%d-%H-%M-%S')
            folder_path = os.path.join(self.all_args.data_dir, datestr)
            if not os.path.exists(folder_path):
                os.mkdir(folder_path)

            max_rollout = 600
            file_list = os.listdir(self.all_args.data_dir)
            if len(file_list) > max_rollout:
                file_list.sort()
                rm_list = file_list[:(len(file_list)-max_rollout)]
                for rm_dir in rm_list:
                    rm_full_path = os.path.join(self.all_args.data_dir, rm_dir)
                    command = "rm -r " + rm_full_path
                    os.system(command)

            for step in range(self.episode_length):
                # Sample actions with teacher
                values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)
                if np.random.uniform() < self.beta:
                    # Sample actions with student
                    imu_input = np.zeros((1,9+3), dtype = np.float32)
                    quat = obs["imu"][0, 0, 3:7]
                    R = np.array(
                    [[1.0 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[0] * quat[2] + quat[1] * quat[3])],
                    [2 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]), 2 * (quat[2] * quat[3] - quat[0] * quat[1])],
                    [2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), 1.0 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])]])
                    imu_input[0, :9] = R.reshape(9)
                    imu_input[0, 9:12] = obs["imu"][0, 0, 7:10]
                    actions_student = self.vision_policy.net(depth = torch.tensor(imgs_square).to(device_student), imu = torch.tensor(imu_input).to(device_student))
                    actions_student = actions_student.detach().cpu().numpy()
                    actions_student = actions_student.reshape(-1, 1, 4)

                    # log state and label
                    st_with_at = np.concatenate((obs['imu'][0, 0, :], actions[0, 0, :]))
                    np.savetxt(os.path.join(self.all_args.data_dir,datestr,str(step)+'.txt'), st_with_at)
                    np.save(os.path.join(self.all_args.data_dir,datestr,str(step)+'.npy'), imgs[0])

                    # Obtain reward and next obs
                    obs, rewards, dones, infos, imgs = self.envs.depthStep(actions_student)
                else:
                    st_with_at = np.concatenate((obs['imu'][0, 0, :], actions[0, 0, :]))
                    np.savetxt(os.path.join(self.all_args.data_dir,datestr,str(step)+'.txt'), st_with_at)
                    np.save(os.path.join(self.all_args.data_dir,datestr,str(step)+'.npy'), imgs[0])
                    obs, rewards, dones, infos, imgs = self.envs.depthStep(actions)

                imgs_square = imgs.reshape(240, 320)
                imgs_square = cv2.resize(imgs_square, (112, 112))
                imgs_square = imgs_square.reshape(1, 1, 112, 112)

                data = obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic 
                
                # insert data into buffer
                self.insert(data)

            self.compute()
            
            # post process
            total_num_steps = (episode + 1) * self.episode_length * self.n_rollout_threads
            self.buffer.after_update()
            # log information
            if episode % self.all_args.save_interval == 0:
                if episode % self.all_args.save_interval == 0 and episode > 2 * self.all_args.save_interval:
                    print("- training...")
                    start = time.time()
                    # self.envs.inner.disconnectUnity()
                    self.vision_policy.make_dataset()
                    # lr_clip = np.clip(0.0002 / self.beta, 0.0002, 0.0005)
                    lr_clip = 0.0004
                    self.vision_policy.train_action(lr = lr_clip, epoch = self.all_args.ppo_epoch, bs = 6144)
                    self.vision_policy.save(save_dir=self.all_args.pretrain_dir[:-3]+str(episode)+".pt")
                    now = time.time()
                    time.sleep(max(15-(now-start), 1.0))
                    self.envs.inner.connectUnity()
                if episode % self.all_args.save_interval == 0:
                    print("- evaluating...")
                    self.beta = self.eval_once()
            print("==== episode", episode, "done ====")
            self.env_infos = defaultdict(list)

    @torch.no_grad()
    def eval_once(self):
        obs = self.eval_envs.reset()
        imgs = np.zeros((1, 1, 112, 112), dtype = np.float32)
        device_student = self.vision_policy.device
        progress = 0.0
        progress_list = []
        success_list = []
        for step in range(10 * self.episode_length):
            # Sample actions
            imu_input = np.zeros((1,9+3), dtype = np.float32)
            quat = obs["imu"][0, 0, 3:7]
            R = np.array(
            [[1.0 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[0] * quat[2] + quat[1] * quat[3])],
            [2 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]), 2 * (quat[2] * quat[3] - quat[0] * quat[1])],
            [2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), 1.0 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])]])
            imu_input[0, :9] = R.reshape(9)
            imu_input[0, 9:12] = obs["imu"][0, 0, 7:10]
            actions = self.vision_policy.net(depth = torch.tensor(imgs).to(device_student), imu = torch.tensor(imu_input).to(device_student))
            actions = actions.detach().cpu().numpy()
            obs, rewards, dones, infos, imgs = self.eval_envs.depthStep(actions.reshape(1, 1, 4))
            imgs = imgs.reshape(240, 320)
            imgs = cv2.resize(imgs, (112, 112))
            imgs = imgs.reshape(1, 1, 112, 112)
            if dones[0,0]:
                progress_list.append(progress)
                if rewards[0] > -1.0:
                    success_list.append(1)
                else:
                    success_list.append(0)
            else: 
                progress = self.eval_envs.progress[0] / self.eval_envs.total_len
        # cal beta
        beta_progress = max(0.0, sum(progress_list) / len(progress_list))
        beta_success = max(0.0, sum(success_list) / len(success_list))
        beta = 0.5*beta_progress + 0.5*beta_success
        beta_new = self.beta * 0.4 + beta * 0.6
        print("average progress: ", beta_progress)
        print("average success rate: ", beta_success)
        print("beta: ", beta_new)
        if self.all_args.use_wandb:
            wandb.log({"average_progress": beta_progress, "average_successrate": beta_success, "beta": beta_new})
        return beta_new
    
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
