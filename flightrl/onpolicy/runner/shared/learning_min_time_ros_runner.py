from collections import defaultdict, deque
from itertools import chain
import os
import time
import copy
import numpy as np
import torch
import wandb
import rospy
import transforms3d
from quadrotor_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from onpolicy.utils.util import update_linear_schedule
from onpolicy.runner.shared.base_runner import Runner

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def _t2n(x):
    return x.detach().cpu().numpy()

class LearningMinTimeRunner(Runner):
    def __init__(self, config):
        super(LearningMinTimeRunner, self).__init__(config)
        self.env_infos = defaultdict(list)
        # init ros node, subs and advertises
        rospy.init_node('learning_min_time', anonymous=True)
        self.odom_sub = rospy.Subscriber("/air/ground_truth/odometry", Odometry, self.odom_callback)
        self.command_pub = rospy.Publisher('/air/autopilot/control_command_input', ControlCommand, queue_size = 10)
        self.ros_odom = None
        self.planning_time_pub = rospy.Publisher('/planning_time', Float32, queue_size = 10)
        self.mapping_time_pub = rospy.Publisher('/mapping_time', Float32, queue_size = 10)
        # create a timer for collect and pub
        self.step = 0
        self.envs.reset()
        self.key_steps = []
        self.obs_cfp_vector = []
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/40.0), self.eval_ros_step)
        self.delay = 1.0
        self.count = 0

    def eval_ros_step(self, event=None):
        self.count += 1
        if self.count < self.delay * 40.0:
            return
        step = self.step
        if step < self.episode_length:

            # set flightlib state with ros state
            time_start = time.time_ns()
            odom = copy.deepcopy(self.ros_odom)
            obs_array = np.zeros(self.envs.n_imu, dtype = np.float32)
            obs_array[0] = odom.pose.pose.position.x
            obs_array[1] = odom.pose.pose.position.y
            obs_array[2] = odom.pose.pose.position.z
            obs_array[3] = odom.pose.pose.orientation.w
            obs_array[4] = odom.pose.pose.orientation.x
            obs_array[5] = odom.pose.pose.orientation.y
            obs_array[6] = odom.pose.pose.orientation.z
            # change into world frame
            mat = transforms3d.quaternions.quat2mat(obs_array[3:7])
            velo_body = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
            obs_array[7:10] = mat@velo_body
            obs_array[10] = odom.twist.twist.angular.x
            obs_array[11] = odom.twist.twist.angular.y
            obs_array[12] = odom.twist.twist.angular.z

            # print("odom new: ", obs_array[:3])

            obs, rewards, dones, infos = self.eval_envs.ros_get_obs(obs_array)

            time_end = time.time_ns()
            time_msg = Float32()
            time_msg.data = (time_end-time_start) / 1e6
            self.mapping_time_pub.publish(time_msg)

            # Sample actions
            for key in obs.keys():
                self.buffer.share_obs[key][step] = obs[key].copy()
                self.buffer.obs[key][step] = obs[key].copy()
            
            time_plan, values, actions, action_log_probs, rnn_states, rnn_states_critic = self.collect(step)

            if step % 20 == 0:
                self.key_steps.append(step)
                self.obs_cfp_vector.append(obs["cfp"][0,0,:])

            # do not take actions in flightlib env  but pub
                
            # actions = np.zeros([1,1,4])
            # actions[0,0,0] = 0
            # if step > 30:
            #     actions[0,0,2] = 0.6
            # else:
            #     actions[0,0,3] = 0.2
            
            cmd_msg = ControlCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.control_mode = cmd_msg.BODY_RATES
            cmd_msg.armed = True
            cmd_msg.expected_execution_time = rospy.Time(0.020)
            cmd_msg.collective_thrust = (np.tanh(actions[0, 0, 0])+1) * 20 / 1.8
            cmd_msg.bodyrates.x = np.tanh(actions[0, 0, 1]) * 7.5
            cmd_msg.bodyrates.y = np.tanh(actions[0, 0, 2]) * 7.5
            cmd_msg.bodyrates.z = np.tanh(actions[0, 0, 3]) * 0.5
            
            self.command_pub.publish(cmd_msg)
            # obs, rewards, dones, infos = self.envs.step(actions)
            time_msg = Float32()
            time_msg.data = time_plan
            self.planning_time_pub.publish(time_msg)

            data = obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic 
            
            # insert data into buffer
            self.insert(data)

        if step == self.episode_length:  
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
            plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 7], label = "vx")
            plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 8], label = "vy")
            plt.plot(time_axis, self.buffer.obs["imu"][:self.episode_length, 0, 0, 9], label = "vz")
            plt.plot(time_axis, self.buffer.masks[:self.episode_length, 0, 0, 0], label = "masks")
            plt.legend()
            plt.title("v-t")
            plt.savefig("v-t.png")

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
            plt.ylim(-8, 8)
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

            for i in range(len(self.key_steps)):
                step = self.key_steps[i]
                if i == 0:
                    ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], self.obs_cfp_vector[i][0], self.obs_cfp_vector[i][1], self.obs_cfp_vector[i][2], color = (0,0.85,0.46,0.8), label = "next_cfp")
                else:
                    ax.quiver(self.buffer.obs["imu"][step, 0, 0, 0], self.buffer.obs["imu"][step, 0, 0, 1], self.buffer.obs["imu"][step, 0, 0, 2], self.obs_cfp_vector[i][0], self.obs_cfp_vector[i][1], self.obs_cfp_vector[i][2], color = (0,0.85,0.46,0.7))
            # ax.quiver(0, 0,0, 3, 3,3,color=(0,1,0,0.5)) 
            ax.legend()
            plt.savefig("traj_cfp.png")

            plt.figure()
            plt.scatter(self.buffer.obs["imu"][:self.episode_length, 0, 0, 0],self.buffer.obs["imu"][:self.episode_length, 0, 0, 1], c = 'b', s = 0.5, label = 'traj')
            for g in range(len(self.envs.guide_path)):
                for p in range(self.envs.guide_path[g].shape[0]):
                    plt.plot([self.envs.guide_path[g][p, 0], self.envs.guide_path[g][p, 3]], [self.envs.guide_path[g][p, 1], self.envs.guide_path[g][p, 4]], c = 'r')
            plt.legend()
            plt.savefig('traj2D.png')

        self.step += 1
        
    def odom_callback(self, data):
        self.ros_odom = (data)
    
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

        time_start = time.time_ns()
        values, actions, action_log_probs, rnn_states, rnn_states_critic = self.trainer.policy.get_actions(
            obs_concate,
            share_obs_concate,
            np.concatenate(self.buffer.rnn_states[step]),
            np.concatenate(self.buffer.rnn_states_critic[step]),
            np.concatenate(self.buffer.masks[step])
        )
        time_end = time.time_ns()
        # [n_envs*n_agents, ...] -> [n_envs, n_agents, ...]
        values = np.array(np.split(_t2n(values), self.n_rollout_threads))
        actions = np.array(np.split(_t2n(actions), self.n_rollout_threads))
        action_log_probs = np.array(np.split(_t2n(action_log_probs), self.n_rollout_threads))
        rnn_states = np.array(np.split(_t2n(rnn_states), self.n_rollout_threads))
        rnn_states_critic = np.array(np.split(_t2n(rnn_states_critic), self.n_rollout_threads))
        time_plan = (time_end-time_start) / 1e6

        return time_plan, values, actions, action_log_probs, rnn_states, rnn_states_critic

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
