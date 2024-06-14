import yaml
from onpolicy.config import get_config
from gym import Env, spaces
import os
import sys
import time
import numpy as np
import torch
from flightgym import QuadrotorEnv_v2
import collections
import time
import os
import random

class  BaseVecEnv(Env):
    def __init__(self, all_args):
        super(BaseVecEnv, self).__init__()
        self.args = all_args
        cfg = self.create_env_yaml()
        with open(os.path.join(os.environ["FLIGHTMARE_PATH"], all_args.vec_env_config), 'w', encoding='utf-8') as f:
            yaml.dump(cfg, f)
        self.inner = QuadrotorEnv_v2(os.path.join(os.environ["FLIGHTMARE_PATH"], all_args.vec_env_config), os.path.join(os.environ["FLIGHTMARE_PATH"], all_args.env_config), os.path.join(os.environ["FLIGHTMARE_PATH"], all_args.dyn_config))
        if self.args.use_unity:
            self.inner.connectUnity()
        self.num_envs = all_args.n_rollout_threads
        self.inner_obs_dim = self.inner.getObsDim()
        self.n_imu = 13 # pos, quat(wxyz), vel, omega
        obs_space_dict = {
            'imu': spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                              shape=(self.args.state_len * self.n_imu, ),
                              dtype="float32"),
        }

        self.observation_space = [spaces.Dict(obs_space_dict)]
        self.share_observation_space = [spaces.Dict(obs_space_dict)]

        self.action_space = [spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                                    shape=(4,),
                                    dtype="float32")]
        
        self.reward_type = ("total")

        # about inner
        self._observation = np.zeros([self.num_envs, self.inner_obs_dim],
                                     dtype=np.float32)
        self._done = np.zeros((self.num_envs), dtype=bool)
        self._extraInfoNames = self.inner.getExtraInfoNames()
        self._extraInfo = np.zeros([self.num_envs,
                                    len(self._extraInfoNames)], dtype=np.float32)
        self.rewards = {}
        for key in self.reward_type:
            self.rewards[key] = [[] for _ in range(self.num_envs)]

        # about obs queue
        self.state_queue = collections.deque([], maxlen=self.args.max_input_seq_len)


    def seed(self, seed):
        random.seed(seed)
        np.random.seed(seed)
        torch.random.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)
        self.inner.setSeed(seed)

    def reset(self, random = False):
        # reset values
        # self._reward = np.zeros(self.num_envs, dtype=np.float32)
        self.reset_values(random = random)
        self.rewards = {}
        for key in self.reward_type:
            self.rewards[key] = [[] for _ in range(self.num_envs)]
        print("reset")

        self._observation = self.random_start_pos().astype(np.float32) # to be defined

        self.inner.reset(self._observation)
        self.update_queue(self._observation, if_reset=True)
        obs = self.get_obs()
        print("reset done")
        return obs

    def reset_values(self):
        """
        reset vars to init state.
        """
        self.update_queue(if_reset=True)

    def step(self):
        """
        Step function
        """
        pass
    
    def create_env_yaml(self):
        cfg_env = {"env":{}}
        cfg_env["env"]["num_envs"] = self.args.n_rollout_threads
        cfg_env["env"]["num_threads"] = min(16, self.args.n_rollout_threads)

        if self.args.use_unity:
            cfg_env["env"]["render"] = "yes"
        else:
            cfg_env["env"]["render"] = "no"
        cfg_env["env"]["seed"] = self.args.seed
        cfg_env["env"]["scene_id"] = self.args.scene_id
        return cfg_env

    def update_queue(self, state, if_reset = False):
        # make state
        """
        update obs queue. Update for more observations.
        """

        if if_reset:
            self.state_queue.clear()
        self.state_queue.append(state.copy())

    def get_obs(self): #{'imu': np.array of (num_env, num_agent, n_imu), 'pcd': np.array of (num_env, num_agent, len_pcd), '...': ...}
        """
        Modify for different observation form.
        """
        obs = {}
        imu = np.zeros((self.num_envs, 1, self.args.state_len * self.n_imu), dtype = np.float32)
        for i in range(self.num_envs):
            each_env_imu_list = []
            for j in range(self.args.state_len):
                each_env_imu_list.append(self.state_queue[-j-1][i])
            imu[i, :, :] = np.array([np.concatenate(each_env_imu_list)])
        obs["imu"] = imu
        return obs
    
    def random_start_pos(self):
        """
        Method to sample random start pos follow certain distribution
        return start_pos: np.array of (num_env, n_imu)
        """
        pass

    def check_task_progress(self):
        """
        Method to calculate task progress, set flags and return dones
        return done: np.array of (num_env, ) dtype = bool
        """
        pass

    def cal_reward(self):
        """
        Method to calculate reward
        return reward: a dict with each type of reward
        """
    
    def close(self):
        """
        close
        """
        pass

def parse_args(args, parser):
    parser.add_argument("--max_input_seq_len", type=int, default=3)
    parser.add_argument("--state_len", type=int, default=1)
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--scene_id", type=int, default=0)
    parser.add_argument("--vec_env_config", type=str, default="flightlib/configs/vec_env.yaml")
    parser.add_argument("--env_config", type=str, default="flightlib/configs/quadrotor_env.yaml")
    parser.add_argument("--dyn_config", type=str, default="flightlib/configs/dyn_param.yaml")
    all_args = parser.parse_known_args(args)[0]
    return all_args

def main(args): # main
    parser = get_config()
    all_args = parse_args(args, parser)
    # path = os.environ["FLIGHTMARE_PATH"] +"/flightrl/air_planner/configs/train_config.yaml"
    # config = create_settings(path = path, mode =  "train" if all_args.train else "test")
    # config.if_random_start_pos = all_args.if_random_start_pos

    env = BaseVecEnv(all_args)
    obs = env.reset()
    t = 0
    start = time.time()
    while(t<50):
        action = np.zeros((all_args.n_rollout_threads, 4))
        action[0][0] = -0.25
        action[0][2] = 0.5
        action = np.stack([action.astype(np.float32) for _ in range(all_args.n_rollout_threads)])
        obs, rewards, dones, infos = env.step(action)
        # print("state:", obs[0]["imu"][0])
        # print("len obs: ", len(obs), "obs imu shape: ", obs[0]["imu"].shape, "obs pcd shape: ", obs[0]["pcd"].shape)
        # print("rewards: \n", rewards)
        # print("dones: ", dones)
        # print("infos: ", infos)
        t+=1
    print("time: ", time.time()-start)

if __name__ == "__main__":
    main(sys.argv[1:])
