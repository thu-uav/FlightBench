#!/usr/bin/env python
# python standard libraries
import os
from pathlib import Path
import sys
import socket

# third-party packages
import numpy as np
import setproctitle
import torch
import wandb
from pathlib import Path
import matplotlib.pyplot as plt
import cv2

# code repository sub-packages
from onpolicy.config import get_config
from onpolicy.envs.learning_perception_aware.perception_aware_vec_env import PAVecEnv
from onpolicy.runner.student_trainer import Trainer

def make_eval_env(all_args):
    if all_args.env_name == "LearningPA" and all_args.n_rollout_threads == 1:
        return PAVecEnv(all_args)
    else:
        print("Only Support LearningMinTime envirionment and n_rollout_threads = 1 for eval")
        exit(-1)


def parse_args(args, parser):
    parser.add_argument("--max_input_seq_len", type=int, default=3)
    parser.add_argument("--state_len", type=int, default=1)
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--scene_id", type=int, default=3)
    parser.add_argument("--use_unity", action='store_true', default=False)
    parser.add_argument("--scene_path", type=str, default="scene/office")
    parser.add_argument("--vec_env_config", type=str, default="flightlib/configs/vec_env.yaml")
    parser.add_argument("--env_config", type=str, default="flightlib/configs/quadrotor_env.yaml")
    parser.add_argument("--dyn_config", type=str, default="flightlib/configs/dyn_param.yaml")
    parser.add_argument("--gate_num", type=int, default=1)
    parser.add_argument("--gate0_x", type=float, default=7.6)
    parser.add_argument("--gate0_y", type=float, default=1.8)
    parser.add_argument("--gate0_z", type=float, default=2.5)
    parser.add_argument("--gate0_angle", type=float, default=45.0)
    parser.add_argument("--gate1_x", type=float, default=7.6)
    parser.add_argument("--gate1_y", type=float, default=1.8)
    parser.add_argument("--gate1_z", type=float, default=2.5)
    parser.add_argument("--gate1_angle", type=float, default=45.0)
    parser.add_argument("--gate2_x", type=float, default=7.6)
    parser.add_argument("--gate2_y", type=float, default=1.8)
    parser.add_argument("--gate2_z", type=float, default=2.5)
    parser.add_argument("--gate2_angle", type=float, default=45.0)
    parser.add_argument("--gate3_x", type=float, default=7.6)
    parser.add_argument("--gate3_y", type=float, default=1.8)
    parser.add_argument("--gate3_z", type=float, default=2.5)
    parser.add_argument("--gate3_angle", type=float, default=45.0)
    parser.add_argument("--gate4_x", type=float, default=7.6)
    parser.add_argument("--gate4_y", type=float, default=1.8)
    parser.add_argument("--gate4_z", type=float, default=2.5)
    parser.add_argument("--gate4_angle", type=float, default=45.0)
    parser.add_argument("--gate5_x", type=float, default=7.6)
    parser.add_argument("--gate5_y", type=float, default=1.8)
    parser.add_argument("--gate5_z", type=float, default=2.5)
    parser.add_argument("--gate5_angle", type=float, default=45.0)
    parser.add_argument("--gate_radius", type=float, default=0.5)

    parser.add_argument("--use_resnet",  action='store_true', default=False)
    parser.add_argument("--use_original_size",  action='store_true', default=False)
    parser.add_argument("--pretrained_global_resnet", type=str, default="")
    parser.add_argument("--stage_two_episode_length", type=int, default=100)
    parser.add_argument("--use_collect",  action='store_true', default=False)
    parser.add_argument("--pretrain_dir",  type=str, default="")
    all_args = parser.parse_known_args(args)[0]

    return all_args


def main(args):
    parser = get_config()
    all_args = parse_args(args, parser)

    if all_args.algorithm_name == "rmappo" or all_args.algorithm_name == "rmappg":
        assert (all_args.use_recurrent_policy or all_args.use_naive_recurrent_policy), ("check recurrent policy!")
    elif all_args.algorithm_name == "mappo" or all_args.algorithm_name == "mappg":
        assert (all_args.use_recurrent_policy == False and all_args.use_naive_recurrent_policy == False), ("check recurrent policy!")
    else:
        raise NotImplementedError

    # cuda
    if all_args.cuda and torch.cuda.is_available():
        print("choose to use gpu...")
        device = torch.device(all_args.device)
        torch.set_num_threads(all_args.n_training_threads)
        if all_args.cuda_deterministic:
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
    else:
        print("choose to use cpu...")
        device = torch.device("cpu")
        torch.set_num_threads(all_args.n_training_threads)

    # run dir
    run_dir = Path(os.path.join(os.environ["FLIGHTMARE_PATH"], "flightrl", "evals", all_args.env_name , all_args.algorithm_name , all_args.experiment_name))
    if not run_dir.exists():
        os.makedirs(str(run_dir))

    # wandb
    if all_args.use_wandb:
        run = wandb.init(config=all_args,
                         project="learning_pa",
                         entity=all_args.wandb_name,
                         notes=socket.gethostname(),
                         name="-".join([
                            all_args.algorithm_name,
                            all_args.experiment_name,
                            "rollout" + str(all_args.n_rollout_threads),
                            "minibatch" + str(all_args.num_mini_batch),
                            "epoch" + str(all_args.ppo_epoch),
                            "seed" + str(all_args.seed)
                         ]),
                         group=all_args.experiment_name,
                         dir=str(run_dir),
                         job_type="training",
                         reinit=True)
    else:
        if not run_dir.exists():
            curr_run = 'run1'
        else:
            exst_run_nums = [int(str(folder.name).split('run')[1]) for folder in run_dir.iterdir() if str(folder.name).startswith('run')]
            if len(exst_run_nums) == 0:
                curr_run = 'run1'
            else:
                curr_run = 'run%i' % (max(exst_run_nums) + 1)
        run_dir = run_dir / curr_run
        if not run_dir.exists():
            os.makedirs(str(run_dir))

    setproctitle.setproctitle("-".join([
        all_args.env_name, 
        all_args.algorithm_name, 
        all_args.experiment_name
    ]) + "@" + all_args.user_name)
    
    # seed
    torch.manual_seed(all_args.seed)
    torch.cuda.manual_seed_all(all_args.seed)
    np.random.seed(all_args.seed)

    # env init
    eval_envs = make_eval_env(all_args)
    eval_envs.seed(all_args.seed)
    num_agents = 1
    envs = eval_envs
    
    print("model path: ", all_args.pretrain_dir)
    vision_policy = Trainer(data_dir="empty", eval_dir="empty", pretrain_dir=all_args.pretrain_dir)

    # start testing
    obs = eval_envs.reset()
    imgs = np.zeros((1, 1, 112, 112), dtype = np.float32)

    key_steps = []
    obs_ori_vector = []
    state = []

    print("start testing")
    for step in range(all_args.episode_length):
        # Sample actions
        imu_input = np.zeros((1,9+3), dtype = np.float32)
        quat = obs["imu"][0, 0, 3:7]
        R = np.array(
        [[1.0 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]), 2 * (quat[1] * quat[2] - quat[0] * quat[3]), 2 * (quat[0] * quat[2] + quat[1] * quat[3])],
         [2 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]), 2 * (quat[2] * quat[3] - quat[0] * quat[1])],
         [2 * (quat[1] * quat[3] - quat[0] * quat[2]), 2 * (quat[2] * quat[3] + quat[0] * quat[1]), 1.0 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])]])
        imu_input[0, :9] = R.reshape(9)
        imu_input[0, 9:12] = obs["imu"][0, 0, 7:10]
        actions = vision_policy.net(depth = torch.tensor(imgs).cuda(), imu = torch.tensor(imu_input).cuda())
        actions = actions.detach().cpu().numpy()
        obs, rewards, dones, infos, imgs = eval_envs.depthStep(actions.reshape(1, 1, 4))
        imgs = imgs.reshape(240, 320)
        imgs = cv2.resize(imgs, (112, 112))
        imgs = imgs.reshape(1, 1, 112, 112)
        state.append(obs['imu'][0, 0, :3])

        # Obtain reward and next obs
        # actions = np.zeros([1,1,4])
        # actions[0,0,0] = 0
        # if step > 30:
        #     actions[0,0,2] = 0.6
        # else:
        #     actions[0,0,3] = 0.2
    
    state = np.stack(state)
    fig = plt.figure(dpi = 150)
    ax = fig.add_subplot(111, projection='3d')

    # ax.set_xlim(-12, 8)
    # ax.set_ylim(0, 4.5)
    ax.set_zlim(0, 4.5)

    ax.scatter(state[:, 0], state[:, 1], state[:, 2], c = 'b', s = 0.5, label = 'flight')
    for g in range(len(eval_envs.guide_path)):
        for p in range(eval_envs.guide_path[g].shape[0]):
            if g == 0 and p == 0:
                ax.plot([eval_envs.guide_path[g][p, 0], eval_envs.guide_path[g][p, 3]], [eval_envs.guide_path[g][p, 1], eval_envs.guide_path[g][p, 4]], [eval_envs.guide_path[g][p, 2], eval_envs.guide_path[g][p, 5]], c = 'r', label = "guide")
            else:
                ax.plot([eval_envs.guide_path[g][p, 0], eval_envs.guide_path[g][p, 3]], [eval_envs.guide_path[g][p, 1], eval_envs.guide_path[g][p, 4]], [eval_envs.guide_path[g][p, 2], eval_envs.guide_path[g][p, 5]], c = 'r')

    plt.savefig("traj.png")

    plt.figure()
    plt.plot(state[:, 2], label='z')
    plt.legend()
    plt.savefig('z-t.png')

    plt.figure()
    plt.scatter(state[:, 0], state[:, 1], c = 'b', s = 0.5, label = 'traj')
    for g in range(len(eval_envs.guide_path)):
        for p in range(eval_envs.guide_path[g].shape[0]):
            plt.plot([eval_envs.guide_path[g][p, 0], eval_envs.guide_path[g][p, 3]], [eval_envs.guide_path[g][p, 1], eval_envs.guide_path[g][p, 4]], c = 'r')
    plt.legend()
    plt.savefig('traj2D.png')
    
    # post process
    envs.close()
    if all_args.use_eval and eval_envs is not envs:
        eval_envs.close()

    if all_args.use_wandb:
        run.finish()


if __name__ == "__main__":
    main(sys.argv[1:])
