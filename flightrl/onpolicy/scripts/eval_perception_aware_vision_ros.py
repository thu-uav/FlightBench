#!/usr/bin/env python
# python standard libraries
import os
from pathlib import Path
import sys
import socket
import rospy
import transforms3d
from quadrotor_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

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
from onpolicy.runner.student_trainer import Trainer


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

    # ros part 
    from onpolicy.runner.PA_ros_runner import PARosRunner as Runner

    print("model path: ", all_args.pretrain_dir)
    runner = Runner(model_dir=all_args.pretrain_dir)
    
    rospy.spin()
    
    # post process
    if all_args.use_wandb:
        run.finish()

if __name__ == "__main__":
    main(sys.argv[1:])
