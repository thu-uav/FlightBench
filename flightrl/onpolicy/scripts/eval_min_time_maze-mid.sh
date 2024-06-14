#! /usr/bin/bash

##### use this for training

python ./eval_min_time.py \
    --n_rollout_threads 1 \
    --n_training_threads 8 \
    --num_mini_batch 1 \
    --algorithm_name mappo \
    --episode_length 170 \
    --stage_two_episode_length 170 \
    --env_name LearningMinTime \
    --experiment_name maze-mid \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 75000000 \
    --max_grad_norm 8.0 \
    --ppo_epoch 10 \
    --seed 1 \
    --device cuda:0 \
    --scene_path scene/maze-mid \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.4 \
    --user_name ysa \
    --wandb_name yushuang20010911 \
    --gate_num 1 \
    --gate0_x 7.0 \
    --gate0_y -12.0 \
    --gate0_z 2.5 \
    --gate0_angle -60 \
    --use_wandb \
    --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/runs/LearningMinTime/mappo/maze-mid/wandb/run-20240403_220902-kdcunqie/files