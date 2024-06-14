#! /usr/bin/bash

##### use this for training

python ./train_min_time.py \
    --n_rollout_threads 128 \
    --n_training_threads 8 \
    --num_mini_batch 1 \
    --algorithm_name mappo \
    --episode_length 650 \
    --stage_two_episode_length 138 \
    --env_name LearningMinTime \
    --experiment_name maze-high \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 55000000 \
    --max_grad_norm 8.0 \
    --ppo_epoch 10 \
    --seed 1 \
    --device cuda:0 \
    --scene_path scene/maze-high \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.4 \
    --user_name ysa \
    --wandb_name yushuang20010911 \
    --gate_num 2 \
    --gate0_x -0.7 \
    --gate0_y 2.5 \
    --gate0_z 2.5 \
    --gate0_angle 70 \
    --gate1_x 10.5 \
    --gate1_y 5.4 \
    --gate1_z 2.5 \
    --gate1_angle 0