#! /usr/bin/bash

##### use this for training

python ./eval_perception_aware.py \
    --n_rollout_threads 1 \
    --n_training_threads 1 \
    --num_mini_batch 32 \
    --algorithm_name mappo \
    --episode_length 330 \
    --env_name LearningPA \
    --experiment_name eval \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 40960000 \
    --ppo_epoch 10 \
    --seed 7 \
    --device cuda:0 \
    --scene_path scene/racing-low \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.6 \
    --user_name ysa \
    --wandb_name yushuang20010911 \
    --use_wandb \
    --gate_num 2 \
    --gate0_x -1.0 \
    --gate0_y 4.7 \
    --gate0_z 2.0 \
    --gate0_angle 180 \
    --gate1_x -6.0 \
    --gate1_y -6.0 \
    --gate1_z 4.0 \
    --gate1_angle -90 \
    --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/runs/LearningPA/mappo/racing_low_latency/wandb/run-20240319_224747-ab7zhlmy/files \
    --use_unity \
    --use_collect \
    --scene_id 4