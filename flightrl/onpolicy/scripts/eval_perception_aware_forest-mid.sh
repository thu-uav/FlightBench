#! /usr/bin/bash

##### use this for training

python ./eval_perception_aware.py \
    --n_rollout_threads 1 \
    --n_training_threads 8 \
    --num_mini_batch 1 \
    --algorithm_name mappo \
    --episode_length 155 \
    --stage_two_episode_length 155 \
    --env_name LearningPA \
    --experiment_name forest-mid-pa \
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
    --scene_path scene/forest-mid \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.4 \
    --user_name ysa \
    --wandb_name yushuang20010911 \
    --gate_num 1 \
    --gate0_x 10.0 \
    --gate0_y -12.5 \
    --gate0_z 2.0 \
    --gate0_angle -51.3 \
    --use_wandb \
    --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/runs/LearningPA/mappo/forest-mid-pa/wandb/run-20240411_004827-07z0149c/files