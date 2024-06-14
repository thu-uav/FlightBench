#! /usr/bin/bash

##### use this for training

python ./eval_perception_aware.py \
    --n_rollout_threads 1 \
    --n_training_threads 1 \
    --num_mini_batch 32 \
    --algorithm_name mappo \
    --episode_length 170 \
    --env_name LearningPA \
    --experiment_name eval \
    --use_recurrent_policy \
    --mlp_hidden_size 256 \
    --layer_N 1 \
    --log_interval 2 \
    --save_interval 2 \
    --num_env_steps 40960000 \
    --ppo_epoch 10 \
    --seed 3 \
    --device cuda:0 \
    --scene_path scene/maze-mid \
    --vec_env_config flightlib/configs/vec_env.yaml \
    --env_config flightlib/configs/quadrotor_env.yaml \
    --dyn_config flightlib/configs/dyn_param.yaml \
    --gate_radius 0.7 \
    --user_name ysa \
    --wandb_name yushuang20010911 \
    --use_wandb \
    --gate_num 1 \
    --gate0_x 7.0 \
    --gate0_y -12.0 \
    --gate0_z 2.5 \
    --gate0_angle -60 \
    --use_wandb \
    --use_unity \
    --use_collect \
    --scene_id 3 \
    --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lpa_policy/maze-mid-teacher/run-20240419_220908-qasm3maa/files
