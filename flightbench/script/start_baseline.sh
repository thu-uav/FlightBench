#!/bin/bash
quad_name="air"
echo -e "\nstarting baselines..."

if [ $# != 2 ];
then echo -e "\033[31mPlease use: './start_baseline <test_base_num> <baseline_name>' to start baselines\033[0m"
exit -1
fi

rostopic pub /"${quad_name}"/bridge/arm std_msgs/Bool "data: true" -1;
rostopic pub /"${quad_name}"/autopilot/start std_msgs/Empty "{}" -1;

sleep 2

case $2 in
    "sb_min_time") echo "sb_min_time, test case $1"
    python3 fly_path.py ../../bench_sb_min_time/path_dense.csv
    ;;
    "ego_planner") echo "ego_planner, test case $1"
    case $1 in
        0) roslaunch ego_planner run_in_sim.launch \
        point_num:=1 \
        point0_x:=14.00 \
        point0_y:=14.00 \
        point0_z:=2.00
        ;;
        1) roslaunch ego_planner run_in_sim.launch \
        point_num:=1 \
        point0_x:=10.00 \
        point0_y:=-12.50 \
        point0_z:=2.00 \
        ;;
        2) roslaunch ego_planner run_in_sim.launch \
        point_num:=1 \
        point0_x:=14.0 \
        point0_y:=-14.0 \
        point0_z:=2.00 \
        ;;
        3) roslaunch ego_planner run_in_sim.launch \
        point_num:=1 \
        point0_x:=-4.2 \
        point0_y:=-9.5 \
        point0_z:=2.50 \
        ;;
        4) roslaunch ego_planner run_in_sim.launch \
        point_num:=2 \
        point0_x:=0.358819 \
        point0_y:=1.20 \
        point0_z:=2.50 \
        point1_x:=7.0 \
        point1_y:=-12.0 \
        point1_z:=2.50 \
        ;;
        5) roslaunch ego_planner run_in_sim.launch \
        point_num:=2 \
        point0_x:=-0.5 \
        point0_y:=1.5 \
        point0_z:=2.50 \
        point1_x:=10.5 \
        point1_y:=5.4 \
        point1_z:=2.50 \
        ;;
        6) roslaunch ego_planner run_in_sim.launch \
        point_num:=2 \
        point0_x:=-1.0 \
        point0_y:=4.7 \
        point0_z:=2.0 \
        point1_x:=-6.0 \
        point1_y:=-6.0 \
        point1_z:=4.00 \
        ;;
        7) roslaunch ego_planner run_in_sim.launch \
        point_num:=4 \
        point0_x:=7.0 \
        point0_y:=-6 \
        point0_z:=2.00 \
        point1_x:=7.0 \
        point1_y:=1.71 \
        point1_z:=2.00 \
        point2_x:=-1.0 \
        point2_y:=4.7 \
        point2_z:=2.00 \
        point3_x:=-6.0 \
        point3_y:=-6.0 \
        point3_z:=1.50 \
        ;;
        *) echo -e "\033[31mInvalid test_case_num\033[0m"
    esac
    ;;
    "fast_planner") echo "fast_planner, test case $1"
    case $1 in
        0) roslaunch plan_manage kino_replan.launch \
        point_num:=1 \
        point0_x:=14.00 \
        point0_y:=14.00 \
        point0_z:=2.00
        ;;
        1) roslaunch plan_manage kino_replan.launch \
        point_num:=1 \
        point0_x:=10.00 \
        point0_y:=-12.50 \
        point0_z:=2.00 \
        ;;
        2) roslaunch plan_manage kino_replan.launch \
        point_num:=1 \
        point0_x:=14.0 \
        point0_y:=-14.0 \
        point0_z:=2.00 \
        ;;
        3) roslaunch plan_manage kino_replan.launch \
        point_num:=1 \
        point0_x:=-4.2 \
        point0_y:=-10.5 \
        point0_z:=2.50 \
        ;;
        4) roslaunch plan_manage kino_replan.launch \
        point_num:=2 \
        point0_x:=0.358819 \
        point0_y:=1.2 \
        point0_z:=2.50 \
        point1_x:=7.0 \
        point1_y:=-12.0 \
        point1_z:=2.50 \
        ;;
        5) roslaunch plan_manage kino_replan.launch \
        point_num:=2 \
        point0_x:=-0.5 \
        point0_y:=1.5 \
        point0_z:=2.50 \
        point1_x:=10.5 \
        point1_y:=5.4 \
        point1_z:=2.50 \
        ;;
        6) roslaunch plan_manage kino_replan.launch \
        point_num:=2 \
        point0_x:=-1.0 \
        point0_y:=4.7 \
        point0_z:=2.0 \
        point1_x:=-6.0 \
        point1_y:=-6.0 \
        point1_z:=4.00 \
        ;;
        7) roslaunch plan_manage kino_replan.launch \
        point_num:=4 \
        point0_x:=7.0 \
        point0_y:=-6 \
        point0_z:=2.00 \
        point1_x:=7.0 \
        point1_y:=1.71 \
        point1_z:=2.00 \
        point2_x:=-1.0 \
        point2_y:=4.7 \
        point2_z:=2.00 \
        point3_x:=-6.0 \
        point3_y:=-6.0 \
        point3_z:=1.50 \
        ;;
        *) echo -e "\033[31mInvalid test_case_num\033[0m"
    esac
    ;;
    "tgk_planner") echo "tgk_planner, test case $1"
    case $1 in
        0) roslaunch state_machine planning.launch \
        point_num:=1 \
        point0_x:=14.00 \
        point0_y:=14.00 \
        point0_z:=2.00 \
        ;;
        1) roslaunch state_machine planning.launch \
        point_num:=1 \
        point0_x:=10.00 \
        point0_y:=-12.50 \
        point0_z:=2.00 \
        ;;
        2) roslaunch state_machine planning.launch \
        point_num:=1 \
        point0_x:=14.0 \
        point0_y:=-14.0 \
        point0_z:=2.00 \
        ;;
        3) roslaunch state_machine planning.launch \
        point_num:=1 \
        point0_x:=-4.2 \
        point0_y:=-10.5 \
        point0_z:=2.50 \
        ;;
        4) roslaunch state_machine planning.launch \
        point_num:=2 \
        point0_x:=0.358819 \
        point0_y:=1.2 \
        point0_z:=2.50 \
        point1_x:=7.0 \
        point1_y:=-12.0 \
        point1_z:=2.50 \
        ;;
        5) roslaunch state_machine planning.launch \
        point_num:=2 \
        point0_x:=-0.4 \
        point0_y:=1.5 \
        point0_z:=2.50 \
        point1_x:=10.5 \
        point1_y:=5.4 \
        point1_z:=2.50 \
        ;;
        6) roslaunch state_machine planning.launch \
        point_num:=2 \
        point0_x:=-1.0 \
        point0_y:=4.7 \
        point0_z:=2.0 \
        point1_x:=-6.0 \
        point1_y:=-6.0 \
        point1_z:=4.00 \
        ;;
        7) roslaunch state_machine planning.launch \
        point_num:=4 \
        point0_x:=7.0 \
        point0_y:=-6 \
        point0_z:=2.00 \
        point1_x:=7.0 \
        point1_y:=1.71 \
        point1_z:=2.00 \
        point2_x:=-1.0 \
        point2_y:=4.7 \
        point2_z:=2.00 \
        point3_x:=-6.0 \
        point3_y:=-6.0 \
        point3_z:=1.50 \
        ;;
        *) echo -e "\033[31mInvalid test_case_num\033[0m"
    esac
    ;;
    "learning_min_time") echo "learning_min_time, test case $1"
    case $1 in
        0) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 8 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 170 \
        --stage_two_episode_length 120 \
        --env_name LearningMinTime \
        --experiment_name forest-low \
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
        --scene_path scene/forest-low \
        --vec_env_config flightlib/configs/vec_env.yaml \
        --env_config flightlib/configs/quadrotor_env.yaml \
        --dyn_config flightlib/configs/dyn_param.yaml \
        --gate_radius 0.6 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --gate_num 1 \
        --gate0_x 14.0 \
        --gate0_y 14.0 \
        --gate0_z 2.0 \
        --gate0_angle 45 \
        --use_wandb \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240410_123353-b4oi8qr2/files
        ;;
        1) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 8 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 150 \
        --stage_two_episode_length 120 \
        --env_name LearningMinTime \
        --experiment_name forest-mid \
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
        --gate_radius 0.6 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --gate_num 1 \
        --gate0_x 10.0 \
        --gate0_y -12.5 \
        --gate0_z 2.0 \
        --gate0_angle -51.3 \
        --use_wandb \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240407_204402-ukhyxglr/files
        ;;
        2) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 8 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 170 \
        --stage_two_episode_length 170 \
        --env_name LearningMinTime \
        --experiment_name forest-high \
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
        --scene_path scene/forest-high \
        --vec_env_config flightlib/configs/vec_env.yaml \
        --env_config flightlib/configs/quadrotor_env.yaml \
        --dyn_config flightlib/configs/dyn_param.yaml \
        --gate_radius 0.6 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --gate_num 1 \
        --gate0_x 14.0 \
        --gate0_y -14.0 \
        --gate0_z 2.0 \
        --gate0_angle -45.0 \
        --use_wandb \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240409_152736-ooxxuyva/files
        ;;
        3) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 1 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 160 \
        --stage_two_episode_length 138 \
        --env_name LearningMinTime \
        --experiment_name maze-low \
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
        --scene_path scene/maze-low \
        --vec_env_config flightlib/configs/vec_env.yaml \
        --env_config flightlib/configs/quadrotor_env.yaml \
        --dyn_config flightlib/configs/dyn_param.yaml \
        --gate_radius 0.6 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --gate_num 1 \
        --gate0_x -4.2 \
        --gate0_y -10.5 \
        --gate0_z 2.5 \
        --gate0_angle -90 \
        --use_wandb \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240403_220247-waqtavuv/files
        ;;
        4) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 8 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 190 \
        --stage_two_episode_length 190 \
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
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240403_220902-kdcunqie/files
        ;;
        5)
        python ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 8 \
        --num_mini_batch 1 \
        --algorithm_name mappo \
        --episode_length 170 \
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
        --gate_radius 0.6 \
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
        --gate1_angle 0 \
        --use_wandb \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240409_153643-jh6hhs3w/files
        ;;
        6)
        python ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 1 \
        --num_mini_batch 32 \
        --algorithm_name mappo \
        --episode_length 110 \
        --env_name LearningMinTime \
        --experiment_name eval \
        --use_recurrent_policy \
        --mlp_hidden_size 256 \
        --layer_N 1 \
        --log_interval 2 \
        --save_interval 2 \
        --num_env_steps 40960000 \
        --ppo_epoch 10 \
        --seed 1 \
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
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240404_204954-45flr3il/files
        ;;
        7) python3 ../../flightrl/onpolicy/scripts/eval_min_time_ros.py \
        --n_rollout_threads 1 \
        --n_training_threads 1 \
        --num_mini_batch 32 \
        --algorithm_name mappo \
        --episode_length 185 \
        --env_name LearningMinTime \
        --experiment_name eval \
        --use_recurrent_policy \
        --mlp_hidden_size 256 \
        --layer_N 1 \
        --log_interval 2 \
        --save_interval 2 \
        --num_env_steps 40960000 \
        --ppo_epoch 10 \
        --seed 1 \
        --device cuda:0 \
        --scene_path scene/racing-mid \
        --vec_env_config flightlib/configs/vec_env.yaml \
        --env_config flightlib/configs/quadrotor_env.yaml \
        --dyn_config flightlib/configs/dyn_param.yaml \
        --gate_radius 0.6 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --gate_num 4 \
        --gate0_x 7.0 \
        --gate0_y -6.0 \
        --gate0_z 2.0 \
        --gate0_angle 90 \
        --gate1_x 7.0 \
        --gate1_y 1.71 \
        --gate1_z 2.0 \
        --gate1_angle 90 \
        --gate2_x -1.0 \
        --gate2_y 4.7 \
        --gate2_z 2.0 \
        --gate2_angle 180 \
        --gate3_x -6.0 \
        --gate3_y -6.0 \
        --gate3_z 1.5 \
        --gate3_angle -90 \
        --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/run-20240410_195400-s3ibu40k/files
        # --model_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lmt_policy/racing-mid-wl/run-20240507_150554-86vvbma2/files
        ;;
        *) echo -e "\033[31mInvalid test_case_num\033[0m"
        ;;
        esac
    ;;
    "agile_autonomy") echo "agile_autonomy, test case $1"
    case $1 in
        0) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case0_settings.yaml
        ;;
        1) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case1_settings.yaml
        ;;
        2) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case2_settings.yaml
        ;;
        3) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case3_settings.yaml
        ;;
        4) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case4_settings.yaml
        ;;
        5) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case5_settings.yaml
        ;;
        6) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case6_settings.yaml
        ;;
        7) cd ../../../benchmark_agile_autonomy/planner_learning
        python test_trajectories.py --settings_file=config/case7_settings.yaml
        ;;
    esac
    ;;
    "learning_pa") echo "learning_perception_aware, test case $1"
    case $1 in
        0) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lpa_policy/forest-low-student/PAnet2060.pt
        ;;
        1) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lpa_policy/forest-mid-student/PAnet2220.pt
        ;;
        3) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/runner/PAnet1880.pt
        ;;
        4) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/runner/PAnet1120.pt
        ;;
        5) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightbench/lpa_policy/maze-high-student/PAnet2140.pt
        ;;
        6) python ../../flightrl/onpolicy/scripts/eval_perception_aware_vision_ros.py \
        --algorithm_name mappo \
        --env_name LearningPA \
        --experiment_name eval \
        --seed 1 \
        --device cuda:0 \
        --user_name ysa \
        --wandb_name yushuang20010911 \
        --use_wandb \
        --pretrain_dir /home/ysa/workspace/flightmare_ws/src/flightmare/flightrl/onpolicy/runner/PAnet2960.pt \
        ;;
    esac
    ;;
    *) echo -e "\033[31mInvalid baseline_name\033[0m"
    ;;
esac
