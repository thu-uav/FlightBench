# Welcome to FlightBench
*FlightBench* is an unified open-source benchmark for planning methods on ego-vision-based navigation for quadrotors built on [Flightmare](https://github.com/uzh-rpg/flightmare). *FlightBench* provides cusomizable test scenarios (including three quantitative task difficulty metrics), representative planning algorithms, and comprehensive evaluation metrics. Here's an overview of FlightBench.

![Overview of FlightBench](overview.jpg)
---

Our code is available [here](https://github.com/thu-uav/FlightBench). For details on the experimental setup and conclusions, please refer to our [paper](https://arxiv.org/abs/2406.05687).

# Table of Contents
1. [Introduction](#welcome-to-flightbench)
2. [Table of Contents](#table-of-contents)
3. [Installation](#installation)
4. [Benchmark Design](#benchmark-design)
5. [Let's Fly](#lets-fly)
6. [Train Own Policy](#train-own-policy)
7. [Citation](#citation)
8. [Acknowledgement](#acknowledgement)

# Installation
Before starting the installation, please add your ssh key to github.
## Environment
- The installation is tested on Ubuntu-20.04
- An Nvidia GPU with installed drivers is necessary for rendering and RL training. CUDA>=11.1 is also necessary.
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- [Gazebo 11](https://classic.gazebosim.org/)
- [Python 3.8](https://www.python.org/downloads/release/python-380/). We recommend using python virtual environment. Use `sudo apt install python3-venv` to install
- [gcc/g++ 9](https://gcc.gnu.org/releases.html)

## Install FlightBench
Install the benchmark platform by the following commands:

```bash
mkdir flightbench_ws && cd flightbench_ws
mkdir src && cd src
sudo apt-get update && sudo apt-get install git cmake python3 python3-dev python3-venv python3-rosbag
git clone git@github.com:thu-uav/FlightBench.git
cd FlightBench
git submodule update --init --recursive

# Add FLIGHTMARE_PATH environment variable to .bashrc file:
echo "export FLIGHTMARE_PATH=path/to/FlightBench" >> ~/.bashrc
source ~/.bashrc

# We recommend using python virtual env to manage python packages
python3 -m venv flightpy # or any other name you like
source flightpy/bin/activate # active the venv

# install packages for training using pip
cd $FLIGHTMARE_PATH/flightrl
sudo apt-get install build-essential cmake libzmqpp-dev libopencv-dev libeigen3-dev
pip install -r requirements.txt
cd $FLIGHTMARE_PATH/flightlib

# get external packages
mkdir build && cd build
cmake ..
cd ..
rm -rf build

# then, install flightgym
pip install .

# install MAPPO
cd $FLIGHTMARE_PATH/flightrl
pip install -e .
```

Then install ROS part.
```bash
sudo apt-get update
export ROS_DISTRO=noetic
sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool python3-empy ros-$ROS_DISTRO-mavros
sudo pip install catkin-tools

# go flightbench_ws 
cd flightbench_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release

catkin build
```

## Install Baselines
```bash
# import benchmarks
cd src
vcs-import < FlightBench/flightbench/benchmarks.yaml

# install deps
sudo apt-get install libarmadillo-dev libglm-dev
# instsll nlopt
# go flightbench_ws 
cd flightbench_ws
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
# install Open3d
cd flightbench_ws
git clone --recursive https://github.com/intel-isl/Open3D
cd Open3D
git checkout v0.9.0
git submodule update --init --recursive
./util/scripts/install-deps-ubuntu.sh
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local/bin/cmake ..
make -j$(nproc)
sudo make install

# compile baselines
cd flightbench_ws
catkin build
```

Install python packages for agile autonomy
```bash
# activate flightpy
source path/to/flightpy/bin/activate
cd path/to/benchmark_agile_autonomy
pip install -r requirements.txt
```

Install benchmark_sb_min_time
```bash
sudo apt-get install build-essential cmake pkg-config ccache zlib1g-dev libomp-dev libyaml-cpp-dev libhdf5-dev libgtest-dev liblz4-dev liblog4cxx-dev libeigen3-dev python3 python3-venv python3-dev python3-wheel python3-opengl
# into sb_min_time dir
cd benchmark_sb_min_time
git submodule update --init --recursive
make dependencies
make
```

# Benchmark Design
## Use Existing or Custom Scenarios
### Use our scenarios

We provide 3 distinct scenarios: forest, maze, and multi-waypoint. 

![scene](scene.jpg)

1. Download scene folder from [here](https://cloud.tsinghua.edu.cn/f/54d91a7afeeb4006b9a3/?dl=1) and put it to `path/to/FlightBench/scene` after unzip.
2. Download render file from [here](https://cloud.tsinghua.edu.cn/f/49674a52f55a451086bd/?dl=1). Put it into `path/to/FlightBench/flightrender` after unzip.

### Customize own scenarios
The Scenarios are customized using [Unity](https://unity.com/). Our unity project is forked from [flightmare_unity](https://github.com/uzh-rpg/flightmare_unity). You may choose to start with either [flightmare_unity](https://github.com/uzh-rpg/flightmare_unity) or [our project]() for modifications.

1. Install Unity hub following [offical document](https://docs.unity3d.com/hub/manual/InstallHub.html)
2. Add a project, unity hub will automatically prompt whether to install unity editor (2020.1.10f1)
3. Then you can edit and build the project following [this](https://flightmare.readthedocs.io/en/latest/building_flightmare_binary/standalone.html#)
4. Place the compiled files into `path/to/FlightBench/flightrender`.

Based on Unity, the **dynamic obstacles** are also supported by scripting. For instance, add the following scripts to your obeject to enable an oribit move.
```cs
﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class orbit : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;
    // Start is called before the first frame update
    void Start() {}
    // Update is called once per frame
    void Update()
    {
        Vector3 targetPos = target.position;
        Vector3 orbitPos = new Vector3(targetPos.x, transform.position.y, targetPos.z);
        Vector3 direction = (orbitPos - transform.position).normalized;
        float distanceToMove = speed * Time.deltaTime;
        Vector3 y = new Vector3(0, 1, 0);
        Vector3 newPosition = transform.position + Vector3.Cross(direction, y) * distanceToMove;
        transform.position = newPosition;
        transform.LookAt(targetPos);
    }
}
```

![orbit](orbit.gif)

For training and evaluating RL-based methods, a scene folder should be organized as follow:
```
scene
|-- <scene_name_0>
|   |-- env.ply
|   |-- env-surface.ply
|   |-- roadmap_shortened_unique_path0_0.csv
|   |-- roadmap_shortened_unique_path1_0.csv
|   |-- roadmap_shortened_unique_pathn_0.csv
|       
|-- <scene_name_1>
|   |-- env.ply
|   |-- env-surface.ply
|   |-- roadmap_shortened_unique_path0_0.csv
|   |-- roadmap_shortened_unique_path1_0.csv
|   |-- roadmap_shortened_unique_pathn_0.csv
|
|-- <scene_name_2>
|   ...
```
`env.ply` is the pointcloud file of the scenario, which can be generated by clicking the 'scene save pointcloud' bottom after compiling the unity project.

[This section (Evaluate task difficulty)](#evaluate-task-difficulty) shows the generation of `env-surface.ply` and `roadmap_shortened_unique_path`.

Then put the folder into `path/to/FlightBench/scene` to support RL training and evluating.

## Perception & control interface
### Perception data
RGBD image, odometry, and imu sensor data are provided. User can subscribe the following topics for flying.

- `/<quad_name>/ground_truth/odometry`: pose and velocity (both linear and angular) under body frame
- `/<quad_name>/ground_truth/imu`: imu data, containing angular velocity and linear acceleration under body frame
- `/<quad_name>/flight_pilot/rgb`: a rgb8 encoding `sensor_msgs.Image` message, containing the ego-vision rgb image.
- `/<quad_name>/flight_pilot/depth`: a 32fc1 encoding `sensor_msgs.Image` message, containing the ego-vision depth image.

Users can enable/disable the RGB and Depth topic separately by modifying `path/to/FlightBench/flightros/params/default.yaml`.

### Control interface
There are many ways to control a quadrotor. At the lowest level, users can control the quadrotor using rate or attitude command.

- Topics for rate or attitude: `/<quad_name>/autopilot/control_command_input`
- Msg for rate or attitude: `quadrotor_msgs/ControlCommand`
We provide a PD controller to track the desired rate or attitude. The parameters are available in `path/to/FlightBench/dep/quadrotor_control/simulation/rpg_rotors_interface/parameters/autopilot_air.yaml`.

Higher level control commands are also supported by a [MPC](https://github.com/uzh-rpg/rpg_mpc) controller. The parameters are are available in `path/to/FlightBench/dep/rpg_mpc/parameters/air.yaml`.\

- Velocity contorl: when setting desired velocities, the quadrotor try tracking the target velocities. Send velocity commands (in `geometry_msgs/TwistStamped` msg) via topic `/<quad_name>/autopilot/velocity_command` to enable velocity mode.
- Full-state control: the message `quadrotor_msgs/TrajectoryPoint` supports both linear and angular states up to 5th order. You can publish the target states with the desired timestamp using the message.

## Training interface
FlightBench provides both state-based and image-based gym-like interfaces for RL training. We encourage users to use our environment as a code base to develop more RL algorithms & applications for drones.

We provide a gym-like base environment `path/to/FlightBench/flightrl/onpolicy/envs/base_vec_env.py`. Users can define their own training environment based on this.

### State-based RL environment
We give an example `onpolicy.envs.learning_min_time.state_based_vec_env`.
- action space: shape: (4, ), range: [-inf, inf]. Then use tanh to map the input into [-1, 1], corresponding to the collective thrust and body rates. A PD controller is applied for tracking the desired command.
- obs space: shape: (13, ), range: [[-inf, inf]], containing position, orientation, linear & angular velocities of quadrotors.

Users can customize their own environment by modifying several functions like `step`, `get_obs`, and `cal_reward`.

### Vision-based RL environment
Please refer to `onpolicy.envs.learning_perception_aware.perception_aware_vec_env`. We use a bridge, communicating with unity server, to get images.

The observation space could be set as a mixed dict:
```python
obs_space_dict = {
    'imu': spaces.Box(low=np.float32(-np.inf), high=np.float32(np.inf),
                        shape=(self.args.state_len * self.n_imu, ),
                        dtype="float32"),
    'img': spaces.Box(low=0, high=256,
                        shape=(320, 240),
                        dtype="uint8"),
}
```


# Let's Fly
## Evaluate task difficulty
The task difficulty metrics defines on test cases. Each test case consists of a scenario, a start & end point, and a guiding path. The scenario, start point, and end point are self defined. We use path searching method from [sb_min_time](https://github.com/uzh-rpg/sb_min_time_quadrotor_planning) to generate guiding path.

According to the instructions of [Fly sb_min_time](#fly-sb_min_time), the topological guiding path will be generated at first step. Then move them into the origanized scene folder. Use the following command to generate the task difficulty value:
```bash
# activate flightpy venv
source path/to/flightpy/bin/activate
cd path/to/FlightBench/flightbench/scripts
python3 cal_difficulty.py scene/<scene_name>
# for example:
python3 cal_difficulty.py scene/maze-mid
```

We provide 8 test cases for evaluation.

| test_case_num |     name    |  TO  |  VO  |   AOL  |
|:-------------:|:-----------:|:----:|:----:|:------:|
| 0             |  forest-low | 0.76 | 0.30 | 7.6e-4 |
| 1             |  forest-mid | 0.92 | 0.44 | 1.6e-3 |
| 2             | forest-high |  0.9 |  0.6 | 5.7e-3 |
| 3             |   maze-low  | 1.42 | 0.51 | 1.4e-3 |
| 4             |   maze-mid  | 1.51 | 1.01 |  0.01  |
| 5             |  maze-high  | 1.54 | 1.39 |  0.61  |
| 6             |  racing-low | 1.81 | 0.55 |  0.08  |
| 7             |  racing-mid | 1.58 | 1.13 |  0.94  |

## Fly with baseline algorithms
We integrate several representative planning algorithms in FlightBench, as detailed in the table below:

| No |   baseline name   |      method type      | code base |
|:--:|:-----------------:|:---------------------:|-----------|
|  1 |    sb_min_time    | privileged & sampling |<https://github.com/uzh-rpg/sb_min_time_quadrotor_planning>|
|  2 |    fast_planner   |   Optimization-based  |<https://github.com/HKUST-Aerial-Robotics/Fast-Planner>|
|  3 |    ego_planner    |   Optimization-based  |<https://github.com/ZJU-FAST-Lab/ego-planner>|
|  4 |    tgk_planner    |   Optimization-based  |<https://github.com/ZJU-FAST-Lab/TGK-Planner>|
|  5 |   agile_autonomy  |           IL          |<https://github.com/uzh-rpg/agile_autonomy>|
|  6 | learning_min_time |    privileged & RL    |     /     |
|  7 |    learning_pa    |        RL + IL        |     /     |

### Fly sb_min_time
1. Generate ESDF map from the pointcloud

```bash
cd path/to/bench_sb_min_time
# activate flightpy venv
source path/to/flightpy/bin/activate

# get surface first
python3 python/pcd_getsurface.py <pointcloud_path> <resolution>
# then generate ESDF
python3 python/map_pcd.py <pointcloud_path>
mv <pointcloud_path>.npy path/to/bench_sb_min_time/maps
```

2. Generate trajectory

Then modify `sst.yaml` to set the map and waypoints. Run `./start.sh` to generate trajectories.

3. Start flying

Start simulator first
```bash
cd path/to/FlightBench/flightbench/scripts
./start_simulator.sh <test_case_num> <baseline_name>
```

Then start flying

```bash
# in another terminal
cd path/to/FlightBench/flightbench/scripts
./start_baseline.sh <test_case_num> <baseline_name>
```

### Fly other baselines
We provide a unified test launch interface in `start_simulator.sh` and `start_baseline.sh`. Use the following commands to start a flight.
```bash
cd path/to/FlightBench/flightbench/scripts
./start_simulator.sh <test_case_num> <baseline_name>

#in another terminal
./start_baseline.sh <test_case_num> <baseline_name>
```
For learning-based methods, activating flightpy venv before start the baseline is needed.

## Results
ROS bags will be saved automatically. Use the following command to parse the bag.
```bash
python3 bag_parser.py <bag_folder> <test_case_num> <baseline_name>
```

For conclusions and more details about our experiments, please refer to our [paper](https://arxiv.org/abs/2406.05687)

# Train Own Policy
We use [MAPPO](https://github.com/marlbenchmark/on-policy) to train the RL-related planning methods. 

## Train & eval learning_min_time policy

After installing flightgym and MAPPO, just activate flightpy venv and enter `path/to/FlightBench/flightrl/onpolicy/scripts` directory. Then use `./train_min_time_<scene_name>.sh` to start training. The RL training process is logged using [wandb](https://wandb.ai).

Use `./train_min_time_<scene_name>.sh` to evaluate the policy. Remember to change the 'model_dir' parameter in the .sh file to select which model to evaluate.

Feel free to adjust hyper-parameters in the .sh files!

## Train & eval learning_perception_aware policy

Training a perception aware policy can be divided into two stages: RL + IL, while the IL stage consists of two steps.

1. RL stage: use `./train_perception_aware_<scene_name>.sh` to train a state-based perception-aware teacher.
2. Pre-train stage: use `./collect_perception_aware_<scene_name>.sh` to collect data into the "offline_data" folder. Then use `python3 runner/studen_trainer.py` to pretrain the visual encoder and the action network. Before data collection, you need to start `FM-full.x86_64` (by double click) for rendering.
3. IL stage: use `dagger_train_pa_student_<scene_name>.sh` to train a vision-based student. We use DAgger to distill knowledge from the teach to the student. The training process will be logged using [wandb](https://wandb.ai). Key parameters are listed in the .sh files and .py files. Feel free to adjust them!

Then we can evaluate the student policy using the same way described in [Fly with baseline algorithms](#fly-with-baseline-algorithms).

1. Modify the `model_dir` parameter to choose your policy.
2. Use `start_simulator` and `start_baseline` to start a flight, as discussed in [Fly with baseline algorithms](#fly-with-baseline-algorithms).

Refer to the reward curves if you train with our default settings.
![curve](r-curve.jpg)

## Train agile_autonomy policy

We recommand creating another workspace and using their [original pipeline](https://github.com/uzh-rpg/agile_autonomy) to train an agile_autonomy policy. Due to differences in gcc version, training within this workspace may cause crashes.


# Citation
Please cite our paper if you use FlightBench in your work.
```bibtex
@article{yu2024flightbench,
  title={FlightBench: A Comprehensive Benchmark of Spatial Planning Methods for Quadrotors},
  author={Yu, Shu-Ang and Yu, Chao and Gao, Feng and Wu, Yi and Wang, Yu},
  journal={arXiv preprint arXiv:2406.05687},
  year={2024}
}
```


# Acknowledgement

