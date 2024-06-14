# FlightBench

FlightBench is an open-source comprehensive benchmark for 3D spatial planning on quadrotors, comparing classical optimization-based methods with emerging learning based approaches. For more details, please refer to our [paper](https://)

## Installation
Test on Ubuntu-20.04.
Install RL part first
```bash
mkdir flightbench_ws && cd flightbench_ws
mkdir src && cd src
sudo apt-get update && sudo apt-get install git cmake python3 python3-dev python3-venv python3-rosbag
git clone git@github.com:thu-uav/FlightBench.git
git submodule update --init
cd flightmare

# We recommend using python virtual env
python3 -m venv flightpy # or any other name you like

# Add FLIGHTMARE_PATH environment variable to .bashrc file:
echo "export FLIGHTMARE_PATH=/path/to/FlightBench" >> ~/.bashrc
source ~/.bashrc
source flightpy/bin/activate # active the venv

cd $FLIGHTMARE_PATH/flightrl
sudo apt-get install build-essential cmake libzmqpp-dev libopencv-dev libeigen3-dev
pip install -r requirements.txt
cd $FLIGHTMARE_PATH/flightlib
# for first time install
mkdir build
cd build
cmake -S .. -B . -DCMAKELists_TXT=../CMakeLists_first.txt
cd ..
rm -rf build

# then, install flightmare-pip
pip install .

cd $FLIGHTMARE_PATH/flightrl
pip install -e .
```
Then download scenes from [here]() and put them to `path/to/FlightBench/scene`

Before install FlightBench, install [ROS Noetic](https://wiki.ros.org/noetic/Installation).
```bash
sudo apt-get update
export ROS_DISTRO=noetic
sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool ros-$ROS_DISTRO-mavros
sudo apt-get install python3-pip
sudo pip install catkin-tools

# go flightbench_ws 
cd flightbench_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
# import benchmarks
vcs-import < flightmare/flightbench/benchmarks.yaml
# install deps
sudo apt-get install libarmadillo-dev
# instsll nlopt
# in any dir
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

# back to flightbench_ws
cd flightbench_ws
catkin build
mkdir FlightBench/flightbench/data
```

Make another python venv to run agile-autonomy
```bash
# deactivate from existing venvs
cd path\yo\benchmark_agile_autonomy
python3 -m venv agilepy # or any other name you like
source agilepy/bin/activate
pip install -r requirements.txt
```
Download [this](). Put it into `path/to/FlightBench/flightrender` after unzip.

## Start benchmark
```bash
# terminal 1
# into flightbench_ws
source devel/setup.bash
cd src/FlightBench/flightbench/script
./start_simulator.sh <test_id> <method_type>

# terminal 2
# into flightbench_ws
source devel/setup.bash
cd src/FlightBench/flightbench/script
./start_benchmark.sh <test_id> <method_type>
```
test_ids are listed in the following table:

| **scenario** | **test** | **test_id** |
|:------------:|:--------:|:-----------:|
| forest       | 1        | 0           |
|              | 2        | 1           |
|              | 3        | 2           |
| maze         | 1        | 3           |
|              | 2        | 4           |
|              | 3        | 5           |
| MW           | 1        | 6           |
|              | 2        | 7           |

Method_types are listed following:
- ego_planner
- fast_planner
- tgk_planner
- agile_autonomy
- learning_pa
- learning_min_time
- sb_min_time

## Train your own policy
```bash
# in flightpy venv
cd /path/to/FlightBench/flightrl/onpolicy/scripts

# use these sh files to start training a state based policy:
./train_min_time_<scenario>-<test>.sh

# use these sh files to start training a perception aware teacher:
./train_perception_aware_<scenario>-<test>.sh

# use these sh files to collect data using a perception aware teacher
./collect_perception_aware_<scenario>-<test>.sh

# run runner/student_trainer.py to pretrain a student
# run these sh files to train a vision student
./dagger_train_pa_student_<scenario>_<test>.sh
```

## License
