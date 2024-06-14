# Flightmare - thu-uav改版

**Flightmare** is a flexible modular quadrotor simulator.
Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.
Those two components are totally decoupled and can run independently from each other. 

This repo is forked from https://github.com/uzh-rpg/flightmare. We modified some interfaces and dynamics of the simulator to make it more suitable for RL training and real world deployment.

We developed a flight benchmark including several learning/sample-based/optimize-based baselines based on this simulator.

## Installation
```bash
mkdir flightmare_ws && cd flightmare_ws
mkdir src && cd src
# verified on ubuntu 18.04 and 20.04
sudo apt-get update && sudo apt-get install git cmake python3 python3-dev python3-venv python3-rosbag
git clone git@github.com:thu-uav/flightmare.git
cd flightmare

# We recommend using python virtual env
# Anaconda may affect the compilation of c++ workspace
python3 -m venv flightpy # or any other name you like

# Add FLIGHTMARE_PATH environment variable to .bashrc file:
echo "export FLIGHTMARE_PATH=/path/to/flightmare" >> ~/.bashrc
source ~/.bashrc
source flightpy/bin/activate # use deactivate to quit venv

cd $FLIGHTMARE_PATH/flightrl
sudo apt-get install build-essential cmake libzmqpp-dev libopencv-dev libeigen3-dev
pip install -r requirements.txt
cd $FLIGHTMARE_PATH/flightlib
# for first time install
mkdir build
cd build
# change CMakeLists line 46 and 47 to
include(cmake/pybind11.cmake)
include(cmake/yaml.cmake)
# then cmake to get dependencies in external folder
cmake ..
cd ..
rm -rf build

# then, change CMakeLists line 46 and 47 to
include(cmake/pybind11_simple.cmake)
include(cmake/yaml_simple.cmake)
pip install .

# for subsequent installations
rm -rf build
# make sure CMakeLists.txt line 46 and 47 with "simple"
pip install .

cd $FLIGHTMARE_PATH/flightrl
pip install -e .
# then use test.sh to test environments installation
```
if want to use ros-version, go ahead
```bash
# # verified on ubuntu 20.04
# install ros
# install deps
sudo apt-get update
export ROS_DISTRO=noetic
sudo apt-get install libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool ros-$ROS_DISTRO-mavros
sudo apt-get install python3-pip
sudo pip install catkin-tools

# go flightmare_ws 
cd flightmare_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
vcs-import < flightmare/flightbench/dependencies.yaml
catkin build
mkdir flightmare/flightbench/data
pip install pyqt5

# install benchmarks
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

# TODO: put dependencies into submodules, benchmarks into vsc-import
```

## Let's Fly!
```bash
# terminal 1
roslaunch flightros rotors_bench.launch
```
Modify 'flightmare/flightros/params/default.yaml' to enable/disable vision output.



```

```

### License
