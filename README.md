# FlightBench
[![Static Badge](https://img.shields.io/badge/flightmare-red)](https://github.com/uzh-rpg/flightmare)
[![Python](https://img.shields.io/badge/python-3.8-yellow.svg)](https://docs.python.org/3.8/whatsnew/3.7.html)
[![Docs](https://img.shields.io/badge/docs-passing-brightgreen.svg)](https://thu-uav.github.io/FlightBench/)
[![License: GPLv3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/)


![Overview of FlightBench](docs/overview.jpg)

<font><div align='center' > [[📖 Documentation](https://thu-uav.github.io/FlightBench/)]  [[📜 arXiv Paper](https://arxiv.org/abs/2406.05687)] </div> </font>

*FlightBench* is an open-source comprehensive benchmark for planning methods on ego-vision-based navigation for quadrotors built on [Flightmare](https://github.com/uzh-rpg/flightmare). *FlightBench* provides cusomizable test scenarios (including three quantitative task difficulty metrics), representative planning algorithms, and comprehensive evaluation metrics. FlightBench also integrates the [MAPPO](https://github.com/marlbenchmark/on-policy) algorithm, facilitating the training of RL-based planning methods.

For usage and more details, please refer to the [documentation](https://thu-uav.github.io/FlightBench/)

## 🔥News
**[2024-11-24]** 🎁 We have added our implementation details and full experimental results to our [website](https://thu-uav.github.io/FlightBench/).

## Citation
Please cite our [paper](https://arxiv.org/abs/2406.05687) if you use FlightBench in your work.
```bibtex
@article{yu2024flightbench,
  title={FlightBench: Benchmarking Learning-based Methods for Ego-vision-based Quadrotors Navigation},
  author={Yu, Shu-Ang and Yu, Chao and Gao, Feng and Wu, Yi and Wang, Yu},
  journal={arXiv preprint arXiv:2406.05687},
  year={2024}
}
```

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

All third party libraries we used are listed bellow:

| repository | License |
|:---:|:---:|
| [flightmare](https://github.com/uzh-rpg/flightmare) | [MIT](https://mit-license.org/) |
| [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) | [GPLv3](https://www.gnu.org/licenses/) |
| [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) | [GPLv3](https://www.gnu.org/licenses/) |
| [TGK-Planner](https://github.com/ZJU-FAST-Lab/TGK-Planner) | [GPLv3](https://www.gnu.org/licenses/) |
| [sb_min_time_quadrotor_planning](https://github.com/uzh-rpg/sb_min_time_quadrotor_planning) | [GPLv3](https://www.gnu.org/licenses/) |
| [agile_autonomy](https://github.com/uzh-rpg/agile_autonomy) | [GPLv3](https://www.gnu.org/licenses/) |
