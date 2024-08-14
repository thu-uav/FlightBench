# FlightBench
![Overview of FlightBench](docs/overview.jpg)
---
*FlightBench* is an open-source comprehensive benchmark for 3D spatial path planning on quadrotors built on [Flightmare](https://github.com/uzh-rpg/flightmare). *FlightBench* provides cusomizable test scenarios (including three quantitative task difficulty metrics), representative planning algorithms, and comprehensive evaluation metrics. FlightBench also integrates the [MAPPO](https://github.com/marlbenchmark/on-policy) algorithm, facilitating the training of RL-based planning methods.

For usage and more details, please refer to the [documentation]()

## Citation
Please cite our paper [FlightBench: A Comprehensive Benchmark of Spatial Planning Methods for Quadrotors](https://arxiv.org/abs/2406.05687) if you use FlightBench in your work.
```bibtex
@article{yu2024flightbench,
  title={FlightBench: A Comprehensive Benchmark of Spatial Planning Methods for Quadrotors},
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