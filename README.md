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