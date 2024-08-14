#!/bin/bash
quad_name="air"
echo -e "\nstarting simulator..."

if [ $# != 2 ];
then echo -e "\033[31mPlease use: './start_simulator <test_case_num> <baseline_name>' to start simulator\033[0m"
exit -1
fi

case $1 in
    0) echo "test case 0"
    scene_id=0
    init_x=-14.0
    init_y=-14.0
    init_yaw=0.7854
    #init_yaw=0.0
    height=2.07
    end_x=14.0
    end_y=14.0
    end_z=2.0
    ;;
    1) echo "test case 1"
    scene_id=1
    init_x=-10.0
    init_y=12.5
    init_yaw=-0.895
    # init_yaw=0.0
    height=2.07
    end_x=10.0
    end_y=-12.5
    end_z=2.0
    ;;
    2) echo "test case 2"
    scene_id=2
    init_x=-14.0
    init_y=14.0
    init_yaw=-0.7854
    #init_yaw=0.0
    height=2.0
    end_x=14.0
    end_y=-14.0
    end_z=2.0
    ;;
    3) echo "test case 3"
    scene_id=3
    init_x=-4.2
    init_y=15.0
    init_yaw=-1.5708
    # init_yaw=0.0
    height=2.5
    end_x=-4.2
    end_y=-9.5
    end_z=2.5
    ;;
    4) echo "test case 4"
    scene_id=3
    init_x=-0.6
    init_y=15.0
    init_yaw=-1.5708
    # init_yaw=0.0
    height=2.5
    end_x=7.0
    end_y=-12.0
    end_z=2.5
    ;;
    5) echo "test case 5"
    scene_id=3
    init_x=7.0
    init_y=-12.0
    init_yaw=2.094
    #init_yaw=0.0
    height=2.5
    end_x=10.5
    end_y=5.4
    end_z=2.5
    ;;
    6) echo "test case 6"
    scene_id=4
    init_x=2.35
    init_y=7.61
    init_yaw=-2.43
    # init_yaw=0.0
    height=2.07
    end_x=-6.0
    end_y=-6.0
    end_z=4.0
    ;;
    7) echo "test case 7"
    scene_id=4
    init_x=7.0
    init_y=-8.5
    init_yaw=1.5708
    #init_yaw=0.0
    height=2.0
    end_x=-6.0
    end_y=-6.0
    end_z=1.5
    ;;
    *) echo -e "\033[31mInvalid test_case_num\033[0m"
    exit -1
    ;;
esac

case $2 in
    "sb_min_time") echo "baseline: sb_min_time"
    if_agile=false
    ;;
    "ego_planner") echo "baseline: ego_planner"
    if_agile=false
    ;;
    "fast_planner") echo "baseline: fast_planner"
    if_agile=false
    ;;
    "tgk_planner") echo "baseline: tgk_planner"
    if_agile=false
    ;;
    "learning_min_time") echo "baseline: learning_min_time"
    if_agile=false
    ;;
    "agile_autonomy") echo "baseline: agile_autonomy"
    if_agile=true
    ;;
    "learning_pa") echo "baseline: learning_pa"
    if_agile=false
    ;;
    *) echo -e "\033[31mInvalid baseline_name\033[0m"
    exit -1
    ;;
esac

roslaunch flightros rotors_bench.launch scene:=${scene_id} x_init:=${init_x} y_init:=${init_y}  yaw_init:=${init_yaw} height:=${height} test_case_num:=$1 baseline_name:=$2 x_end:=${end_x} y_end:=${end_y} z_end:=${end_z} if_agile:=${if_agile}