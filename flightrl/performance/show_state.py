import numpy as np
from flightgym import QuadrotorEnv_v2
from threading import Timer
import time
import matplotlib.pyplot as plt

def main():
    test_num = 1000

    # init the env
    inner = QuadrotorEnv_v2("/home/ysa/workspace/flightmare/flightlib/configs/vec_env.yaml", 
                            "/home/ysa/workspace/flightmare/flightlib/configs/quadrotor_env.yaml",
                            "/home/ysa/workspace/flightmare/real2sim/src/optimize/config/target_param.yaml")

    obs_dim = inner.getObsDim()
    act_dim = inner.getActDim()
    num_env = inner.getNumOfEnvs()
    extra_info_name = inner.getExtraInfoNames()
    print("num_env: ", num_env)
    print("extra_info: ", extra_info_name)
    init = np.zeros((num_env, obs_dim), dtype=np.float32)
    init[0][2] = 0.0
    init[0][3] = 1.0
    inner.reset(init)
    num = 0
    states_total = np.zeros((test_num, num_env, obs_dim), dtype=np.float32)

    def test_takeoff():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        print("num: ", num)
        if num<200:
            action[0][0] = -0.5
        elif num<300:
            action[0][0] = 0.2*(0.5-states_total[num-1][0][9])-0.5
        elif num<700:
            action[0][0] = 0.2*(0-states_total[num-1][0][9])-0.5
        elif num<800:
            action[0][0] = 0.2*(-0.5-states_total[num-1][0][9])-0.5
        else:
            action[0][0] = 0.2*(0-states_total[num-1][0][9])-0.5

        #action[0][0] = -0.5

        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        done = np.zeros((num_env), dtype=bool)
        extra_info = np.zeros([num_env, len(extra_info_name)], 
                              dtype=np.float32)
        inner.step(action, obs, done, extra_info)
        states_total[num, :, :] = obs

    def test_turn():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        print("num: ", num)
        action[0][0] = -0.5
        action[0][3] = 0.2

        #action[0][0] = -0.5

        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        done = np.zeros((num_env), dtype=bool)
        extra_info = np.zeros([num_env, len(extra_info_name)], 
                              dtype=np.float32)
        inner.step(action, obs, done, extra_info)
        states_total[num, :, :] = obs

    # start_test
    print("Start quad dynamics test!")
    while(num<test_num):
        test_takeoff()
        num+=1

    plt.figure()
    plt.plot(states_total[:,0,0], label = "x")
    plt.plot(states_total[:,0,1], label = "y")
    plt.plot(states_total[:,0,2], label = "z")
    plt.legend()
    plt.title("Quadrotor Position")
    plt.savefig('quad-orientation.png')

    num = 0
    while(num<test_num):
        test_turn()
        num+=1

    plt.figure()
    plt.plot(states_total[:,0,4], label = "qx")
    plt.plot(states_total[:,0,5], label = "qy")
    plt.plot(states_total[:,0,6], label = "qz")
    plt.plot(states_total[:,0,3], label = "qw")
    plt.legend()
    plt.title("Quadrotor Orientation")
    plt.savefig('quad-orientation.png')

    print("State Test Done!")

if __name__ == "__main__":
    main()