import numpy as np
from flightgym import QuadrotorEnv_v2
from threading import Timer
import time

def main():
    test_num = 1000
    frequency = 24

    # init the env
    inner = QuadrotorEnv_v2("/home/ysa/workspace/flightmare_unitytest/flightlib/configs/vec_env.yaml", 
                            "/home/ysa/workspace/flightmare_unitytest/flightlib/configs/quadrotor_env.yaml",
                            "/home/ysa/workspace/flightmare_unitytest/real2sim/src/optimize/config/target_param.yaml")

    obs_dim = inner.getObsDim()
    act_dim = inner.getActDim()
    num_env = inner.getNumOfEnvs()
    extra_info_name = inner.getExtraInfoNames()
    print("num_env: ", num_env)
    print("extra_info: ", extra_info_name)
    init = np.zeros((num_env, obs_dim), dtype=np.float32)
    for i in range(5):
        for j in range(10):
            init[i*10+j][2] = 1.0+i*0.5
            init[i*10+j][0] = -5+j
            init[i*10+j][1] = i
            init[i*10+j][3] = 1.0
    inner.reset(init)
    inner.connectUnity()
    num = 0
    states_total = np.zeros((test_num, num_env, obs_dim), dtype=np.float32)

    def test_takeoff():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        print("num: ", num)
        if num<200:
            action[:,0] = -0.5
        elif num<300:
            for i in range(num_env):
                action[i,0] = 0.1*(i % 10*0.1-states_total[num-1][i][9])-0.5
        elif num<700:
            for i in range(num_env):
                action[i,0] = 0.1*(0-states_total[num-1][i][9])-0.5
        elif num<800:
            for i in range(num_env):
                action[i,0] = 0.1*(-(i % 8*0.1)-states_total[num-1][i][9])-0.5
        else:
            for i in range(num_env):
                action[i,0] = 0.1*(0-states_total[num-1][i][9])-0.5

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
        action[:,0] = -0.5
        for i in range(num_env):
            action[i,3] = -0.25+i*0.01

        #action[0][0] = -0.5

        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        done = np.zeros((num_env), dtype=bool)
        extra_info = np.zeros([num_env, len(extra_info_name)], 
                              dtype=np.float32)
        inner.step(action, obs, done, extra_info)
        states_total[num, :, :] = obs

    # start_test
    start = time.time()
    last_time = time.time()
    print("Start RGBD camera test!")
    while(num<test_num):
        if time.time()-last_time < 0.975*1.0/frequency:
            continue
        test_takeoff()
        num+=1
        last_time = time.time()
    end = time.time()

    for i in range(num_env):
        np.savetxt("results/"+str(i)+'state.txt', states_total[:, i, :])

    # plt.figure()
    # plt.plot(states_total[:,0,0], label = "x")
    # plt.plot(states_total[:,0,1], label = "y")
    # plt.plot(states_total[:,0,2], label = "z")
    # plt.legend()
    # plt.title("Quadrotor Position")
    # plt.savefig('quad-orientation.png')

    print("%d step inference use time %fs, average frequency: %f" % (test_num, end-start, test_num/(end-start)))
    print("RGBD Test Done!")

if __name__ == "__main__":
    main()