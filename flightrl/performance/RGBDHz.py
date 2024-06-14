import numpy as np
from flightgym import QuadrotorEnv_v2
from threading import Timer
import time

def main():
    frequency = 24
    test_num = 1000

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
    init[0,2] = 1
    inner.reset(init)
    inner.connectUnity()
    num = 0

    def test_step():
        action = np.zeros((num_env, 4), dtype = np.float32)
        action[0,0] = -0.5
        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        done = np.zeros((num_env), dtype=bool)
        extra_info = np.zeros([num_env, len(extra_info_name)], 
                              dtype=np.float32)
        inner.step(action, obs, done, extra_info)

    # start_test
    print("Start RGBD frequency test!")
    start = time.time()
    last_time = time.time()
    while(num<test_num):
        if(time.time()-last_time<0.985/frequency):
            continue
        test_step()
        num+=1
        last_time = time.time()
    end = time.time()

    print("%d step inference use time %fs, average frequency: %f" % (test_num, end-start, test_num/(end-start)))
    print("RGBD Performance Testing Done!")

if __name__ == "__main__":
    main()