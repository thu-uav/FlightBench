import numpy as np
from flightgym import QuadrotorEnv_v2
from threading import Timer
import time
import math

def main():
    test_num = 362
    frequency = 40

    # init the env
    inner = QuadrotorEnv_v2("/home/ysa/workspace/flightmare_ws/src/flightmare/flightlib/configs/vec_env.yaml", 
                            "/home/ysa/workspace/flightmare_ws/src/flightmare/flightlib/configs/quadrotor_env.yaml",
                            "/home/ysa/workspace/flightmare_ws/src/flightmare/flightlib/configs/dyn_param.yaml")
    z = [1+i*0.05 for i in range(20)]
    obs_dim = inner.getObsDim()
    act_dim = inner.getActDim()
    num_env = inner.getNumOfEnvs()
    extra_info_name = inner.getExtraInfoNames()
    print("num_env: ", num_env)
    print("extra_info: ", extra_info_name)
    init = np.zeros((num_env, obs_dim), dtype=np.float32)
    for i in range(2):
        for j in range(10):
            init[i*10+j][0] = -5+abs(j / 2)*(-1)**(j%2)
            init[i*10+j][1] = i
            init[i*10+j][2] = 0.5
            init[i*10+j][3] = 1.0
    flight_id = list(map(int, input("please input ids in 0~19, use space to split: ").split()))
    print(flight_id)
    inner.reset(init)
    inner.connectUnity()
    num = 0
    states_total = np.zeros((test_num, num_env, obs_dim), dtype=np.float32)
    acc_g = np.zeros((num_env, 3))

    def test_circle():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        # print("time: ", num * 0.025)
        
        #action[0][0] = -0.5

        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        
        for i in range(2):
            for j in range(10):
                obs[i*10+j][0] = -5+abs(j / 2)*(-1)**(j%2)
                obs[i*10+j][1] = i
                obs[i*10+j][2] = z[i*10+j]

        a = 2
        vmax = 1.5
        # scale = 2*3.41421/(a*vmax)

        sample_time = 0.025 * vmax / a
        u = num*sample_time
        u_max = test_num * sample_time
        # if u < 1:
        #     u = 1 - np.cos(u * np.pi / 2)
        # if u_max - u <1:
        #     u = u_max - 1 + np.sin((u-u_max+1) * np.pi / 2)

        if u < 0.5:
            u = u ** 2
        elif u<u_max-0.5:
            u = u-0.25
        else:
            u = u_max - 0.5 - (u-u_max)**2
        # print("u: ", u)

        y = a * np.sin(u)
        x = a * np.cos(u) - a
        vy = vmax * np.cos(u)
        vx = -vmax * np.sin(u)
        heading = math.atan2(vy,vx)
        for i in range(num_env):
            if i in flight_id:
                obs[i,0] += x
                obs[i,1] += y
                obs[i,3] = math.cos(heading / 2)
                obs[i,6] = math.sin(heading / 2)
        
        obs[:, :3] += acc_g
        inner.reset(obs)
        states_total[num, :, :] = obs

    def hover():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        action[:, 0] = (9.81 / 20.0) * 2 - 1 + ((np.array(z)-obs[:, 2]) - obs[:, 9]) * 0.5
        done = np.zeros((num_env), dtype=bool)
        extra_info = np.zeros([num_env, len(extra_info_name)], 
                              dtype=np.float32)
        inner.step(action, obs, done, extra_info)
    # start_test
    start = time.time()
    last_time = time.time()
    print("Start xushi test!")
    obs = np.zeros([num_env, obs_dim], dtype=np.float32)
    while(num<130):
        if time.time()-last_time < 0.975*1.0/frequency:
            continue
        hover()
        num+=1
        last_time = time.time()
    num = 0
    while(num<test_num):
        if time.time()-last_time < 0.975*1.0/frequency:
            continue
        test_circle()
        acc_g*=0.9
        acc_g+=0.1 * 0.03 * np.random.randn(num_env, 3)
        num+=1
        last_time = time.time()
    print("xushi Test Done!")
    num = 0
    while(num<130):
        if time.time()-last_time < 0.975*1.0/frequency:
            continue
        hover()
        num+=1
        last_time = time.time()
    end = time.time()

    for i in range(num_env):
        if i in flight_id:
            np.savetxt("position/"+str(i)+'state.txt', states_total[:, i, :3])

if __name__ == "__main__":
    main()