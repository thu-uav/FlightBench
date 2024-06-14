import numpy as np
from flightgym import QuadrotorEnv_v2
from threading import Timer
import time
import math

def main():
    test_num = 300
    frequency = 20

    # init the env
    inner = QuadrotorEnv_v2("/home/ysa/workspace/flightmare_ws/flightlib/configs/vec_env.yaml", 
                            "/home/ysa/workspace/flightmare_ws/flightlib/configs/quadrotor_env.yaml",
                            "/home/ysa/workspace/flightmare_ws/flightlib/configs/dyn_param.yaml")

    obs_dim = inner.getObsDim()
    act_dim = inner.getActDim()
    num_env = inner.getNumOfEnvs()
    extra_info_name = inner.getExtraInfoNames()
    print("num_env: ", num_env)
    print("extra_info: ", extra_info_name)
    init = np.zeros((num_env, obs_dim), dtype=np.float32)
    for i in range(2):
        for j in range(10):
            init[i*10+j][2] = 1.0+i*0.5
            init[i*10+j][0] = -5+j
            init[i*10+j][1] = i
            init[i*10+j][3] = 1.0
    inner.reset(init)
    inner.connectUnity()
    num = 0
    states_total = np.zeros((test_num, num_env, obs_dim), dtype=np.float32)

    def test_eight():
        action = np.zeros((num_env, act_dim), dtype = np.float32)
        print("num: ", num)
        
        #action[0][0] = -0.5

        obs = np.zeros([num_env, obs_dim],
                        dtype=np.float32)
        
        for i in range(2):
            for j in range(10):
                obs[i*10+j][0] = -5+abs(j)*(-1)**(j%2)
                obs[i*10+j][1] = i
                obs[i*10+j][2] = 1.0+i*0.5

        a = 2
        vmax = 2
        scale = 2*3.41421/(a*vmax)

        sample_time = 0.15/(vmax*a)
        u = num*sample_time
        y = (a*math.cos(u/scale-math.sin(u/scale)/1.414))/(1+math.sin(u/scale-math.sin(u/scale)/1.414)**2)
        x = (a*math.cos(u/scale-math.sin(u/scale)/1.414)*math.sin(u/scale-math.sin(u/scale)/1.414))/(1+math.sin(u/scale-math.sin(u/scale)/1.414)**2)
        vy = (a*(-2+1.414*math.cos(u/scale))*(5+math.cos(2*u/scale-1.414*math.sin(u/scale)))*math.sin(u/scale-math.sin(u/scale)/1.414))/(scale*(-3+math.cos(2*u/scale-1.414*math.sin(u/scale)))**2)
        vx = -(a*(-2+1.414*math.cos(u/scale))*(-1+3*math.cos(2*u/scale-1.414*math.sin(u/scale))))/(scale*(-3+math.cos(2*u/scale-1.414*math.sin(u/scale)))**2)
        heading = math.atan2(vy,vx)
        obs[:,3] = math.cos(heading / 2)
        obs[:,6] = math.sin(heading / 2)
        obs[:,0] += x
        obs[:,1] += y

        
        inner.reset(obs)
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
        test_eight()
        num+=1
        last_time = time.time()
    end = time.time()

    # for i in range(num_env):
    #     np.savetxt("results/"+str(i)+'state.txt', states_total[:, i, :])

    # plt.figure()
    # plt.plot(states_total[:,0,0], label = "x")
    # plt.plot(states_total[:,0,1], label = "y")
    # plt.plot(states_total[:,0,2], label = "z")
    # plt.legend()
    # plt.title("Quadrotor Position")
    # plt.savefig('quad-orientation.png')

    print("parallel  collect %d traj use time %fs" % (num_env, end-start))
    print("RGBD Test Done!")

if __name__ == "__main__":
    main()