#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include "quadrotor_msgs/ControlCommand.h"
#include "quadrotor_msgs/AutopilotFeedback.h"
#include "flightlib/objects/quadrotor.hpp"
#include "ceres/ceres.h"
#include <cmath>
#include "yaml-cpp/yaml.h"

namespace optimize{ 
    struct OptimizeConfig {
        std::string bag_path;
        std::string odom_topic;
        std::string command_topic;
        std::string default_param;
        std::string target_param;
        std::string res_path;
        std::string feedback_topic;
    };

    struct Odometry {
        double time;
        float q_x;
        float q_y;
        float q_z;
        float q_w;
        float x;
        float y;
        float z;
        float v_x;
        float v_y;
        float v_z;
        float omega_x;
        float omega_y;
        float omega_z;

        friend std::ostream & operator<<( std::ostream & os,const optimize::Odometry & odom) {
            os<<"Odometry: "<<std::endl;
            os<<"  time: "<<odom.time<<std::endl;
            os<<"  position: ["<<odom.x<<", "<<odom.y<<", "<<odom.z<<"]"<<std::endl;
            os<<"  orientation: ["<<odom.q_x<<", "<<odom.q_y<<", "<<odom.q_z<<", "<<odom.q_w<<"]"<<std::endl;
            os<<"  Velocity: ["<<odom.v_x<<", "<<odom.v_y<<", "<<odom.v_z<<"]"<<std::endl;
            os<<"  Omega: ["<<odom.omega_x<<", "<<odom.omega_y<<", "<<odom.omega_z<<"]"<<std::endl;
            return os;
        }
    };
    struct Command {
        double time;
        bool armed;
        unsigned char control_mode;// NONE, ATTITUDE, RATE, ACC, ROTORS
        float ct;
        float omega_x;
        float omega_y;
        float omega_z;
        friend std::ostream & operator<<( std::ostream & os,const optimize::Command cmd) {
            os<<"Command: "<<std::endl;
            os<<"  time: "<<cmd.time<<std::endl;
            os<<"  CT: "<<cmd.ct<<std::endl;
            os<<"  BR: ["<<cmd.omega_x<<", "<<cmd.omega_y<<", "<<cmd.omega_z<<"]"<<std::endl;
            return os;
        }
    };

    class QuadCost {
        public:
        QuadCost(const optimize::Odometry odom, const optimize::Command command, const optimize::Odometry odom_next,flightlib::Quadrotor* const quad_ptr) :  odom_(odom), cmd_(command), odom_next_(odom_next), quad_ptr_(quad_ptr) {dt_ = odom_next_.time-odom_.time;}

        template<typename T>
        bool operator()(const T* const parameters, T* residuals) const;

        template<typename T>
        optimize::Odometry eval(const T* const parameters, T* residuals) const;

        private:
        bool convert_optimize_to_quadstate(const optimize::Odometry& odom, const optimize::Command& cmd, flightlib::QuadState& current_state, flightlib::Command& cmd_input) const;
        bool convert_quadstate_to_optimize(const flightlib::QuadState& state, optimize::Odometry& odom) const;
        const optimize::Odometry odom_, odom_next_;
        const optimize::Command cmd_;
        flightlib::Quadrotor* const quad_ptr_;
        double dt_;
    };

    class Optimizer {
        public:
        Optimizer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
        ~Optimizer() {}
        bool read_bag();
        bool read_feedback();
        bool get_config();
        bool test_onestep(std::string mode = "default");
        bool test_boostrap(std::string mode = "default");
        bool optimize();
        template<typename T>
        std::pair<double, double> mean_and_var(const std::vector<T> data) {
            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            double mean = sum / data.size();
            double acc = 0;
            for(auto i:data) {acc+=std::pow(i-mean, 2);}
            auto stddev = std::sqrt(acc/(data.size()-1));
            return std::pair<double, double>(mean, stddev);
        }

        template<typename T>
        bool read_yaml(const std::string path, T* const param);

        template<typename T>
        bool write_yaml(const std::string path, T* const param);


        private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        optimize::OptimizeConfig config_;
        std::vector<std::pair<optimize::Odometry, optimize::Command>> pairs_;
    };
}