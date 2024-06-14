#include <optimize/optimize.h>
#define Wx 1.0
#define Wv 1.0
#define Wq 1.0
#define Wome 1.0

namespace optimize {
    template<typename T>
    bool QuadCost::operator()(const T* const parameters, T* residuals) const {
        optimize::Odometry odom_next_output;
        flightlib::QuadState state_input, state_next_output;
        flightlib::Command cmd_input;
        std::vector<double> param_into_quad(parameters, parameters+flightlib::OptParam::N_Param);

        //convert from odom to state
        state_input.setZero();
        state_next_output.setZero();
        convert_optimize_to_quadstate(odom_, cmd_, state_input, cmd_input);

        quad_ptr_->runstep(cmd_input, state_input, param_into_quad, dt_, state_next_output);

        // convert result to odom
        convert_quadstate_to_optimize(state_next_output, odom_next_output);
        // print odom
        // std::cout<<"---- exp start----"<<std::endl;
        // std::cout<<"init pose: "<<std::endl;
        // std::cout<<odom_<<std::endl;
        // std::cout<<"final pose(real): "<<std::endl;
        // std::cout<<odom_next_<<std::endl;
        // std::cout<<"final pose(sim): "<<std::endl;
        // std::cout<<odom_next_output<<std::endl;
        // cal residual with result odom and odom_next_

        residuals[0] = std::sqrt(std::pow(odom_next_.x-odom_next_output.x, 2)+std::pow(odom_next_.y-odom_next_output.y, 2)+std::pow(odom_next_.z-odom_next_output.z, 2))*Wx;
        residuals[1] = std::sqrt(std::pow(odom_next_.v_x-odom_next_output.v_x, 2)+std::pow(odom_next_.v_y-odom_next_output.v_y, 2)+std::pow(odom_next_.v_z-odom_next_output.v_z, 2))*Wv;
        residuals[2] = std::sqrt(std::pow(odom_next_.q_x-odom_next_output.q_x, 2)+std::pow(odom_next_.q_y-odom_next_output.q_y, 2)+std::pow(odom_next_.q_z-odom_next_output.q_z, 2)+std::pow(odom_next_.q_w-odom_next_output.q_w, 2))*Wq;
        residuals[3] = std::sqrt(std::pow(odom_next_.omega_x-odom_next_output.omega_x, 2)+std::pow(odom_next_.omega_y-odom_next_output.omega_y, 2)+std::pow(odom_next_.omega_z-odom_next_output.omega_z, 2))*Wome;

        // residuals[4] = std::sqrt(std::pow(odom_next_.x-odom_.x, 2)+std::pow(odom_next_.y-odom_.y, 2)+std::pow(odom_next_.z-odom_.z, 2));
        // residuals[5] = std::sqrt(std::pow(odom_next_.v_x-odom_.v_x, 2)+std::pow(odom_next_.v_y-odom_.v_y, 2)+std::pow(odom_next_.v_z-odom_.v_z, 2));
        // residuals[6] = std::sqrt(std::pow(odom_next_.q_x-odom_.q_x, 2)+std::pow(odom_next_.q_y-odom_.q_y, 2)+std::pow(odom_next_.q_z-odom_.q_z, 2)+std::pow(odom_next_.q_w-odom_.q_w, 2));
        // residuals[7] = std::sqrt(std::pow(odom_next_.omega_x-odom_.omega_x, 2)+std::pow(odom_next_.omega_y-odom_.omega_y, 2)+std::pow(odom_next_.omega_z-odom_.omega_z, 2));
        // std::cout<<"error x: "<<residuals[0]<<std::endl;

        // std::cout<<"---- exp end ----"<<std::endl<<std::endl;
        return true;
    }

    template<typename T>
    optimize::Odometry QuadCost::eval(const T* const parameters, T* residuals) const {
        optimize::Odometry odom_next_output;
        flightlib::QuadState state_input, state_next_output;
        flightlib::Command cmd_input;
        std::vector<double> param_into_quad(parameters, parameters+flightlib::OptParam::N_Param);

        //convert from odom to state
        state_input.setZero();
        state_next_output.setZero();
        convert_optimize_to_quadstate(odom_, cmd_, state_input, cmd_input);

        quad_ptr_->runstep(cmd_input, state_input, param_into_quad, dt_, state_next_output);

        // convert result to odom
        convert_quadstate_to_optimize(state_next_output, odom_next_output);
        // print odom
        std::cout<<"---- exp start----"<<std::endl;
        std::cout<<"init pose: "<<std::endl;
        std::cout<<odom_<<std::endl;
        std::cout<<"final pose(real): "<<std::endl;
        std::cout<<odom_next_<<std::endl;
        std::cout<<"final pose(sim): "<<std::endl;
        std::cout<<odom_next_output<<std::endl;

        //cal residual with result odom and odom_next_
        residuals[0] = std::sqrt(std::pow(odom_next_.x-odom_next_output.x, 2)+std::pow(odom_next_.y-odom_next_output.y, 2)+std::pow(odom_next_.z-odom_next_output.z, 2));
        residuals[1] = std::sqrt(std::pow(odom_next_.v_x-odom_next_output.v_x, 2)+std::pow(odom_next_.v_y-odom_next_output.v_y, 2)+std::pow(odom_next_.v_z-odom_next_output.v_z, 2));
        residuals[2] = std::sqrt(std::pow(odom_next_.q_x-odom_next_output.q_x, 2)+std::pow(odom_next_.q_y-odom_next_output.q_y, 2)+std::pow(odom_next_.q_z-odom_next_output.q_z, 2)+std::pow(odom_next_.q_w-odom_next_output.q_w, 2));
        residuals[3] = std::sqrt(std::pow(odom_next_.omega_x-odom_next_output.omega_x, 2)+std::pow(odom_next_.omega_y-odom_next_output.omega_y, 2)+std::pow(odom_next_.omega_z-odom_next_output.omega_z, 2));

        residuals[4] = std::sqrt(std::pow(odom_next_.x-odom_.x, 2)+std::pow(odom_next_.y-odom_.y, 2)+std::pow(odom_next_.z-odom_.z, 2));
        residuals[5] = std::sqrt(std::pow(odom_next_.v_x-odom_.v_x, 2)+std::pow(odom_next_.v_y-odom_.v_y, 2)+std::pow(odom_next_.v_z-odom_.v_z, 2));
        residuals[6] = std::sqrt(std::pow(odom_next_.q_x-odom_.q_x, 2)+std::pow(odom_next_.q_y-odom_.q_y, 2)+std::pow(odom_next_.q_z-odom_.q_z, 2)+std::pow(odom_next_.q_w-odom_.q_w, 2));
        residuals[7] = std::sqrt(std::pow(odom_next_.omega_x-odom_.omega_x, 2)+std::pow(odom_next_.omega_y-odom_.omega_y, 2)+std::pow(odom_next_.omega_z-odom_.omega_z, 2));

        std::cout<<"---- exp end ----"<<std::endl<<std::endl;
        ros::Duration(0.025).sleep();
        return odom_next_output;
    }

    bool QuadCost::convert_optimize_to_quadstate(const optimize::Odometry& odom, const optimize::Command& cmd, flightlib::QuadState& current_state, flightlib::Command& cmd_input) const {
        current_state.t = 0;
        // only set p q v w, a and tau will be set when running, bw ba unused (I guess)
        current_state.p(0) = odom.x;
        current_state.p(1) = odom.y;
        current_state.p(2) = odom.z;
        current_state.qx(0) = odom.q_w;
        current_state.qx(1) = odom.q_x;
        current_state.qx(2) = odom.q_y;
        current_state.qx(3) = odom.q_z;
        current_state.v(0) = odom.v_x;
        current_state.v(1) = odom.v_y;
        current_state.v(2) = odom.v_z;
        current_state.w(0) = odom.omega_x;
        current_state.w(1) = odom.omega_y;
        current_state.w(2) = odom.omega_z;

        // //set next state
        // next_state.t = odom_next.time-odom.time;
        // // only set p q v w, a and tau will be set when running, bw ba unused (I guess)
        // next_state.p(0) = odom_next.x;
        // next_state.p(1) = odom_next.y;
        // next_state.p(2) = odom_next.z;
        // next_state.qx(0) = odom_next.q_w;
        // next_state.qx(1) = odom_next.q_x;
        // next_state.qx(2) = odom_next.q_y;
        // next_state.qx(3) = odom_next.q_z;
        // next_state.v(0) = odom_next.v_x;
        // next_state.v(1) = odom_next.v_y;
        // next_state.v(2) = odom_next.v_z;
        // next_state.w(0) = odom_next.omega_x;
        // next_state.w(1) = odom_next.omega_y;
        // next_state.w(2) = odom_next.omega_z; 

        //set command
        cmd_input.t = cmd.time-odom.time;
        cmd_input.collective_thrust = cmd.ct;
        cmd_input.omega(0) = cmd.omega_x;
        cmd_input.omega(1) = cmd.omega_y;
        cmd_input.omega(2) = cmd.omega_z;
        return true;
    }

    bool QuadCost::convert_quadstate_to_optimize(const flightlib::QuadState& state, optimize::Odometry& odom) const {
        odom.time = state.t;
        // only set p q v w, a and tau will be set when running, bw ba unused (I guess)
        odom.x = state.p(0);
        odom.y = state.p(1);
        odom.z = state.p(2);
        odom.q_w = state.qx(0);
        odom.q_x = state.qx(1);
        odom.q_y = state.qx(2);
        odom.q_z = state.qx(3);
        odom.v_x = state.v(0);
        odom.v_y = state.v(1);
        odom.v_z = state.v(2);
        odom.omega_x = state.w(0);
        odom.omega_y = state.w(1);
        odom.omega_z = state.w(2);
        return true;
    }

    Optimizer::Optimizer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
    nh_(nh), pnh_(pnh) {
        if(!this->get_config()) abort();
    }

    bool Optimizer::read_bag() {
        rosbag::Bag bag;
        bag.open(config_.bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(config_.odom_topic);
        topics.push_back(config_.command_topic);
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        auto it = std::next(view.begin(), 4900);
        while(it!=view.end() && std::next(it,1)!=view.end() && std::next(it,2)!=view.end() && std::next(it,3)!=view.end()) {
            // std::cout<<it->getTopic()<<" at "<<it->getTime().sec<<"'"<<it->getTime().nsec<<std::endl;
            bool valid = (it->getTopic() == config_.odom_topic && std::next(it,1)->getTopic() == config_.odom_topic && std::next(it,2)->getTopic() == config_.odom_topic &&  std::next(it,3)->getTopic() == config_.command_topic);
            if(valid) {
                nav_msgs::Odometry::ConstPtr odom_ptr_ = it->instantiate<nav_msgs::Odometry>();
                if(odom_ptr_->pose.pose.position.z>0.4) {
                    optimize::Odometry odom_tmp;
                    odom_tmp.time = it->getTime().toSec();

                    // world frame to body frame
                    Eigen::Matrix<double, 3, 3> rotation;
                    rotation(0,0) = 1-2*std::pow(odom_tmp.q_y, 2) - 2*std::pow(odom_tmp.q_z, 2);
                    rotation(0,1) = 2*odom_tmp.q_x*odom_tmp.q_y-2*odom_tmp.q_z*odom_tmp.q_w;
                    rotation(0,2) = 2*odom_tmp.q_x*odom_tmp.q_z+2*odom_tmp.q_y*odom_tmp.q_w;
                    rotation(1,0) = 2*odom_tmp.q_x*odom_tmp.q_y+2*odom_tmp.q_z*odom_tmp.q_w;
                    rotation(1,1) = 1-2*std::pow(odom_tmp.q_x,2)-2*std::pow(odom_tmp.q_z,2);
                    rotation(1,2) = 2*odom_tmp.q_y*odom_tmp.q_z-2*odom_tmp.q_x*odom_tmp.q_w;
                    rotation(2,0) = 2*odom_tmp.q_x*odom_tmp.q_z-2*odom_tmp.q_y*odom_tmp.q_w;
                    rotation(2,1) = 2*odom_tmp.q_y*odom_tmp.q_z+2*odom_tmp.q_x*odom_tmp.q_w;
                    rotation(2,2) = 1-2*std::pow(odom_tmp.q_x, 2) - 2*std::pow(odom_tmp.q_y, 2);
                    //rotation = rotation.inverse();

                    odom_tmp.x = odom_ptr_->pose.pose.position.x;
                    odom_tmp.y = odom_ptr_->pose.pose.position.y;
                    odom_tmp.z = odom_ptr_->pose.pose.position.z;
                    odom_tmp.q_x = odom_ptr_->pose.pose.orientation.x;
                    odom_tmp.q_y = odom_ptr_->pose.pose.orientation.y;
                    odom_tmp.q_z = odom_ptr_->pose.pose.orientation.z;
                    odom_tmp.q_w = odom_ptr_->pose.pose.orientation.w;
                    odom_tmp.v_x = odom_ptr_->twist.twist.linear.x;
                    odom_tmp.v_y = odom_ptr_->twist.twist.linear.y;
                    odom_tmp.v_z = odom_ptr_->twist.twist.linear.z;

                    Eigen::Matrix<double, 3, 1> omega_origin{odom_ptr_->twist.twist.angular.x, odom_ptr_->twist.twist.angular.y, odom_ptr_->twist.twist.angular.z};
                    odom_tmp.omega_x = omega_origin(0,0);
                    odom_tmp.omega_y = omega_origin(1,0);
                    odom_tmp.omega_z = omega_origin(2,0);
                    //std::cout<<"push odom at "<<it->getTime()<<", z="<<odom_ptr_->pose.pose.position.z<<std::endl;
                    std::advance(it, 3);
                    quadrotor_msgs::ControlCommand::ConstPtr command_ptr_ = it->instantiate<quadrotor_msgs::ControlCommand>();
                    optimize::Command command_tmp;
                    command_tmp.time = it->getTime().toSec();
                    command_tmp.armed = command_ptr_->armed;
                    command_tmp.control_mode = command_ptr_->control_mode;
                    command_tmp.ct = command_ptr_->collective_thrust;

                    omega_origin = Eigen::Matrix<double, 3, 1>(command_ptr_->bodyrates.x, command_ptr_->bodyrates.y, command_ptr_->bodyrates.z);

                    // if(config_.bodyrates_in_world) {
                    //     // std::cout<<"before rotation: "<< omega_origin<<std::endl;
                    //     // std::cout<<"rotation: \n"<<rotation<<std::endl<<std::endl;
                    //     omega_origin = rotation*omega_origin;
                    //     // std::cout<<"after rotation: "<< omega_origin<<std::endl<<std::endl;
                    // }

                    command_tmp.omega_x = omega_origin(0,0);
                    command_tmp.omega_y = omega_origin(1,0);
                    command_tmp.omega_z = omega_origin(2,0);

                    // command_tmp.omega_x = command_ptr_->bodyrates.x;
                    // command_tmp.omega_y = command_ptr_->bodyrates.y;
                    // command_tmp.omega_z = command_ptr_->bodyrates.z;
                    //std::cout<<"push command at "<<it->getTime()<<std::endl<<std::endl;
                    std::pair<optimize::Odometry, optimize::Command> pair_tmp(odom_tmp, command_tmp);
                    if(pairs_.size()==0) {
                        pairs_.push_back(pair_tmp);
                    } else if (pair_tmp.first.time-pairs_.back().first.time>0.01){
                        pairs_.push_back(pair_tmp);
                    }
                }
            }
            it++;
        }
        // cal time
        std::vector<double> time_gap, time_step;
        for(int i=0;i<pairs_.size()-1;i++) {
            time_gap.push_back(pairs_[i].second.time-pairs_[i].first.time);
            time_step.push_back(pairs_[i+1].first.time-pairs_[i].first.time);
        }
        auto res = mean_and_var(time_gap);
        std::cout<<"time_gap: mean: "<<res.first<<std::endl
                 <<"          std:  "<<res.second<<std::endl;
        res = mean_and_var(time_step);
        std::cout<<"time_step: mean: "<<res.first<<std::endl
                 <<"          std:  "<<res.second<<std::endl;
        // ros::Duration(10.0).sleep();
        bag.close();
        return true;
    }

    bool Optimizer::read_feedback() {
        rosbag::Bag bag;
        bag.open(config_.bag_path, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(config_.feedback_topic);
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        auto it = std::next(view.begin(), 500);
        double time_start = it->getTime().toSec();
        while(it!=view.end()) {
            //std::cout<<" at "<<it->getTime().sec<<"'"<<it->getTime().nsec<<std::endl;
            bool valid = (it->getTopic() == config_.feedback_topic);
            if(valid) {
                quadrotor_msgs::AutopilotFeedback::ConstPtr fb_ptr_ = it->instantiate<quadrotor_msgs::AutopilotFeedback>();
                optimize::Odometry odom_tmp;
                odom_tmp.time = fb_ptr_->header.stamp.toSec()-time_start;

                // world frame to body frame
                Eigen::Matrix<double, 3, 3> rotation;
                rotation(0,0) = 1-2*std::pow(odom_tmp.q_y, 2) - 2*std::pow(odom_tmp.q_z, 2);
                rotation(0,1) = 2*odom_tmp.q_x*odom_tmp.q_y-2*odom_tmp.q_z*odom_tmp.q_w;
                rotation(0,2) = 2*odom_tmp.q_x*odom_tmp.q_z+2*odom_tmp.q_y*odom_tmp.q_w;
                rotation(1,0) = 2*odom_tmp.q_x*odom_tmp.q_y+2*odom_tmp.q_z*odom_tmp.q_w;
                rotation(1,1) = 1-2*std::pow(odom_tmp.q_x,2)-2*std::pow(odom_tmp.q_z,2);
                rotation(1,2) = 2*odom_tmp.q_y*odom_tmp.q_z-2*odom_tmp.q_x*odom_tmp.q_w;
                rotation(2,0) = 2*odom_tmp.q_x*odom_tmp.q_z-2*odom_tmp.q_y*odom_tmp.q_w;
                rotation(2,1) = 2*odom_tmp.q_y*odom_tmp.q_z+2*odom_tmp.q_x*odom_tmp.q_w;
                rotation(2,2) = 1-2*std::pow(odom_tmp.q_x, 2) - 2*std::pow(odom_tmp.q_y, 2);
                //rotation = rotation.inverse();

                odom_tmp.x = fb_ptr_->state_estimate.pose.pose.position.x;
                odom_tmp.y = fb_ptr_->state_estimate.pose.pose.position.y;
                odom_tmp.z = fb_ptr_->state_estimate.pose.pose.position.z;
                odom_tmp.q_x = fb_ptr_->state_estimate.pose.pose.orientation.x;
                odom_tmp.q_y = fb_ptr_->state_estimate.pose.pose.orientation.y;
                odom_tmp.q_z = fb_ptr_->state_estimate.pose.pose.orientation.z;
                odom_tmp.q_w = fb_ptr_->state_estimate.pose.pose.orientation.w;
                odom_tmp.v_x = fb_ptr_->state_estimate.twist.twist.linear.x;
                odom_tmp.v_y = fb_ptr_->state_estimate.twist.twist.linear.y;
                odom_tmp.v_z = fb_ptr_->state_estimate.twist.twist.linear.z;

                odom_tmp.omega_x = fb_ptr_->state_estimate.twist.twist.angular.x;
                odom_tmp.omega_y = fb_ptr_->state_estimate.twist.twist.angular.y;
                odom_tmp.omega_z = fb_ptr_->state_estimate.twist.twist.angular.z;
            
                optimize::Command command_tmp;
                command_tmp.time = fb_ptr_->header.stamp.toSec()-time_start;
                command_tmp.armed = fb_ptr_->control_command.armed;
                command_tmp.control_mode = fb_ptr_->control_command.control_mode;
                command_tmp.ct = fb_ptr_->control_command.collective_thrust;

                command_tmp.omega_x = fb_ptr_->control_command.bodyrates.x;
                command_tmp.omega_y = fb_ptr_->control_command.bodyrates.y;
                command_tmp.omega_z = fb_ptr_->control_command.bodyrates.z;

                //std::cout<<"---------------\ntime "<<odom_tmp.time<<": \n"<<odom_tmp<<command_tmp<<std::endl;

                std::pair<optimize::Odometry, optimize::Command> pair_tmp(odom_tmp, command_tmp);
                pairs_.push_back(pair_tmp);
            }
            it++;
        }
        // cal time
        std::vector<double> time_gap, time_step;
        for(int i=0;i<pairs_.size()-1;i++) {
            time_gap.push_back(pairs_[i].second.time-pairs_[i].first.time);
            std::cout<<"time step: "<<pairs_[i+1].first.time-pairs_[i].first.time<<std::endl;
            time_step.push_back(pairs_[i+1].first.time-pairs_[i].first.time);
        }
        auto res = mean_and_var(time_gap);
        std::cout<<"time_gap: mean: "<<res.first<<std::endl
                 <<"          std:  "<<res.second<<std::endl;
        res = mean_and_var(time_step);
        std::cout<<"time_step: mean: "<<res.first<<std::endl
                 <<"          std:  "<<res.second<<std::endl;
        // ros::Duration(10.0).sleep();
        bag.close();
        return true;
    }

    bool Optimizer::get_config() {
        if(!pnh_.getParam("bag_path", config_.bag_path)) {
            std::cout<<"no bag path!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("odom_topic", config_.odom_topic)) {
            std::cout<<"no odom topic!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("command_topic", config_.command_topic)) {
            std::cout<<"no command topic!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("feedback_topic", config_.feedback_topic)) {
            std::cout<<"no fb topic!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("default_param", config_.default_param)) {
            std::cout<<"no default_param path!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("target_param", config_.target_param)) {
            std::cout<<"no target_param path!"<<std::endl;
            return false;
        }
        if(!pnh_.getParam("result_path", config_.res_path)) {
            std::cout<<"no result path!"<<std::endl;
            return false;
        }
        return true;
    }

    bool Optimizer::test_onestep(std::string mode) {
        auto quad = flightlib::Quadrotor("/home/ysa/workspace/flightmare/flightmare/flightlib/configs/quadrotor_env.yaml");
        std::cout<<"pairs: "<<pairs_.size()<<std::endl;
        std::vector<double> error_x, error_q, error_v, error_omega, change_x, change_q, change_v, change_omega;

        double param[flightlib::OptParam::N_Param];

        if(mode == "default") {
            read_yaml(config_.default_param, param);
        } else if(mode =="target") {
            read_yaml(config_.target_param, param);
        } else {
            std::cout<<"invalid test mode"<<std::endl;
            return false;
        }

        std::ofstream res_file(config_.res_path+"/onestep.csv");

        for(int i=0;i<pairs_.size()-1;i++) {
            optimize::QuadCost test_cost(pairs_[i].first, pairs_[i].second, pairs_[i+1].first, &quad);

            double residual[8];
            auto odom_output = test_cost.eval(param, residual);
            error_x.push_back(residual[0]);
            error_v.push_back(residual[1]);
            error_q.push_back(residual[2]);
            error_omega.push_back(residual[3]);
            change_x.push_back(residual[4]);
            change_v.push_back(residual[5]);
            change_q.push_back(residual[6]);
            change_omega.push_back(residual[7]);
            res_file<<pairs_[i+1].first.x<<", "<<pairs_[i+1].first.y<<", "<<pairs_[i+1].first.z<<", "<<odom_output.x<<", "<<odom_output.y<<", "<<odom_output.z<<std::endl;
        }
        // cal
        auto res = mean_and_var(error_x);
        std::cout<<"x error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_v);
        std::cout<<"v error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_q);
        std::cout<<"q error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_omega);
        std::cout<<"omega error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        std::cout<<"abs relative x error: "<<std::accumulate(error_x.begin(), error_x.end(), 0.0) / std::accumulate(change_x.begin(), change_x.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative v error: "<<std::accumulate(error_v.begin(), error_v.end(), 0.0) / std::accumulate(change_v.begin(), change_v.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative q error: "<<std::accumulate(error_q.begin(), error_q.end(), 0.0) / std::accumulate(change_q.begin(), change_q.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative omega error: "<<std::accumulate(error_omega.begin(), error_omega.end(), 0.0) / std::accumulate(change_omega.begin(), change_omega.end(), 0.0)*100.0<<"% "<<std::endl;
        return true;
    }

    bool Optimizer::test_boostrap(std::string mode) {
        auto quad = flightlib::Quadrotor("/home/ysa/workspace/flightmare/flightmare/flightlib/configs/quadrotor_env.yaml");
        std::cout<<"pairs: "<<pairs_.size()<<std::endl;
        std::vector<double> error_x, error_q, error_v, error_omega, change_x, change_q, change_v, change_omega;

        double param[flightlib::OptParam::N_Param];

        if(mode == "default") {
            read_yaml(config_.default_param, param);
        } else if(mode =="target") {
            read_yaml(config_.target_param, param);
        } else {
            std::cout<<"invalid test mode"<<std::endl;
            return false;
        }
        
        optimize::Odometry odometry_last = pairs_.begin()->first;
        std::ofstream res_file(config_.res_path+"/boostrap.csv");

        for(int i=0;i<pairs_.size()-1;i++) {
            optimize::QuadCost test_cost(odometry_last, pairs_[i].second, pairs_[i+1].first, &quad);
            std::cout<<"ct: "<<pairs_[i].second.ct<<std::endl;

            double residual[8];
            odometry_last = test_cost.eval(param, residual);
            odometry_last.time = pairs_[i+1].first.time;
            error_x.push_back(residual[0]);
            error_v.push_back(residual[1]);
            error_q.push_back(residual[2]);
            error_omega.push_back(residual[3]);
            change_x.push_back(residual[4]);
            change_v.push_back(residual[5]);
            change_q.push_back(residual[6]);
            change_omega.push_back(residual[7]);
            res_file<<pairs_[i+1].first.x<<", "<<pairs_[i+1].first.y<<", "<<pairs_[i+1].first.z<<", "<<odometry_last.x<<", "<<odometry_last.y<<", "<<odometry_last.z<<std::endl;
        }
        // cal
        auto res = mean_and_var(error_x);
        std::cout<<"x error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_v);
        std::cout<<"v error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_q);
        std::cout<<"q error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        res = mean_and_var(error_omega);
        std::cout<<"omega error: mean: "<<res.first<<std::endl
                 <<"         std:  "<<res.second<<std::endl;
        std::cout<<"abs relative x error: "<<std::accumulate(error_x.begin(), error_x.end(), 0.0) / std::accumulate(change_x.begin(), change_x.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative v error: "<<std::accumulate(error_v.begin(), error_v.end(), 0.0) / std::accumulate(change_v.begin(), change_v.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative q error: "<<std::accumulate(error_q.begin(), error_q.end(), 0.0) / std::accumulate(change_q.begin(), change_q.end(), 0.0)*100.0<<"% "<<std::endl;
        std::cout<<"abs relative omega error: "<<std::accumulate(error_omega.begin(), error_omega.end(), 0.0) / std::accumulate(change_omega.begin(), change_omega.end(), 0.0)*100.0<<"% "<<std::endl;

        return true;
    }

    bool Optimizer::optimize() {
        auto quad = flightlib::Quadrotor("/home/ysa/workspace/flightmare/flightmare/flightlib/configs/quadrotor_env.yaml");
        std::cout<<"pairs: "<<pairs_.size()<<std::endl;
        //init problem
        ceres::Problem problem;
        std::cout<<"create problem done"<<std::endl;

        // set param_default
        double param[flightlib::OptParam::N_Param];
        read_yaml(config_.default_param, param);

        double param_init[flightlib::OptParam::N_Param];
        std::string param_name[flightlib::OptParam::N_Param] = {"mass", "arm length", "motor tau", "thrust mapping 0", "thrust mapping 1", "thrust mapping 2", "kappa", "J(0,0)", "J(0,1)", "J(0,2)", "J(1,1)", "J(1,2)", "J(2,2)", "roll gain", "pitch gain", "yaw gain"};
        for(int i=0;i<flightlib::OptParam::N_Param;i++) {
            param_init[i] = param[i];
        }

        // add residual block
        for(int i=0;i<pairs_.size()-1;i++) {
            ceres::CostFunction* cost_func = new ceres::NumericDiffCostFunction<optimize::QuadCost, ceres::CENTRAL, 4, flightlib::OptParam::N_Param>(new optimize::QuadCost(pairs_[i].first, pairs_[i].second, pairs_[i+1].first, &quad));

            //add block
            problem.AddResidualBlock(cost_func, nullptr, param);
        }
        std::cout<<"add residual block done"<<std::endl;

        problem.SetParameterLowerBound(param, 2, 1e-7);
        ceres::Solver::Options option;
        option.max_num_iterations = 500;
        ceres::Solver::Summary summary;
        ceres::Solve(option, &problem, &summary);
        std::cout << summary.FullReport() << std::endl;
        write_yaml(config_.target_param, param);
    }

    template<typename T>
    bool Optimizer::read_yaml(const std::string path, T* const param) {
        std::ifstream file(path);
        YAML::Node node = YAML::Load(file);
        file.close();
        std::cout<<"reading node: \n"<<node<<"  from: "<<path<<std::endl;
        param[flightlib::OptParam::mass] = node["mass"].as<T>();
        param[flightlib::OptParam::arm_l] = node["arm_l"].as<T>();
        param[flightlib::OptParam::motor_tau] = node["motor_tau"].as<T>();
        param[flightlib::OptParam::thrust_map] = node["thrust_map"][0].as<T>();
        param[flightlib::OptParam::thrust_map+1] =node["thrust_map"][1].as<T>();
        param[flightlib::OptParam::thrust_map+2] = node["thrust_map"][2].as<T>();
        param[flightlib::OptParam::kappa] = node["kappa"].as<T>();
        param[flightlib::OptParam::J] = node["J"][0].as<T>();
        param[flightlib::OptParam::J+1] = node["J"][1].as<T>();
        param[flightlib::OptParam::J+2] = node["J"][2].as<T>();
        param[flightlib::OptParam::J+3] = node["J"][3].as<T>();
        param[flightlib::OptParam::J+4] = node["J"][4].as<T>();
        param[flightlib::OptParam::J+5] = node["J"][5].as<T>();
        param[flightlib::OptParam::P_gain] = node["P_gain"][0].as<T>();
        param[flightlib::OptParam::P_gain+1] = node["P_gain"][1].as<T>();
        param[flightlib::OptParam::P_gain+2] = node["P_gain"][2].as<T>();
        param[flightlib::OptParam::G_equal] = node["Gz"].as<T>();
        return true;
    }

    template<typename T>
    bool Optimizer::write_yaml(const std::string path, T* const param) {
        std::ofstream file(path);
        YAML::Node node;
        node["Gz"] = param[flightlib::OptParam::G_equal];
        node["mass"] = param[flightlib::OptParam::mass];
        node["arm_l"] = param[flightlib::OptParam::arm_l];
        node["motor_tau"] = param[flightlib::OptParam::motor_tau];
        node["thrust_map"].push_back(param[flightlib::OptParam::thrust_map]);
        node["thrust_map"].push_back(param[flightlib::OptParam::thrust_map+1]);
        node["thrust_map"].push_back(param[flightlib::OptParam::thrust_map+2]);
        node["kappa"] = param[flightlib::OptParam::kappa];
        node["J"].push_back(param[flightlib::OptParam::J]);
        node["J"].push_back(param[flightlib::OptParam::J+1]);
        node["J"].push_back(param[flightlib::OptParam::J+2]);
        node["J"].push_back(param[flightlib::OptParam::J+3]);
        node["J"].push_back(param[flightlib::OptParam::J+4]);
        node["J"].push_back(param[flightlib::OptParam::J+5]);
        node["P_gain"].push_back(param[flightlib::OptParam::P_gain]);
        node["P_gain"].push_back(param[flightlib::OptParam::P_gain+1]);
        node["P_gain"].push_back(param[flightlib::OptParam::P_gain+2]);
        file<<node;
        std::cout<<"writing node: \n"<<node<<"  to: "<<path<<std::endl;
        file.close();
        return true;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "optimizer");
    std::cout<<"start read"<<std::endl;
    optimize::Optimizer optimizer(ros::NodeHandle(), ros::NodeHandle("~"));
    optimizer.read_feedback();
    std::cout<<"Start Collecting data"<<std::endl;
    //optimizer.optimize();
    auto begin = ros::Time::now();
    optimizer.test_onestep("default");
    auto end = ros::Time::now();
    //optimizer.test_boostrap("target");
    std::cout<<"Collecting data done!"<<std::endl;
    std::cout<<"Time consuming: "<<(end-begin).toSec()<<std::endl;
    return 0;
}