#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::FOREST_LOW),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.08, 0, 0.08); // right, forward, up
  Matrix<3, 3> R_BC = Quaternion(0.70711, 0.0, 0.0, -0.70711).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(75);
  rgb_camera_->setWidth(320);
  rgb_camera_->setHeight(240);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{enable_depth_, enable_segmentation_, enable_depth_});  // depth, segmentation, optical flow
  if(enable_rgb_) {
    quad_ptr_->addRGBCamera(rgb_camera_);
  }

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);
  
  //image pubs
  image_transport::ImageTransport it(nh_);
  rgb_pub_ = it.advertise("flight_pilot/rgb", 1);
  depth_pub_ = it.advertise("flight_pilot/depth", 1);
  segmentation_pub_ = it.advertise("flight_pilot/segmentation", 1);
  opticalflow_pub_ = it.advertise("flight_pilot/opticalflow", 1);

  if_start_pub_ = nh_.advertise<std_msgs::Float32>("flight_pilot/if_start", 10);
  if_end_pub_ = nh_.advertise<std_msgs::Float32>("flight_pilot/if_end", 10);
  if_collision_pub_ = nh_.advertise<std_msgs::Bool>("flight_pilot/if_collision", 10);

  // wait until the gazebo and unity are loaded
  ros::Duration(3.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;

  // Quaternion turn, temp;
  // Quaternion p = Quaternion(0,0,0,1);
  // temp = quad_state_.q() * p * quad_state_.q().inverse();
  // turn = Quaternion(0.70711, -0.70711*temp.x(), -0.70711*temp.y(), -0.70711*temp.z());
  // Quaternion final = turn*Quaternion((Scalar)msg->pose.pose.orientation.w, (Scalar)msg->pose.pose.orientation.x, (Scalar)msg->pose.pose.orientation.y, (Scalar)msg->pose.pose.orientation.z);
  // quad_state_.x[QS::ATTW] = (Scalar)final.w();
  // quad_state_.x[QS::ATTX] = (Scalar)final.x();
  // quad_state_.x[QS::ATTY] = (Scalar)final.y();
  // quad_state_.x[QS::ATTZ] = (Scalar)final.z();
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    std_msgs::Bool collision_msg;
    if (quad_ptr_->getCollision()) {
      // collision happened
      collision_msg.data = true;
      //ROS_INFO("COLLISION");
    } else {
      collision_msg.data = false;
    }
    if_collision_pub_.publish(collision_msg);
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // pub images
  ros::Time timestamp = ros::Time::now();
  cv::Mat img_get;
  if(enable_rgb_) {
    rgb_camera_->getRGBImage(img_get);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_get).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub_.publish(rgb_msg);
    if(enable_depth_) {
      rgb_camera_->getDepthMap(img_get);
      sensor_msgs::ImagePtr depth_msg =
        cv_bridge::CvImage(std_msgs::Header(), "32FC1", img_get).toImageMsg();
      depth_msg->header.stamp = timestamp;
      depth_pub_.publish(depth_msg);
    }
    if(enable_segmentation_) {
      rgb_camera_->getSegmentation(img_get);
      sensor_msgs::ImagePtr segmentation_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_get).toImageMsg();
      segmentation_msg->header.stamp = timestamp;
      segmentation_pub_.publish(segmentation_msg);
    }
    if(enable_opticalflow_) {
      rgb_camera_->getOpticalFlow(img_get);
      sensor_msgs::ImagePtr opticflow_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_get).toImageMsg();
      opticflow_msg->header.stamp = timestamp;
      opticalflow_pub_.publish(opticflow_msg);
    }
  }

  std_msgs::Float32 start_msg;
  start_msg.data = std::sqrt(std::pow(quad_state_.x[QS::POSX]-start_x_, 2)+std::pow(quad_state_.x[QS::POSY]-start_y_, 2));
  if_start_pub_.publish(start_msg);

  std_msgs::Float32 end_msg;
  end_msg.data = std::sqrt(std::pow(quad_state_.x[QS::POSX]-end_x_, 2)+std::pow(quad_state_.x[QS::POSY]-end_y_, 2) + std::pow(quad_state_.x[QS::POSZ]-end_z_, 2));
  if_end_pub_.publish(end_msg);

}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);
  int scene;
  quadrotor_common::getParam("scene_id", scene, pnh_);
  scene_id_ = flightlib::SceneID(scene);
  std::cout<<"scene id: "<<scene_id_<<std::endl;
  quadrotor_common::getParam("enable_rgb", enable_rgb_, pnh_);
  quadrotor_common::getParam("enable_depth", enable_depth_, pnh_);
  quadrotor_common::getParam("enable_segmentation", enable_segmentation_, pnh_);
  quadrotor_common::getParam("enable_opticalflow", enable_opticalflow_, pnh_);
  quadrotor_common::getParam("start_x", start_x_, pnh_);
  quadrotor_common::getParam("start_y", start_y_, pnh_);
  quadrotor_common::getParam("end_x", end_x_, pnh_);
  quadrotor_common::getParam("end_y", end_y_, pnh_);
  quadrotor_common::getParam("end_z", end_z_, pnh_);

  return true;
}

}  // namespace flightros