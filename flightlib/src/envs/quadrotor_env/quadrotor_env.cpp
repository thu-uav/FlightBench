#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml"),
                 getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/dyn_param.yaml")
                 ) {}

QuadrotorEnv::QuadrotorEnv(const std::string &env_cfg_path, const std::string &dyn_cfg_path)
  : EnvBase() {
  // load configuration file

  quadrotor_ptr_ = std::make_shared<Quadrotor>(dyn_cfg_path);

  // define input and output dimension for the environment
  obs_dim_ = quadenv::kNObs;
  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  // do not use srt now
  srt_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  srt_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;
  omega_max_ = quadrotor_ptr_->getDynamics().omega_max();

  // load self parameters
  loadParam(env_cfg_path);
  disx_ = std::normal_distribution<double>(0, sigma_x_);
  disy_ = std::normal_distribution<double>(0, sigma_y_);
  disz_ = std::normal_distribution<double>(0, sigma_z_);
  //addRGBMono(); // just for test camera
}

QuadrotorEnv::~QuadrotorEnv() {}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  if (random) {
    // randomly reset the quadrotor state
    // reset position
    quad_state_.x(QS::POSX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSZ) = uniform_dist_(random_gen_) + 1.5;
    if (quad_state_.x(QS::POSZ) < -0.0)
      quad_state_.x(QS::POSZ) = -quad_state_.x(QS::POSZ);
    // reset linear velocity
    quad_state_.x(QS::VELX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELZ) = uniform_dist_(random_gen_);
    // reset orientation
    quad_state_.x(QS::ATTW) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTZ) = uniform_dist_(random_gen_);
    quad_state_.qx /= quad_state_.qx.norm();
  } else {
    // follow obs to set
    //printf("reset, obs: %f, %f, %f, %f, %f, %f, %f", obs(0), obs(1), obs(2), obs(3), obs(4), obs(5), obs(6));
    quad_state_.x(QS::POSX) = obs(quadenv::kPos);
    quad_state_.x(QS::POSY) = obs(quadenv::kPos+1);
    quad_state_.x(QS::POSZ) = obs(quadenv::kPos+2);
    quad_state_.x(QS::ATTW) = obs(quadenv::kOri);
    quad_state_.x(QS::ATTX) = obs(quadenv::kOri+1);
    quad_state_.x(QS::ATTY) = obs(quadenv::kOri+2);
    quad_state_.x(QS::ATTZ) = obs(quadenv::kOri+3);
    quad_state_.qx /= quad_state_.qx.norm();
  }
  // reset quadrotor with random states
  quadrotor_ptr_->reset(quad_state_);

  // reset control command

  if(if_ctbr_) {
    cmd_.t = 0.0;
    cmd_.collective_thrust = 0;
    cmd_.omega.setZero();
  } else {
    cmd_.t = 0.0;
    cmd_.thrusts.setZero();
  }

  // obtain observations
  getObs(obs);
  return true;
}

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  quadrotor_ptr_->getState(&quad_state_);

  // convert quaternion to euler angle
  // Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
  Vector<4> rotation;
  rotation(0) = quad_state_.q().w(); // wxyz
  rotation(1) = quad_state_.q().x(); // wxyz
  rotation(2) = quad_state_.q().y(); // wxyz
  rotation(3) = quad_state_.q().z(); // wxyz
  // quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, rotation, quad_state_.v, quad_state_.w;

  obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
  return true;
}

bool QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  // act all in [-1,1]
  if(if_ctbr_) {
    //printf("ctbr!\n");
    quad_act_ = act.cwiseProduct(omega_max_);
    quad_act_(0) = (act(0)+1)* acc_max_/ 2;
    cmd_.t += sim_dt_;
    cmd_.collective_thrust = (act(0)+1)* acc_max_/ 2;
    // ct: desired a, in m/s^2
    cmd_.omega = (act.bottomRows(3)).cwiseProduct(omega_max_);
    // br: in rad/s
    //std::cout<<"ct: "<<act(0)<<" -> "<<cmd_.collective_thrust<<std::endl;
    //std::cout<<"br: "<<cmd_.omega<<std::endl;
  } else {
    quad_act_ = act.cwiseProduct(srt_std_) + srt_mean_;
    cmd_.t += sim_dt_;
    cmd_.thrusts = quad_act_;
  }

  // simulate quadrotor
  quadrotor_ptr_->run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  // Matrix<3, 3> rot = quad_state_.q().toRotationMatrix();

  return true;
}

bool QuadrotorEnv::isTerminalState() {
  if (quad_state_.x(QS::POSZ) <= -0.1) {
    return true;
  }
  return false;
}

bool QuadrotorEnv::loadParam(const std::string cfg_path) {
  std::ifstream file(cfg_path);
  YAML::Node cfg = YAML::Load(file);
  if (cfg["quadrotor_env"]) {
    sim_dt_ = cfg["quadrotor_env"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["quadrotor_env"]["max_t"].as<Scalar>();
    sigma_x_ = cfg["quadrotor_env"]["sigma_x"].as<double>();
    sigma_y_ = cfg["quadrotor_env"]["sigma_y"].as<double>();
    sigma_z_ = cfg["quadrotor_env"]["sigma_z"].as<double>();
  } else {
    return false;
  }

  if (cfg["control"]) {
    // load reinforcement learning related parameters
    if_ctbr_ = cfg["control"]["ctbr"].as<bool>();
    acc_max_ = cfg["control"]["acc_max"].as<Scalar>();
  } else {
    return false;
  }
  return true;
}

bool QuadrotorEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorEnv::randomizeKv() {
  double kvx = disx_(gen_);
  double kvy = disy_(gen_);
  double kvz = disz_(gen_);
  // printf("kvx: %f, kvy: %f, kvz: %f\n", kvx, kvy, kvz);
  quadrotor_ptr_->randomizeKv(kvx, kvy, kvz);
}

void QuadrotorEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

void QuadrotorEnv::addRGBDMono() {
  std::shared_ptr<RGBCamera> camera = std::make_shared<RGBCamera>();
  Eigen::Vector3f B_r_BC;
  B_r_BC << 0.08,0,0.08;
  Eigen::Matrix3f R_BC = Quaternion(0.70711, 0.0, 0.0, -0.70711).toRotationMatrix();
  camera->setFOV(90);
  camera->setWidth(320);
  camera->setHeight(240);
  camera->setRelPose(B_r_BC,R_BC);
  camera->enableDepth(true);
  quadrotor_ptr_->addRGBCamera(camera);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorEnv &quad_env) {
  os.precision(3);
  os << "Quadrotor Environment:\n"
     << "obs dim =            [" << quad_env.obs_dim_ << "]\n"
     << "act dim =            [" << quad_env.act_dim_ << "]\n"
     << "sim dt =             [" << quad_env.sim_dt_ << "]\n"
     << "max_t =              [" << quad_env.max_t_ << "]\n"
     << "act_mean =           [" << quad_env.srt_mean_.transpose() << "]\n"
     << "act_std =            [" << quad_env.srt_std_.transpose() << "]\n"
     << "obs_mean =           [" << quad_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << quad_env.obs_std_.transpose() << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib