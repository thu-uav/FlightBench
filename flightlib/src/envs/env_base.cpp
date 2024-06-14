#include "flightlib/envs/env_base.hpp"

namespace flightlib {

EnvBase::EnvBase() : obs_dim_(0), act_dim_(0), sim_dt_(0.0) {}

EnvBase::~EnvBase() {}

void EnvBase::curriculumUpdate() {}

void EnvBase::close() {}

void EnvBase::render() {}

void EnvBase::updateExtraInfo() {}

bool EnvBase::isTerminalState() {
  return false;
}

}  // namespace flightlib
