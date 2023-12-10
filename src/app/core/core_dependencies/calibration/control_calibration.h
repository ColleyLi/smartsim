// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "core/calibration/interpolation_2d.h"
#include "third_party/smartsim/proto/control/control_conf.pb.h"
#include "core/common/local_view.h"
#include "common/proto/control/carsim_control_cmd.pb.h"

namespace smartsim {
namespace core {
namespace control {

using smartsim::control::ControlConf;
using smartsim::control::LonControllerConf;

class control_calibration {
  public:
    bool Init(const ControlConf *control_conf);
    bool run(
      Chassis chassis_,
      ControlCommand control_cmd_);
  
  public:
    void LoadControlCalibrationTable(const LonControllerConf &lon_control_conf);
    double acceleration = 0.0;
    double throttle = 0.0;
    double brake = 0.0;
    double speed = 0.0;
    ::drivers::Chassis_GearPosition gear;
    

  private:
    LonControllerConf lon_control_conf_;
    const ControlConf *control_conf_ = nullptr;
    std::unique_ptr<Interpolation2D> control_interpolation_;

};


} // namespace control
} // namespace core
} // namespace smartsim