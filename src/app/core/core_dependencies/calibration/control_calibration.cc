// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "core/calibration/control_calibration.h"

#include "algorithm"
#include "sstream"
#include "utility"

#include "common/logging/log.h"
#include "core/common/core_gflags.h"
#include "core/common/status.h"


namespace smartsim {
namespace core {
namespace control {

bool control_calibration::Init(const ControlConf *control_conf) {
    CINFO << "control calibration Init, starting...";
    
    control_conf_ = control_conf;
    if (control_conf_ == nullptr) {
        CERROR << "et_longitudinal_param() nullptr";
        return false;
    }
    const LonControllerConf &lon_controller_conf = 
        control_conf_->lon_controller_conf();
    
    LoadControlCalibrationTable(lon_controller_conf);
}



void control_calibration::LoadControlCalibrationTable(const LonControllerConf &lon_control_conf) {
    

    const auto &control_table = lon_control_conf.calibration_table();
    CINFO << "control calibration table size: " << control_table.calibration_size();
    Interpolation2D::DataType xyz;
    for (const auto &calibration : control_table.calibration()) {
        xyz.push_back(std::make_tuple(calibration.speed(),
                                      calibration.acceleration(),
                                      calibration.command()));
        CERROR << "calibration table: "
               << "speed: " << calibration.speed()
               << ", acceleration: " << calibration.acceleration()
               << ", command: " << calibration.command();
    }
    control_interpolation_.reset(new Interpolation2D);
    assert(control_interpolation_->Init(xyz));

}


bool control_calibration::run(Chassis chassis_, ControlCommand control_cmd_) {
    double throttle_lowbound = 
        std::max(15.7, lon_control_conf_.throttle_minimum_action());
    double brake_lowbound = 
        std::max(14.5, lon_control_conf_.brake_minimum_action());
    double calibration_value = 0.0;
    double acceleration_lookup = 
        (gear == chassis_.GEAR_REVERSE)
         ? -control_cmd_.acceleration()
         :  control_cmd_.acceleration();
    calibration_value = control_interpolation_->Interpolate(
        std::make_pair(chassis_.speed_mps(), acceleration_lookup));
    
    if (acceleration_lookup >=0) {
        if (calibration_value >=0) {
            throttle = std::max(calibration_value, throttle_lowbound);
        } else {
            throttle_lowbound = throttle_lowbound;
        }
        brake = 0.0;
    } else {
        throttle = 0.0;
        if (calibration_value >=0) {
            brake = brake_lowbound;
        } else {
            brake = std::max(-calibration_value, brake_lowbound);
        }
    }

}

} // namespace control
} // namespace core
} // namespace smartsim