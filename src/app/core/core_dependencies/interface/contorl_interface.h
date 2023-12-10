//Copyright 2023 The XUCONG Authors. All Rights Reserved.
#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

#include "core/common/core_gflags.h"
#include "proto/core/sim_control.pb.h"
#include "submodule/third_party/smartsim/proto/control/control_cmd.pb.h"

namespace smartsim {
namespace core {
namespace control {
/**
 * @class VehControlInterface
 *
 * @brief Interface of the control module
 */
class VehControlInterface {
  public:
    /**
     * @brief main logic of the canbus module
     */
    virtual void RunOnce() = 0;

    /**
     * @brief Fill the header.
     */
    void FillHeader(SimControl *sim_control) {
        auto *header = sim_control->mutable_header();
        double timestamp = ros::Time::now().toSec();
        header->set_module_name(FLAGS_vehicle_control_module_name);
        header->set_timestamp_sec(timestamp);
        header->set_sequence_num(++seq_num_);
    }

    /**
     * @brief Serialize all descriptors of the given message to string.
     */
    void GetDescriptorString(const SimControl &sim_control, std_msgs::String message) {
        if (!sim_control.SerializeToString(&message.data)) {
            ROS _ERROR("failed to serialize!");
        }
    }

    /**
     * @brief Convert the serialized FileDescriptorProto to real descriptors.
     */
    bool RegisterMessage(const std_msgs::String &message, smartsim::control::ControlCommand control_cmd) {
        if (!control_cmd.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

    /**
     * @brief Convert the serialized FileDescriptorProto to real descriptors.
     */
    bool RegisterMessage(const std_msgs::String &message, smartsim::drivers::Chassis chassis) {
        if (!chassis.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

  private:
    uint32_t seq_num_ = 0;
};

} // namespace control
} // namespace core
} // namespace smartsim
#endif // CONTROL INTERFACE H
