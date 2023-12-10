//Copyright 2023 The XUCONG Authors. All Rights Reserved.
#ifndef CANBUS_INTERFACE_H
#define CANBUS_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

#include "core/common/core_gflags.h"
#include "proto/core/sim_chassis.pb.h"I
#include "submodule/third_party/smartsim/proto/drivers/chassis.pb.h"

namespace cpilotm {
namespace core {
namespace canbus {

/**
 * @class CanbusInterface
 *
 * @brief Interface of the canbus module
 */
class CanbusInterface {
  public:
    /**
     * @brief main logic of the canbus module
     */
    virtual void RunOnce() = 0;

    /**
     * @brief Fill the header.
     */
    void FillHeader(smartsim::drivers::Chassis *chassis) {
        auto *header = chassis->mutable_header();
        double timestamp = ros::Time::now().toSec();
        header->set_module_name(FLAGS_canbus_module_name);
        header->set_timestamp_sec(timestamp);
        header->set_sequence_num(++seq_num_);
    }

    /**
     * @brief Serialize all descriptors of the given message to string.
     */
    void GetDescriptorString(const smartsim::drivers::Chassis &chassis, std_msgs::String message) {
        if (!chassis.SerializeToString(&message.data)) {
            ROS_ERROR("failed to serialize!");
        }
    }

    /**
     * @brief Convert the serialized FileDescriptorProto to real descriptors.
     */
    bool RegisterMessage(const std_msgs::String &message, SimChassis sim_chassis) {
        if (!sim_chassis.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

  private:
    uint32_t seq_num_ = 0;
};

} // namesapce canbus
} // namespace core
} // namespace smartsim

#endif // CANBUS_INTERFACE_H