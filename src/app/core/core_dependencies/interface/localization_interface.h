// Copyright 2023 The XUCONG Authors. All Rights Reserved.
#ifndef LOCALIZATION_INTERFACE_H
#define LOCALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

#include "core/common/core_gflags.h"
#include "proto/core/sim_localization.pb.h"
#include "submodule/third_party/smartsim/proto/localization/localization.pb.h"

namespace smartsim {
namespace core {
namespace localization {

/**
 * @class LocalizationInterface
 *
 * @brief Interface of the Localization module
 */
class LocalizationInterface {
public:
    /**
     * @brief main logic of the Localization module
     */
    virtual void RunOnce() = 0;

    /**
     * @brief Fill the header.
     */
    void FillHeader(smartsim::localization::LocalizationEstimate *chassis) {
        auto *header = chassis->mutable_header();
        double timestamp = ros::Time: :now().toSec();
        header->set_module_name(FLAGS_localization_module_name);
        header->set_timestamp_sec(timestamp);
        header->set_sequence_num(++seq_num_);
    }

    /**
     * @brief Serialize all descriptors of the given message to string.
     */
    void GetDescriptorString(const smartsim::localization::LocalizationEstimate &loc, std_msgs::String message) {
        if (!loc.SerializeToString(&message.data)) {
            ROS_ERROR("failed to serialize!");
        }
    }

    /**
     * @brief Convert the serialized FileDescriptorProto to real descriptors.
     */
    bool RegisterMessage(const std_msgs::String &message, SimLocalization sim_loc) {
        if (!sim_loc.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

private:
    uint32_t seq_num_ = 0;
};

} // namespace localization
} // namespace core
} // namespace smartsim

#endif // LOCALIZATION_INTERFACE_H