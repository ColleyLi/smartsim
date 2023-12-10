//Copyright 2023 The XUCONG Authors. All Rights Reserved.
#ifndef PERCEPTION_INTERFACE_H
#define PERCEPTION_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

#include "core/common/core_gflags.h"
#include <osi3/osi_sensordate.pb.h>
#include "submodule/third_party/smartsim/proto/perception/perception_obstacle.pb.h"
#include "submodule/third_party/smartsim/proto/perception/traffic_light_detection.pb.h"

namespace smartsim {
namespace core {
namespace perception {

/**
 * @class CanbusInterface*
 * @brief Interface of the canbus module
 */
class PerceptionInterface {
public:
    /**
     * @brief main logic of the perception module
     */
    virtual void RunOnce() = 0;

    /**
     * @brief Fill the header.
     */
     void FillHeader(smartsim::perception::PerceptionObstacles* perception_objs) {
        auto *header = perception_objs->mutable_header();
        double timestamp = ros::Time::now().toSec();
        header->set_module_name(FLAGS_obstacles_module_name);
        header->set_timestamp_sec(timestamp);
        header->set_sequence_num(++seq_num_);
    }

    void FillHeader(smartsim::perception::TrafficLightDetection* traffic_light) {
        auto *header = traffic_light->mutable_header();
        double timestamp = ros::Time::now().toSec();
        header->set_module_name(FLAGS_traffic_light_module_name);
        header->set_timestamp_sec(timestamp);
        header->set_sequence_num(++seq_num_);
    }

    /**
     * @brief Serialize all descriptors of the given message to string.
     */
    void GetDescriptorString(const smartsim::perception::PerceptionObstacles& perception_objs,std_msgs::String message) {
        if (!perception_objs.SerializeToString(&message.data)) {
            ROS_ERROR("failed to serialize!");
        }
    }

    void GetDescriptorString(const smartsim::perception::TrafficLightDetection& traffic_light, std_msgs::String message) {
        if (!traffic_light.SerializeToString(&message.data)) {
            ROS_ERROR("failed to serialize!");
        }
    }

    /**
     * @brief Convert the serialized FileDescriptorProto to real descriptors.
     */
    bool RegisterMessage(const std_msgs::String& message, smartsim::perception::PerceptionObstacles perception_objs) {
        if (!perception_objs.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

    bool RegisterMessage(const std_msgs::String& message,smartsim::perception::TrafficLightDetection traffic_light) {
        if (!traffic_light.ParseFromString(message.data)) {
            ROS_ERROR("failed to parse proto data");
            return false;
        }
        return true;
    }

private:
    uint32_t seq_num_ = 0;
};

} // namespace perception
} // namespace core
} // namespace smartsim
#endif// PERCEPTION_INTERFACE_H

