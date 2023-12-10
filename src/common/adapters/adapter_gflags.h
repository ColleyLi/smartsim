#ifndef ADAPTER_GFLAGS_H
#define ADAPTER_GFLAGS_H

#include "gflags/gflags.h"

DECLARE_string(chassis_topic);
DECLARE_string(localization_topic);
DECLARE_string(control_command_topic);
DECLARE_string(perception_topic);
DECLARE_string(traffic_light_detection_topic);
DECLARE_string(routing_dest_topic);
DECLARE_string(ad_chassis_topic);
DECLARE_string(ad_localization_topic);
DECLARE_string(ad_control_topic);
DECLARE_string(ad_perception_topic);
DECLARE_string(ad_traffic_light_topic);
// TO DO
DECLARE_string(pointcloud_topic);
DECLARE_string(radar_topic);
DECLARE_string(ultrasonic_radar_topic);
DECLARE_string(image_topic);
#endif // ADAPTER_GFLAGS_H