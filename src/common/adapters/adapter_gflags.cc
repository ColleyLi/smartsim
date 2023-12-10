#include "common/adapters/adapter_gflags.h"

DEFINE_string(chassis_topic, "/smartsim/canbus/canbus", "smartsim canbus topic name");
DEFINE_string(Localization_topic, "/smartsim/localization",
                "smartsim localization topic name");
DEFINE_string(control_command_topic, "/smartsim/control",
"smartsim control command topic name");
DEFINE_string(perception_topic, "/smartsim/perception/obstacles",
"smartsim perception obstacle topic name");
DEFINE_string(traffic_light_detection_topic,"/smartsim/perception/traffic_light",
"smartsim traffic light detection topic name");
EFINE_string(routing_dest_topic, "/move_base_simple/goal",
"smartsim routing destination topic name");
DEFINE_string(ad_chassis_topic,"/smartcar/canbus/canbus", "adas canbus topic name");
DEFINE_string(ad_localization_topic,"/smartcar/localization/pose",
"adas localization topic name");
EFINE_string(ad_control_topic,"/smartcar/control_command",
"adas control command topic name");
EFINE_string(ad_perception_topic, "/perception_node/perception_objects","perception obstacle topic name");
EFINE_string(ad_traffic_light_topic,"/smartcar/perception/traffic_light","adas traffic light detection topic name");

// TO DO
DEFINE_string(pointcloud_topic,
              "/smartcar/sensor/lidar16/PointCloud2",
              "16 beam Lidar pointcloud topic name");
DEFINE_string(radar_topic,
              "/smartcar/sensor/radar",
              "radar topic name");
DEFINE_string(ultrasonic_radar_topic,
              "/smartcar/sensor/ultrasonic_radar",
              "ultrasonic esr radar topic name");
DEFINE_string(image_topic,
              "/usb_cam/image_raw/compressed",
              "camera image topic name for obstacles from camera");