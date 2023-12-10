#include "bridge_gflags.h"

DEFINE_string(bridge_module_name, "Bridge", "Bridge module name");
DEFINE_double(timeout, 1.0, "receive/send proto msg time out");
DEFINE_string(bridge_dir, "",
              "Directory which contains udp config files");
DEFINE_string(udp_bridge_chassis_conf_file, "/conf/udp_bridge_receiver_carsim_chassis.pb.txt",
              "send to carsim config file");
DEFINE_string(udp_bridge_localization_config_file, "/conf/udp_bridge_receiver_carsim_localization.pb.txt",
              "receive from carsim config file");
DEFINE_string(udp_bridge_control_config_file, "/conf/udp_bridge_sender_carsim_control.pb.txt",
              "receive from carsim config file");
DEFINE_string(udp_bridge_obstacles_config_file, "/conf/udp_bridge_receiver_perception_objects.pb.txt",
              "receive from vtd obstacles config file");
DEFINE_string(udp_bridge_traffic_light_config_file, "/conf/udp_bridge_receiver_traffic_light.pb.txt",
              "receive from vtd traffic light config file");