#pragma once

#include "gflags/gflags.h"

DECLARE_string(bridge_module_name);
DECLARE_double(timeout);
DECLARE_string(bridge_dir);
DECLARE_string(udp_bridge_chassis_config_file);
DECLARE_string(udp_bridge_localization_config_file);
DECLARE_string(udp_bridge_control_config_file);
DECLARE_string(udp_bridge_obstacles_config_file);
DECLARE_string(udp_bridge_traffic_light_config_file);