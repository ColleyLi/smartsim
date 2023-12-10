// Copyright 2022 The XUCONG Authors.All Rights Reserved.
#pragma once

#include "gflags/gflags.h"

DECLARE_string(core_dir);
DECLARE_string(core_config_file);
DECLARE_string(control_conf_file);
DECLARE_string(routing_conf_file);
DECLARE_string(canbus_module_name);
DECLARE_int32(canbus_loop_rate);
DECLARE_string(localization_module_name);
DECLARE_int32(localization_loop_rate);
DECLARE_string(obstacles_module_name);
DECLARE_int32(obstacles_loop_rate);
DECLARE_string(traffic_light_module_name);
DECLARE_int32(traffic_light_loop_rate);
DECLARE_string(vehicle_control_module_name);
DECLARE_int32(vehicle_control_loop_rate);
DECLARE_int64(min_cmd_interval);
DECLARE_bool(use_sil_mode);
