// Copyright 2022 The XUCONG Authors. All Rights Reserved.
#include "core/common/core_gflags.h"

DEFINE_string(core_dir, "",
              "vehicle provider dir for file load and initilized when node starts");
DEFINE_string(core_config_file,
              "/src/canbus/conf/canbus.conf", "The adapter configuration file");
DEFINE_string(control_conf_file, "/conf/control_conf.pb.txt",
              "default control conf data file");
DEFINE_string(routing_conf_file, "/routing/conf/routing_conf.pb.txt",
              "default routing conf data file");
DEFINE_string(canbus_module_name, "Canbus", "Canbus module name");
DEFINE_int32(canbus_loop_rate, 100, "Loop rate for canbus node");
DEFINE_string(localization_module_name, "Localization", "Localization module name");
DEFINE_int32(localization_loop_rate, 100, "Loop rate for localization node");
DEFINE_string(obstacles_module_name, "obstacles", "Canbus module name");
DEFINE_int32(obstacles_loop_rate, 5, "Loop rate for canbus node");
DEFINE_string(traffic_light_module_name,"traffic_light", "Localization module name");
DEFINE_int32(traffic_light_loop_rate, 5, "Loop rate for localization node");
DEFINE_string(vehicle_control_module_name, "Vehicle_Control", "Canbus module name");
DEFINE_int32(vehicle_control_loop_rate, 100, "Loop rate for canbus node");
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");
DEFINE_bool(use_sil_mode, false, "");