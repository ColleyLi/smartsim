// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "bridge/carsim_ros_bridge/udp_bridge_receiver_chassis.h"
#include "common/proto/proto_utils.h"

namespace smartsim {
namespace bridge {

UDPBridgeReceiverChassis::UDPBridgeReceiverChassis(ros::NodeHandle& nh) : nh_(nh) {
  CINFO << "udp bridge receiver carsim chassis init, starting...";

  std::string bridge_dir;
  nh.getParam("bridge_dir", bridge_dir);
  FLAGS_udp_bridge_chassis_conf_file = bridge_dir + FLAGS_udp_bridge_chassis_conf_file;

  smartsim::config::UDPReceiverConfig receiver_config;
  CHECK(common::GetProtoFromFile(FLAGS_udp_bridge_chassis_conf_file, &receiver_config))
    CERROR << "Unable to load chassis conf file: " + FLAGS_udp_bridge_chassis_conf_file;
  CINFO << "Conf file: " << FLAGS_udp_bridge_chassis_conf_file << " is loaded.";

  bind_port_ = receiver_config.bind_port();
  topic_name_ = receiver_config.topic_name();

  WebSocketServer::Init(udpConnection_t, bind_port_);

  // publisher
  publisher_ = nh.advertise<std_msgs::String>(topic_name_.c_str(), 1);
  
}

bool UDPBridgeReceiverChassis::RunOnce() {
  WebSocketServer::RecvData(udpConnection_t, veh_chassis_);
  
  //  publish carsim canbus
  core::SimChassis carsim_chassis;
  PublishSimChassisPb(&carsim_chassis);
}

void UDPBridgeReceiverChassis::PublishSimChassisPb(core::SimChassis* carsim_chassis) {
  carsim_chassis.set_speed(veh_chassis_.speed);
  carsim_chassis.set_acceleration(veh_chassis_.acceleration);
  carsim_chassis.set_steering_angle(veh_chassis_.steering_angle);

  std_msgs::String carsim_chassis_str;
  if (!carsim_chassis.SerializeToString(&carsim_chassis_str.data)) {
      CERROR << "failed to serialize.";
      return false;
  }
  publisher_.publish(carsim_chassis_str);


}


}  // namespace bridge
}  // namespace smartsim

using smartsim::bridge::UDPBridgeReceiverChassis;
int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_bridge_recevier_chassis_node");
  ros::NodeHandle nh("~");
  UDPBridgeReceiverChassis udp_recevier_chassis_node(nh);
  ros::Rate loop_rate = 100;
  while (ros::ok()) {
    ros::spinOnce();
    udp_recevier_chassis_node.run();
    loop_rate.sleep();
  }

  return 0;
}
