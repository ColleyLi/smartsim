// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "bridge/carsim_ros_bridge/udp_bridge_sender_control.h"
#include "common/proto/proto_utils.h"


namespace smartsim {
namespace bridge {

UDPBridgeSenderControl::UDPBridgeSenderControl(ros::NodeHandle& nh) : nh_(nh) {
  CINFO << "udp bridge sender carsim control init, starting...";

  // load udp bridge config file
  // modify by xucong, 2022-09-08
  // std::string bridge_dir;
  // nh.getParam("bridge_dir", bridge_dir);
  // std::string bridge_sender_carsim_config_file = 
  //   bridge_dir + "/" + std::string(FLAGS_bridge_sender_carsim_config_file);

  smartsim::config::UDPSenderConfig sender_config;
  CHECK(common::GetProtoFromFile(FLAGS_udp_bridge_control_config_file, &receiver_config))
    CERROR << "Unable to load chassis conf file: " + FLAGS_udp_bridge_control_config_file;
  CINFO << "Conf file: " << FLAGS_udp_bridge_control_config_file << " is loaded.";
  topic_name_ = udp_sender_config.topic_name();
  remote_ip_ = udp_sender_config.remote_ip();
  remote_port_ = udp_sender_config.remote_port();
  
  // subscriber
  subscriber_ = nh.subscribe(topic_name_.c_str(), 1, &UDPBridgeSenderControl::OnSimControl, 
                              this, ros::TransportHints().tcpNoDelay());
  
  WebSocketServer::Init(udpConnection_t, remote_ip_, remote_port_);
  
}

bool UDPBridgeSenderControl::RunOnce() {
  if (!is_control_ready_) {
    CERROR << "carsim control msg not ready!";
  }

  WebSocketServer::SendData(udpConnection_t, veh_ctrl_);
  
}

void UDPBridgeSenderControl::OnSimControl(const std_msgs::String& msg) {
  core::SimControl ctrl_cmd_;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!ctrl_cmd_.ParseFromString(msg.data)) {
    CERROR << "sim control command data cannot be converted to protobuf object.";
    veh_ctrl_.acceleration = 0.0;
    veh_ctrl_.steering_angle = 0.0;
  }
  else {
    CINFO << "control command data converted in callback func.";
    is_control_ready_ = true;

    veh_ctrl_.acceleration = ctrl_cmd_.acceleration();
    veh_ctrl_.steering_angle = ctrl_cmd_.steering_angle();

    CERROR << "Sim control command: "
             << " acceleration: " << veh_ctrl_.acceleration
             << ", steering_angle: " << veh_ctrl_.steering_angle;
  }

}

}  // namespace bridge
}  // namespace smartsim

using smartsim::bridge::UDPBridgeSenderControl;
int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_bridge_sender_control_node");
  ros::NodeHandle nh("~");
  UDPBridgeSenderControl udp_sender_control_node(nh);
  ros::Rate loop_rate = 100;
  while (ros::ok()) {
    ros::spinOnce();
    udp_sender_control_node.run();
    loop_rate.sleep();
  }

  return 0;
}
