// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "bridge/vtd_ros_bridge/udp_bridge_receiver_traffic_light.h"

#include "bridge/common/bridge_file.h"
#include "common/proto/proto_utils.h"
#include "common/util/time_util.h"
#include "common/logging/log.h"

namespace smartsim {
namespace bridge {

UDPBridgeReceiverVtdTrafficLight::UDPBridgeReceiverVtdTrafficLight(ros::NodeHandle& nh) 
    : nh_(nh) {
  CINFO << "udp bridge receiver vtd traffic light init, starting...";

  // std::string bridge_dir;
  // nh.getParam("bridge_dir", bridge_dir);
  // std::string bridge_receiver_trafficlight_config_file = 
  //   bridge_dir + "/" + std::string(FLAGS_vtd_traffic_light_config_file);
  smartsim::config::UDPReceiverConfig udp_receiver_config;
  if (!common::GetProtoFromFile(FILE_PATH(FLAGS_vtd_traffic_light_config_file), &udp_receiver_config)) {
    CERROR << "load udp bridge component proto param failed";
  }
  bind_port_ = udp_receiver_config.bind_port();
  proto_name_ = udp_receiver_config.proto_name();
  topic_name_ = udp_receiver_config.topic_name();
  enable_timeout_ = udp_receiver_config.enable_timeout();
  CINFO << "UDP Bridge port is: " << bind_port_;
  CINFO << "UDP Bridge for topic name is: " << topic_name_;
  
  // publisher
  publisher_ = nh.advertise<std_msgs::String>(topic_name_.c_str(), 1);

  sock_fd = socket(AF_INET, SOCK_DGRAM, 0); 
  if (sock_fd < 0) {
    CERROR << "create socket failed!\n";
  }

  memset(&client_addr, 0, sizeof(client_addr));
  client_addr.sin_family = AF_INET;
  client_addr.sin_port = htons(bind_port_); 
  client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock_fd, (struct sockaddr*)&client_addr, sock_len) == -1) {
      CERROR << "bind socket failed";
      close(sock_fd);
  }
  
}


bool UDPBridgeReceiverVtdTrafficLight::IsTimeout(double time_stamp) {
  if (enable_timeout_ == false) {
    return false;
  }
  double cur_time = ros::Time::now().toSec();
  if (cur_time < time_stamp) {
    return true;
  }
  if (FLAGS_timeout < cur_time - time_stamp) {
    return true;
  }
  return false;
}

bool UDPBridgeReceiverVtdTrafficLight::recv(int sockfd) {
  int bytes = 0;
  int total_recv = 2 * FRAME_SIZE;
  char recvBuf[2 * FRAME_SIZE] = {0};
  memset(recvBuf, 0, sizeof(char) * total_recv);

  bytes = 
    recvfrom(sockfd, recvBuf, sizeof(recvBuf), 0, (sockaddr *)&client_addr, &sock_len);
  CINFO << "bytes: " << bytes;
  if (bytes == -1 || bytes > total_recv) {
    CERROR << "failed to receive data.";
    return false;
  }

  // Recv third_party msg
  memcpy((char*)(&perception_traffic_light), recvBuf, sizeof(recvBuf));

  std::lock_guard<std::mutex> lock(mutex_);
  vtd_traffic_light_.set_id(perception_traffic_light.id);
  vtd_traffic_light_.set_state(perception_traffic_light.state);
  vtd_traffic_light_.set_statemask(perception_traffic_light.stateMask);
  vtd_traffic_light_.set_ctrlid(perception_traffic_light.ctrlId);
  vtd_traffic_light_.set_cycletime(perception_traffic_light.cycleTime);


  CINFO << "Traffic light id: " << vtd_traffic_light_.id()
           << ", state: " << vtd_traffic_light_.state()
           << ", statemask: " << vtd_traffic_light_.statemask()
           << ", ctrlid; " << vtd_traffic_light_.ctrlid()
           << ", cycletime: " << vtd_traffic_light_.cycletime();

  std_msgs::String vtd_traffic_light_str_;
  if (!vtd_traffic_light_.SerializeToString(&vtd_traffic_light_str_.data)) {
      CERROR << "failed to serialize.";
      return false;
  }
  publisher_.publish(vtd_traffic_light_str_);

  return true;

}

bool UDPBridgeReceiverVtdTrafficLight::run() {
  if (IsTimeout(vtd_traffic_light_.header().timestamp_sec())) {
    CERROR << "recv vtd traffic light msg time out!";
    return false;
  }
  
  recv(sock_fd);

  return true;  
}

}  // namespace bridge
}  // namespace smartsim

using smartsim::bridge::UDPBridgeReceiverVtdTrafficLight;
int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_recevier_vtd_traffic_light_node");
  ros::NodeHandle nh("~");
  UDPBridgeReceiverVtdTrafficLight udp_recevier_vtd_traffic_light_node(nh);
  ros::Rate loop_rate = 100;
  while (ros::ok()) {
    ros::spinOnce();
    udp_recevier_vtd_traffic_light_node.run();
    loop_rate.sleep();
  }

  return 0;
}
