// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "bridge/vtd_ros_bridge/udp_bridge_receiver_obstacles.h"

#include "bridge/common/bridge_file.h"
#include "common/proto/proto_utils.h"
#include "common/util/time_util.h"
#include "common/logging/log.h"

namespace smartsim {
namespace bridge {

UDPBridgeReceiverVtdObstacles::UDPBridgeReceiverVtdObstacles(ros::NodeHandle& nh) 
    : nh_(nh) {
  CINFO << "udp bridge receiver vtd obstacles init, starting...";

  // std::string bridge_dir;
  // nh.getParam("bridge_dir", bridge_dir);
  // std::string bridge_receiver_obstacles_config_file = 
  //   bridge_dir + "/" + std::string(FLAGS_vtd_obstacles_config_file);
  smartsim::config::UDPReceiverConfig udp_receiver_config;
  if (!common::GetProtoFromFile(FILE_PATH(FLAGS_vtd_obstacles_config_file), &udp_receiver_config)) {
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
    close(sock_fd);
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

bool UDPBridgeReceiverVtdObstacles::IsTimeout(double time_stamp) {
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

bool UDPBridgeReceiverVtdObstacles::recv(int sockfd) {
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
  memcpy((char*)(&perception_obstacles), recvBuf, sizeof(recvBuf));

  std::lock_guard<std::mutex> lock(mutex_);
  vtd_perception_obstacles_.set_id(perception_obstacles.id);
  vtd_perception_obstacles_.set_category(perception_obstacles.category);
  vtd_perception_obstacles_.set_type(perception_obstacles.type);
  vtd_perception_obstacles_.set_vismask(perception_obstacles.visMask);
  vtd_perception_obstacles_.set_pos_x(perception_obstacles.pos_x);
  vtd_perception_obstacles_.set_pos_y(perception_obstacles.pos_y);
  vtd_perception_obstacles_.set_pos_z(perception_obstacles.pos_z);
  vtd_perception_obstacles_.set_pos_h(perception_obstacles.pos_h);
  vtd_perception_obstacles_.set_pos_p(perception_obstacles.pos_p);
  vtd_perception_obstacles_.set_pos_r(perception_obstacles.pos_r);
  vtd_perception_obstacles_.set_speed_x(perception_obstacles.speed_x);
  vtd_perception_obstacles_.set_speed_y(perception_obstacles.speed_y);
  vtd_perception_obstacles_.set_speed_z(perception_obstacles.speed_z);
  vtd_perception_obstacles_.set_accel_x(perception_obstacles.accel_x);
  vtd_perception_obstacles_.set_accel_y(perception_obstacles.accel_y);
  vtd_perception_obstacles_.set_accel_z(perception_obstacles.accel_z);
  vtd_perception_obstacles_.set_length(perception_obstacles.length);
  vtd_perception_obstacles_.set_width(perception_obstacles.width);
  vtd_perception_obstacles_.set_height(perception_obstacles.height);

  CINFO << "obstacles id: " << vtd_perception_obstacles_.id()
           << ", pos_x; " << vtd_perception_obstacles_.pos_x()
           << ", pos_y; " << vtd_perception_obstacles_.pos_y()
           << ", pos_z; " << vtd_perception_obstacles_.pos_z()
           << ", lenght; " << vtd_perception_obstacles_.length()
           << ", width; " << vtd_perception_obstacles_.width()
           << ", height; " << vtd_perception_obstacles_.height();

  std_msgs::String vtd_perception_obstacles_str_;
  if (!vtd_perception_obstacles_.SerializeToString(&vtd_perception_obstacles_str_.data)) {
      CERROR << "failed to serialize.";
      return false;
  }
  publisher_.publish(vtd_perception_obstacles_str_);

  return true;

}

bool UDPBridgeReceiverVtdObstacles::run() {
  if (IsTimeout(vtd_perception_obstacles_.header().timestamp_sec())) {
    CERROR << "recv vtd perception obstacles msg time out!";
    return false;
  }
  
  recv(sock_fd);

  return true;  
}

}  // namespace bridge
}  // namespace smartsim

using smartsim::bridge::UDPBridgeReceiverVtdObstacles;
int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_recevier_vtd_obstacles_node");
  ros::NodeHandle nh("~");
  UDPBridgeReceiverVtdObstacles udp_recevier_vtd_obstacles_node(nh);
  ros::Rate loop_rate = 100;
  while (ros::ok()) {
    ros::spinOnce();
    udp_recevier_vtd_obstacles_node.run();
    loop_rate.sleep();
  }

  return 0;
}
