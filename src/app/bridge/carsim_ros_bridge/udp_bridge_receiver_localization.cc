// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#include "bridge/carsim_ros_bridge/udp_bridge_receiver_localization.h"
#include "common/proto/proto_utils.h"

namespace smartsim {
namespace bridge {

UDPBridgeReceiverLocalization::UDPBridgeReceiverLocalization(ros::NodeHandle& nh) : nh_(nh) {
  CINFO << "udp bridge receiver localization init, starting...";

  std::string bridge_dir;
  nh.getParam("bridge_dir", bridge_dir);
  FLAGS_udp_bridge_localization_conf_file = bridge_dir + FLAGS_udp_bridge_localization_conf_file;

  smartsim::config::UDPReceiverConfig receiver_config;
  CHECK(common::GetProtoFromFile(FLAGS_udp_bridge_localization_conf_file, &receiver_config))
    CERROR << "Unable to load chassis conf file: " + FLAGS_udp_bridge_localization_conf_file;
  CINFO << "Conf file: " << FLAGS_udp_bridge_localization_conf_file << " is loaded.";

  bind_port_ = receiver_config.bind_port();
  topic_name_ = receiver_config.topic_name();

  WebSocketServer::Init(udpConnection_t, bind_port_);

  // publisher
  publisher_ = nh.advertise<std_msgs::String>(topic_name_.c_str(), 1);

}

bool UDPBridgeReceiverLocalization::RunOnce() {
  WebSocketServer::RecvData(udpConnection_t, vehicle_loc_);
  
  //  publish carsim localization
  core::SimLocalization carsim_loc;
  PublishSimLocalizationPb(&carsim_loc);
}

bool UDPBridgeReceiverLocalization::PublishSimLocalizationPb(core::SimLocalization* carsim_loc) {
  auto* header = carsim_loc->mutable_header();
  header->set_timestamp_sec(common::nowSeconds());
  header->set_sequence_num(++msg_seq_num);

  auto mutable_pose = carsim_loc->mutable_pose();
  // position
  mutable_pose->mutable_position()->set_x(veh_loc_.pos_x);
  mutable_pose->mutable_position()->set_y(veh_loc_.pos_y);
  mutable_pose->mutable_position()->set_z(veh_loc_.pos_z);

  // linear velocity
  mutable_pose->mutable_linear_velocity_vrf()->set_x(veh_loc_.vel_x);
  mutable_pose->mutable_linear_velocity_vrf()->set_y(veh_loc_.vel_y);
  mutable_pose->mutable_linear_velocity_vrf()->set_z(veh_loc_.vel_z);

  // linear acceleration
  mutable_pose->mutable_linear_acceleration_vrf()->set_x(veh_loc_.accel_x);
  mutable_pose->mutable_linear_acceleration_vrf()->set_y(veh_loc_.accel_y);
  mutable_pose->mutable_linear_acceleration_vrf()->set_z(veh_loc_.accel_z);

  // angular_velocity
  mutable_pose->mutable_angular_velocity_vrf()->set_x(veh_loc_.angular_vel_x);
  mutable_pose->mutable_angular_velocity_vrf()->set_y(veh_loc_.angular_vel_y);
  mutable_pose->mutable_angular_velocity_vrf()->set_z(veh_loc_.angular_vel_z);

  // heading
  mutable_pose->set_heading(veh_loc_.yaw);
  
  // euler angles
  mutable_pose->mutable_euler_angles()->set_x(veh_loc_.roll);
  mutable_pose->mutable_euler_angles()->set_y(veh_loc_.pitch);
  mutable_pose->mutable_euler_angles()->set_z(veh_loc_.yaw);

  std_msgs::String localization_str;
  if (!carsim_localization.SerializeToString(&localization_str.data)) {
      CERROR << "failed to serialize.";
  }
  publisher_.publish(localization_str);

  return true;

}


}  // namespace bridge
}  // namespace smartsim

using smartsim::bridge::UDPBridgeReceiverLocalization;
int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_bridge_recevier_localization");
  ros::NodeHandle nh("~");
  UDPBridgeReceiverLocalization udp_recevier_localization(nh);
  ros::Rate loop_rate = 100;
  while (ros::ok()) {
    ros::spinOnce();
    udp_recevier_localization.run();
    loop_rate.sleep();
  }

  return 0;
}
