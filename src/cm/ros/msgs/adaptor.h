#pragma once

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>

#include "cm/alg/pool/object_pool.hpp"
#include "third_party/apollo/proto/control/control_cmd.pb.h"
namespace cm {

using namespace stoic;  // PRQA S 2522 # Todo: to be solved

template <typename _T, bool _Adaptive = false>
struct AdaptorTraits;

template <>
struct AdaptorTraits<caic_sensor::Canbus, true> {
  using Type = caic_sensor::Canbus;
  using ProtoType = ::apollo::drivers::Chassis;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ros_msg.data);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, ROSType* ros_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ros_msg->data);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg.header().timestamp_sec() * 1000000.0;
    msg->driving_mode = (caic_sensor::DrivingMode)proto_msg.driving_mode();
    msg->chassis_info.esp.vehicle_speed = proto_msg.speed_mps();
    msg->chassis_info.esp.wheel_pulse_info.fl = proto_msg.wheel_pluse().wheel_pluse_fl();
    msg->chassis_info.esp.wheel_pulse_info.fr = proto_msg.wheel_pluse().wheel_pluse_fr();
    msg->chassis_info.esp.wheel_pulse_info.rl = proto_msg.wheel_pluse().wheel_pluse_rl();
    msg->chassis_info.esp.wheel_pulse_info.rr = proto_msg.wheel_pluse().wheel_pluse_rr();
    msg->chassis_info.eps.steering_wheel_info.angle =
        proto_msg.steering_percentage() * 8.726646 / 100;
    msg->chassis_info.eps.steering_wheel_info.speed = proto_msg.steeringwheelspeed();
    msg->chassis_info.eps.steering_wheel_info.speed_sign = proto_msg.steeringwheelspeedsign();
    msg->chassis_info.esp.wheel_speed_info.fl = proto_msg.wheel_speed().wheel_spd_fl();
    msg->chassis_info.esp.wheel_speed_info.fr = proto_msg.wheel_speed().wheel_spd_fr();
    msg->chassis_info.esp.wheel_speed_info.rl = proto_msg.wheel_speed().wheel_spd_rl();
    msg->chassis_info.esp.wheel_speed_info.rr = proto_msg.wheel_speed().wheel_spd_rr();
    switch (proto_msg.wheel_speed().wheel_direction_rr()) {
      case 0:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::FORWARD;
        break;
      case 1:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::BACKWARD;
        break;
      case 2:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::STOP;
        break;
      case 3:
        msg->chassis_info.esp.driving_direction = caic_sensor::DrivingDirection::INVALID_VALUE;
        break;
      default:
        break;
    }

    switch (proto_msg.gear_location()) {
      case 0:
        msg->chassis_info.gear_info = caic_std::GearState::NEUTRAL;
        break;
      case 1:
        msg->chassis_info.gear_info = caic_std::GearState::DRIVE;
        break;
      case 2:
        msg->chassis_info.gear_info = caic_std::GearState::REVERSE;
        break;
      case 3:
        msg->chassis_info.gear_info = caic_std::GearState::PARKING;
        break;
      default:
        msg->chassis_info.gear_info = caic_std::GearState::INVALID;
        break;
    }
  }
  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    // 1.0 head
    // proto::common::Header* header = proto_msg->mutable_header();
    // // header->set_seq(msg.header.seq);
    // header->set_stamp(msg.header.stamp);

    // proto_msg->mutable_chassis_info()->mutable_esp()->set_vehicle_speed(
    //     msg.chassis_info.esp.vehicle_speed);
    // proto_msg->mutable_chassis_info()->set_gear_info(
    //     (proto::sensor::Chassis_Info_Gear_Info)(msg.chassis_info.gear_info));
    // proto_msg->mutable_chassis_info()->mutable_yrs()->set_yaw_rate(msg.chassis_info.yrs.yaw_rate);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_fl(
    //     msg.chassis_info.esp.wheel_speed_info.fl);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_fr(
    //     msg.chassis_info.esp.wheel_speed_info.fr);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_rl(
    //     msg.chassis_info.esp.wheel_speed_info.rl);
    // proto_msg->mutable_chassis_info()->mutable_esp()->mutable_wheel_speed_info()->set_rr(
    //     msg.chassis_info.esp.wheel_speed_info.rr);
    // TODO set frame_id
    // 2.0 meta
    // proto::common::OdomMeta* meta = rt_msg.mutable_meta();
    // meta->set_start_timestamp_us(msg.meta.start_timestamp_us);
    // meta->set_finish_timestamp_us(msg.meta.finish_timestamp_us);
    // 3.0
    // rt_msg.set_available((uint64_t)msg.available);
    // 4.0
    // rt_msg.set_child_frame_id(msg.child_frame_id);
    // 5.0
    // 6.0
    // 7.0
    printf("convert odo, pub msg: %d\n",
           ((proto::sensor::Chassis_Info_Gear_Info)(msg.chassis_info.gear_info)));
  }
};


template <typename _T>
struct AdaptorTraits<_T, false> {
  using Type = _T;
  using ROSType = std_msgs::String;

  static void convert(const ROSType& ros_msg, Type* msg) { msg->ParseFromString(ros_msg.data); }

  static void convert(const Type& msg, ROSType* ros_msg) { msg.SerializeToString(&ros_msg->data); }
};

template <typename _T>
struct AdaptorTraits<_T, true> {
  using Type = _T;
  using ROSType = std_msgs::String;

  static void convert(const ROSType&, Type*) {}

  static void convert(const Type&, ROSType*) {}
};

template <typename _Msg, typename _ROSMsg, bool _Adaptive>
void callbackAdaptor(const typename _ROSMsg::ConstPtr& ros_msg, void (*callback)(const _Msg&)) {
  _Msg msg;
  AdaptorTraits<_Msg, _Adaptive>::convert(*ros_msg, &msg);
  callback(msg);
}

template <typename _Msg, typename _ROSMsg, bool _Adaptive>
void callbackAdaptorBoost(typename _ROSMsg::ConstPtr ros_msg,
                          const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  static stoic::cm::alg::ObjectPool<_Msg> pool(9);
  std::shared_ptr<_Msg> msg = pool.calloc();
  AdaptorTraits<_Msg, _Adaptive>::convert(*ros_msg, msg.get());
  callback(msg);
}

}  // namespace cm
