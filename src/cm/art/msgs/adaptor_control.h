#pragma once

#include "adaptor_base.h"

namespace cm {
//using GearProto = stoic::cm::proto::drivers::Chassis::GearPosition;
//using TurnSignalProto = stoic::cm::proto::control::TurnSignal;

template <>
struct AdaptorTraits<::caic_control::ControlCommand, true> {
  using Type = ::caic_control::ControlCommand;
  using RTType =stoic::cm::proto::control::ControlCommand;
  using RTTypePtr = std::shared_ptr<RTType>;
  using GearProto = stoic::cm::proto::drivers::Chassis;
  using TurnSignalProto = stoic::cm::proto::control::TurnSignal;


  static void convert(const RTTypePtr& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg->header().stamp() * 1000000l;
    msg->acceleration = proto_msg->acceleration();
    msg->brake = proto_msg->brake();
    msg->steering_target = proto_msg->steering_target();
    msg->steering_rate = proto_msg->steering_rate();
    msg->speed = proto_msg->speed();
    msg->throttle = proto_msg->throttle();
    switch (proto_msg->gear_location()) {
      case GearProto::GEAR_NEUTRAL:
        msg->gear_state = caic_std::GearState::NEUTRAL;
        break;
      case GearProto::GEAR_DRIVE:
        msg->gear_state = caic_std::GearState::DRIVE;
        break;
      case GearProto::GEAR_REVERSE:
        msg->gear_state = caic_std::GearState::REVERSE;
        break;
      case GearProto::GEAR_PARKING:
        msg->gear_state = caic_std::GearState::PARKING;
        break;
      default:
        break;
    }

    if (proto_msg->engine_on_off()) {
      msg->engine_on_off = caic_control::EngineReqSt::ENGINE_OFF_REQUEST;
    }

    msg->vehicle_signal.high_beam = proto_msg->high_beam();
    msg->vehicle_signal.low_beam = proto_msg->low_beam();
    msg->vehicle_signal.horn = proto_msg->horn();
    msg->vehicle_signal.emergency_light = proto_msg->signal().emergency_light();
    switch (proto_msg->turnsignal()) {
      case TurnSignalProto::TURN_NONE:
        msg->vehicle_signal.turn_signal = caic_std::VehicleSignal::TurnSignal::TURN_NONE;
        break;
      case TurnSignalProto::TURN_LEFT:
        msg->vehicle_signal.turn_signal = caic_std::VehicleSignal::TurnSignal::TURN_LEFT;
        break;
      case TurnSignalProto::TURN_RIGHT:
        msg->vehicle_signal.turn_signal = caic_std::VehicleSignal::TurnSignal::TURN_RIGHT;
        break;
      default:
        break;
    }
  }

  static void convert(const Type& msg, RTType& proto_msg) {
    proto_msg.Clear();
    // header
    static long long control_seq_num_ = 0;
    std::string module_name = "control_proto";
    auto* header = proto_msg.mutable_header();
    double timestamp = msg.header.stamp;
    header->set_module_name(module_name);
    header->set_seq(static_cast<unsigned int>(++control_seq_num_));
    // header->set_frame_id(module_name);
    header->set_stamp(timestamp * 1.0 / 1000000l);
    // header->set_seq(static_cast<unsigned int>(++control_seq_num_));
  
    proto_msg.set_acceleration(msg.acceleration);
    proto_msg.set_throttle(msg.throttle);
    proto_msg.set_brake(msg.brake);
    proto_msg.set_steering_target(msg.steering_target);
    proto_msg.set_steering_rate(msg.steering_rate);
    switch (msg.gear_state) {
      case caic_std::GearState::NEUTRAL:
        proto_msg.set_gear_location(GearProto::GEAR_NEUTRAL);
        break;
      case caic_std::GearState::DRIVE:
        proto_msg.set_gear_location(GearProto::GEAR_DRIVE);
        break;
      case caic_std::GearState::REVERSE:
        proto_msg.set_gear_location(GearProto::GEAR_REVERSE);
        break;
      case caic_std::GearState::PARKING:
        proto_msg.set_gear_location(GearProto::GEAR_PARKING);
        break;
      default:
        break;
    }
    proto_msg.set_speed(msg.speed);
    proto_msg.set_high_beam(msg.vehicle_signal.high_beam);
    proto_msg.set_low_beam(msg.vehicle_signal.low_beam);
    proto_msg.set_horn(msg.vehicle_signal.horn);
    proto_msg.mutable_signal()->set_emergency_light(msg.vehicle_signal.emergency_light);
    if (msg.engine_on_off == caic_control::EngineReqSt::ENGINE_OFF_REQUEST) {
      proto_msg.set_engine_on_off(true);
    }

    switch (msg.vehicle_signal.turn_signal) {
      case caic_std::VehicleSignal::TurnSignal::TURN_NONE:
        proto_msg.set_turnsignal(TurnSignalProto::TURN_NONE);
        break;
      case caic_std::VehicleSignal::TurnSignal::TURN_LEFT:
        proto_msg.set_turnsignal(TurnSignalProto::TURN_LEFT);
        break;
      case caic_std::VehicleSignal::TurnSignal::TURN_RIGHT:
        proto_msg.set_turnsignal(TurnSignalProto::TURN_RIGHT);
        break;
      default:
        break;
    }
  }
};

}  // namespace cm
