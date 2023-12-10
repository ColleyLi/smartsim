#pragma once

#include "cm/ap/msgs/adaptor.h"
#include "cm/subscriber.h"

namespace cm {

struct SubscriberImpl {
  SubscriberImpl() {}
  SubscriberImpl(RawDataProxyPtr&& raw_subscriber)
      : sub_raw(std::forward<RawDataProxyPtr>(raw_subscriber)) {}
  SubscriberImpl(PcdProxyPtr&& pcd_subscriber)
      : sub_pcd(std::forward<PcdProxyPtr>(pcd_subscriber)) {}
  SubscriberImpl(ImageProxyPtr&& image_subscriber)
      : sub_image(std::forward<ImageProxyPtr>(image_subscriber)) {}
  SubscriberImpl(RadarProxyPtr&& radar_subscriber)
      : sub_radar(std::forward<RadarProxyPtr>(radar_subscriber)) {}
  SubscriberImpl(InsProxyPtr&& ins_subscriber)
      : sub_ins(std::forward<InsProxyPtr>(ins_subscriber)) {}
  SubscriberImpl(ImuProxyPtr&& imu_subscriber)
      : sub_imu(std::forward<ImuProxyPtr>(imu_subscriber)) {}
  SubscriberImpl(ChassisProxyPtr&& chassis_subscriber)
      : sub_chassis(std::forward<ChassisProxyPtr>(chassis_subscriber)) {}
  SubscriberImpl(StrProxyPtr&& string_subscriber)
      : sub_str(std::forward<StrProxyPtr>(string_subscriber)) {}

  RawDataProxyPtr sub_raw;
  PcdProxyPtr sub_pcd;
  ImageProxyPtr sub_image;
  RadarProxyPtr sub_radar;
  InsProxyPtr sub_ins;
  ImuProxyPtr sub_imu;
  ChassisProxyPtr sub_chassis;
  StrProxyPtr sub_str;
};

Subscriber::Subscriber(Subscriber&&) = default;

Subscriber::Subscriber(std::unique_ptr<SubscriberImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<SubscriberImpl>>(impl)) {}

Subscriber::~Subscriber() = default;

}  // namespace cm
