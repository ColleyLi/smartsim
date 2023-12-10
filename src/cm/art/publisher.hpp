#pragma once

#include "cm/art/msgs/adaptor.h"
#include "cm/publisher.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "tzc/ARTNode.h"
#include "tzc/tzc_helper.h"

#include "cm/base/time.hpp"

namespace cm {

using RTString = stoic::cm::proto::common::PString;
using RTImage = stoic::cm::proto::sensor::Image;

using RTPointCloud = stoic::cm::proto::sensor::PointCloudTypeArray;
struct PublisherImpl {
  using PubCommonString = std::shared_ptr<apollo::cyber::Writer<RTString>>;
  using PubSensorImage = std::shared_ptr<apollo::cyber::Writer<RTImage>>;
  using PubSensorPointCloud = std::shared_ptr<apollo::cyber::Writer<RTPointCloud>>;

  PublisherImpl(PubSensorImage&& pub) : pub_sensor_image(std::forward<PubSensorImage>(pub)) {
    printf("PublisherImpl GetChannelNameis: %s\n", pub_sensor_image->GetChannelName().c_str());
    p_tzcHelper_ =
        std::make_shared<autoplt::tzc::TZCHelper<RTImage>>(pub_sensor_image->GetChannelName());

    auto tzcHInitSucc = p_tzcHelper_->InitTZC(16);  // TODO
    if (!tzcHInitSucc) {
      AERROR << "[Example tzc talker] TZC talker Init failed.";
    } else {
      printf("CM InitTZC for PubSensorImage ok.\n");
    }
  }
  PublisherImpl(PubSensorPointCloud&& pub) : pub_sensor_pc(std::forward<PubSensorPointCloud>(pub)) {
    printf("PublisherImpl GetChannelNameis: %s\n", pub_sensor_pc->GetChannelName().c_str());
    p_tzcHelper_pc_ =
        std::make_shared<autoplt::tzc::TZCHelper<RTPointCloud>>(pub_sensor_pc->GetChannelName());

    auto tzcHInitSucc = p_tzcHelper_pc_->InitTZC(16);  // TODO
    if (!tzcHInitSucc) {
      AERROR << "[Example tzc talker] TZC talker Init failed.";
    } else {
      printf("CM InitTZC for PubSensorPointCloud ok.\n");
    }
  }
  PublisherImpl(PubCommonString&& pub) : pub_common_string(std::forward<PubCommonString>(pub)) {}

  PubSensorImage pub_sensor_image;
  PubSensorPointCloud pub_sensor_pc;
  PubCommonString pub_common_string;

  // shm for image.
  std::shared_ptr<autoplt::tzc::TZCHelper<RTImage>> p_tzcHelper_;
  std::shared_ptr<autoplt::tzc::TZCHelper<RTPointCloud>> p_tzcHelper_pc_;
};

Publisher::Publisher(std::unique_ptr<PublisherImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<PublisherImpl>>(impl)) {}

Publisher::~Publisher() = default;

template <typename _Msg, bool _Adaptive = true>
inline void Publisher::publish(const _Msg& msg) {
  using T = _Msg;
  typename AdaptorTraits<T, _Adaptive>::RTType rt_msg;

  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  {
    stoic::cm::Performance perf(msg_type, 0, "send_convert_to_proto",
                                "Warning: Please Use POD MSg!!!");
    AdaptorTraits<T, _Adaptive>::convert(msg, rt_msg);
  }

  stoic::cm::proto::common::PString rtstring;
  std::string s;

  {
    stoic::cm::Performance perf(msg_type, 0, "send_serialize_to_string",
                                "Warning: Please Use POD MSg!!!");
    rt_msg.SerializeToString(&s);
  }

  {
    stoic::cm::Performance perf(msg_type, s.length(), "send_pub_write",
                                "Warning: Please Use POD MSg!!!");
    rtstring.set_data(std::move(s));
    impl_->pub_common_string->Write(rtstring);
  }
}

//template <>
//inline void Publisher::publish<::caic_sensor::Image, true>(const ::caic_sensor::Image& msg) {
//  using T = ::caic_sensor::Image;
//  using RTType = AdaptorTraits<T, true>::RTType;
//  std::shared_ptr<RTType> p_rt_msg = std::make_shared<RTType>();
//  AdaptorTraits<T, true>::convert(msg, p_rt_msg);
//  // shm
//  static apollo::cyber::transport::WritableBlock wb;
//  impl_->p_tzcHelper_->GetSHMBlockToWrite(msg.size, wb, p_rt_msg);
//  memcpy(wb.buf, msg.p_data, msg.size);
//  impl_->pub_sensor_image->Write(p_rt_msg);  // TODO,  call this by Type_name.
//  impl_->p_tzcHelper_->ReleaseWriteLock(wb);
//  try_exit_safely();
//}
//
//template <>
//inline void Publisher::publish<::caic_sensor::PointCloudTypeArray, true>(
//    const ::caic_sensor::PointCloudTypeArray& msg) {
//  using T = ::caic_sensor::PointCloudTypeArray;
//  using RTType = AdaptorTraits<T, true>::RTType;  // proto type
//  std::shared_ptr<RTType> p_rt_msg = std::make_shared<RTType>();
//  AdaptorTraits<T, true>::convert(msg, p_rt_msg);
//  // shm
//  size_t size = msg.width * msg.height * sizeof(::caic_sensor::PointXYZIRTL);
//  void* buf = (void*)msg.points.data();
//
//  // printf("sizeof(::caic_sensor::PointXYZIRTL) is: %d\n", sizeof(::caic_sensor::PointXYZIRTL));
//  // printf("publish points SHM, timestamp is: %ld, memory size is: %ld\n", msg.timestamp, size);
//
//  static apollo::cyber::transport::WritableBlock wb;
//  impl_->p_tzcHelper_pc_->GetSHMBlockToWrite(size, wb, p_rt_msg);
//  memcpy(wb.buf, buf, size);
//  impl_->pub_sensor_pc->Write(p_rt_msg);  // TODO,  call this by Type_name.
//  impl_->p_tzcHelper_pc_->ReleaseWriteLock(wb);
//}

}  // namespace cm
