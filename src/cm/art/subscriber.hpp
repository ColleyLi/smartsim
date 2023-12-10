#pragma once

#include "caic_interface.h"
#include "cm/subscriber.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "tzc/tzc_helper.h"

namespace cm {

using RTPString = stoic::cm::proto::common::PString;
using RTImage = stoic::cm::proto::sensor::Image;
using RTPointCloud = stoic::cm::proto::sensor::PointCloudTypeArray;

struct SubscriberImpl {
  using readerNull = nullptr_t;
  using readerRTPString = std::shared_ptr<apollo::cyber::Reader<RTPString>>;
  using readerRTImage = std::shared_ptr<apollo::cyber::Reader<RTImage>>;
  using readerRTPointCloud = std::shared_ptr<apollo::cyber::Reader<RTPointCloud>>;

  SubscriberImpl(readerNull&& sub) : sub_(std::forward<readerNull>(sub)) {}

  SubscriberImpl(readerRTPString&& sub) : sub_string(std::forward<readerRTPString>(sub)) {}
  SubscriberImpl(readerRTImage&& sub) : sub_image(std::forward<readerRTImage>(sub)) {}
  SubscriberImpl(readerRTPointCloud&& sub)
      : sub_pointcloud(std::forward<readerRTPointCloud>(sub)) {}

  readerNull sub_;
  readerRTPString sub_string;
  readerRTImage sub_image;
  readerRTPointCloud sub_pointcloud;
};

Subscriber::Subscriber(Subscriber&&) = default;

Subscriber::Subscriber(std::unique_ptr<SubscriberImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<SubscriberImpl>>(impl)) {}

Subscriber::~Subscriber() = default;

}  // namespace cm
