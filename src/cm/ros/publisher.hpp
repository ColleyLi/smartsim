#pragma once

#include "cm/publisher.h"
#include "cm/ros/msgs/adaptor.h"

#include <ros/ros.h>

namespace cm {

struct PublisherImpl {
  PublisherImpl(ros::Publisher&& publisher) : publisher(std::forward<ros::Publisher>(publisher)) {}

  ros::Publisher publisher;
};

Publisher::Publisher(std::unique_ptr<PublisherImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<PublisherImpl>>(impl)) {}

Publisher::~Publisher() = default;

template <typename _Msg, bool _Adaptive = false>
void Publisher::publish(const _Msg& msg) {
  typename AdaptorTraits<_Msg, _Adaptive>::ROSType ros_msg;
  AdaptorTraits<_Msg, _Adaptive>::convert(msg, &ros_msg);
  impl_->publisher.publish(ros_msg);
}

}  // namespace cm
