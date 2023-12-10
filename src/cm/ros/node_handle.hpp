#pragma once

#include "cm/node_handle.h"
#include "cm/ros/msgs/adaptor.h"

#include <ros/ros.h>


namespace cm {

struct NodeHandleImpl {
  NodeHandleImpl(const std::string& name) : nh(name), spinner(3){spinner.start();}

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
};

NodeHandle::NodeHandle(const std::string& name) { impl_ = std::make_unique<NodeHandleImpl>(name); }

NodeHandle::~NodeHandle() = default;

template <typename _Msg, bool _Adaptive = false>
Publisher NodeHandle::advertise(const std::string& topic, size_t queue_size) {
  return Publisher(std::make_unique<PublisherImpl>(
      impl_->nh.advertise<typename AdaptorTraits<_Msg, _Adaptive>::ROSType>(topic, queue_size)));
}

template <typename _Msg, bool _Adaptive = false>
Subscriber NodeHandle::subscribe(const std::string& topic, size_t queue_size,
                                 void (*callback)(const _Msg&)) {
  using ROSType = typename AdaptorTraits<_Msg, _Adaptive>::ROSType;
  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh.subscribe<ROSType>(
      topic, queue_size, boost::bind(&callbackAdaptor<_Msg, ROSType, _Adaptive>, _1, callback))));
}

template <typename _Msg, bool _Adaptive = false>
Subscriber NodeHandle::subscribe(const std::string& topic, size_t queue_size,
                                 const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  using ROSType = typename AdaptorTraits<_Msg, _Adaptive>::ROSType;
  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh.subscribe<ROSType>(
      topic, queue_size,
      boost::bind(&callbackAdaptorBoost<_Msg, ROSType, _Adaptive>, _1, callback))));
}

bool NodeHandle::getParam(const std::string& name, std::string& value) {
  return impl_->nh.getParam(name, value);
}

}  // namespace cm
