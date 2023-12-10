#pragma once

#include <boost/bind/bind.hpp>
#include <boost/function.hpp>

#include "caic_interface.h"
#include "cm/art/msgs/adaptor.h"
#include "cm/node_handle.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "tzc/ARTNode.h"
#include "tzc/tzc_helper.h"

namespace cm {

using RTImage = stoic::cm::proto::sensor::Image;
using RTString = stoic::cm::proto::common::PString;
using RTPStringPtr = std::shared_ptr<RTString>;

struct NodeHandleImpl {
  NodeHandleImpl(const std::string& name) { nh = apollo::cyber::CreateNode(name); }
  std::unique_ptr<apollo::cyber::Node> nh;
};

NodeHandle::NodeHandle(const std::string& name) { impl_ = std::make_unique<NodeHandleImpl>(name); }

NodeHandle::~NodeHandle() = default;

template <typename _Msg, bool _Adaptive = false>
Publisher NodeHandle::advertise(const std::string& topic, size_t /*queue_size*/) {
  impl_->nh->CreateWriter<RTString>(topic);

  return Publisher(std::make_unique<PublisherImpl>(impl_->nh->CreateWriter<RTString>(topic)));
}

//template <>
//inline Publisher NodeHandle::advertise<::caic_sensor::Image, true>(const std::string& topic,
//                                                                   size_t /*queue_size*/) {
//  using RTType = typename AdaptorTraits<::caic_sensor::Image, true>::RTType;
//  return Publisher(std::make_unique<PublisherImpl>(impl_->nh->CreateWriter<RTType>(topic)));
//}
//
//template <>
//inline Publisher NodeHandle::advertise<::caic_sensor::PointCloudTypeArray, true>(
//    const std::string& topic, size_t /*queue_size*/) {
//  using RTType = typename AdaptorTraits<::caic_sensor::PointCloudTypeArray, true>::RTType;
//  return Publisher(std::make_unique<PublisherImpl>(impl_->nh->CreateWriter<RTType>(topic)));
//}

template <typename _Msg, bool _Adaptive = false>
inline Subscriber NodeHandle::subscribe(const std::string& topic, size_t /*queue_size*/,
                                        void (*callback)(const _Msg&)) {
  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh->CreateReader<RTString>(
      topic, boost::bind(&callbackAdaptor<_Msg, RTPStringPtr, _Adaptive>, boost::placeholders::_1,
                         callback, topic))));
}
template <typename _Msg, bool _Adaptive = false>
inline Subscriber NodeHandle::subscribe(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh->CreateReader<RTString>(
      topic, boost::bind(&callbackAdaptorBoost<_Msg, RTPStringPtr, _Adaptive>,
                         boost::placeholders::_1, callback, topic))));
}

//template <>
//inline Subscriber NodeHandle::subscribe<::caic_sensor::Image, true>(
//    const std::string& topic, size_t /*queue_size*/,
//    void (*callback)(const ::caic_sensor::Image&)) {
//  using T = ::caic_sensor::Image;
//  using RTType = typename AdaptorTraits<T, true>::RTType;
//  using RTTypePtr = typename AdaptorTraits<T, true>::RTTypePtr;
//  // for CyberRT recv, have to use callback in smart_pointer.
//  std::shared_ptr<apollo::cyber::Reader<RTType>> tzc_reader = nullptr;
//  tzc_reader = impl_->nh->CreateReader<RTType>(
//      topic,
//      boost::bind(&callbackAdaptor<T, RTTypePtr, true>, boost::placeholders::_1, callback, topic));
//  return Subscriber(std::make_unique<SubscriberImpl>(std::move(tzc_reader)));
//}
//template <>
//inline Subscriber NodeHandle::subscribe<::caic_sensor::Image, true>(
//    const std::string& topic, size_t /*queue_size*/,
//    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback) {
//  using T = ::caic_sensor::Image;
//  using RTType = typename AdaptorTraits<T, true>::RTType;
//  using RTTypePtr = typename AdaptorTraits<T, true>::RTTypePtr;
//
//  std::shared_ptr<apollo::cyber::Reader<RTType>> tzc_reader = nullptr;
//  tzc_reader = impl_->nh->CreateReader<RTType>(
//      topic, boost::bind(&callbackAdaptorBoost<T, RTTypePtr, true>, boost::placeholders::_1,
//                         callback, topic));  // TODO queue_size
//  return Subscriber(std::make_unique<SubscriberImpl>(std::move(tzc_reader)));
//}
//
//template <>
//inline Subscriber NodeHandle::subscribe<::caic_sensor::PointCloudTypeArray, true>(
//    const std::string& topic, size_t /*queue_size*/,
//    void (*callback)(const ::caic_sensor::PointCloudTypeArray&)) {
//  using T = ::caic_sensor::PointCloudTypeArray;
//  using RTType = typename AdaptorTraits<T, true>::RTType;
//  using RTTypePtr = typename AdaptorTraits<T, true>::RTTypePtr;
//  // for CyberRT recv, have to use callback in smart_pointer.
//  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh->CreateReader<RTType>(
//      topic, boost::bind(&callbackAdaptor<T, RTTypePtr, true>, boost::placeholders::_1, callback,
//                         topic))));
//}
//template <>
//inline Subscriber NodeHandle::subscribe<::caic_sensor::PointCloudTypeArray, true>(
//    const std::string& topic, size_t /*queue_size*/,
//    const boost::function<void(std::shared_ptr<::caic_sensor::PointCloudTypeArray>)>& callback) {
//  using T = ::caic_sensor::PointCloudTypeArray;
//  using RTType = typename AdaptorTraits<T, true>::RTType;
//  using RTTypePtr = typename AdaptorTraits<T, true>::RTTypePtr;
//  return Subscriber(std::make_unique<SubscriberImpl>(impl_->nh->CreateReader<RTType>(
//      topic, boost::bind(&callbackAdaptorBoost<T, RTTypePtr, true>, boost::placeholders::_1,
//                         callback, topic))));
//}

bool NodeHandle::getParam(const std::string& /*name*/, std::string& /*value*/) {
  // return impl_->nh.getParam(name, value);
  return true;
}

}  // namespace cm
