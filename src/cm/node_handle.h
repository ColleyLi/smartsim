#pragma once

#include <boost/function.hpp>
#include <functional>
#include <memory>

#include "cm/publisher.h"
#include "cm/subscriber.h"

namespace cm {

struct NodeHandleImpl;

class NodeHandle {
 public:
  inline NodeHandle(const std::string& name);
  inline ~NodeHandle();

  template <typename _Msg, bool _Adaptive>
  inline Publisher advertise(const std::string& topic, size_t queue_size);

  template <typename _Msg, bool _Adaptive>
  inline Subscriber subscribe(const std::string& topic, size_t queue_size,
                              void (*callback)(const _Msg&));

  template <typename _Msg, bool _Adaptive>
  inline Subscriber subscribe(const std::string& topic, size_t queue_size,
                              const boost::function<void(std::shared_ptr<_Msg>)>& callback);

  template <typename _Msg>
  inline Publisher advertise(const std::string& topic, size_t queue_size);

  template <typename _Msg, typename Enable = void>
  Subscriber subscribe(const std::string& topic, size_t queue_size, void (*callback)(const _Msg&));

  template <typename _Msg, typename Enable = void>
  Subscriber subscribe(const std::string& topic, size_t queue_size,
                       const boost::function<void(std::shared_ptr<_Msg>)>& callback);

  inline bool getParam(const std::string& name, std::string& value);

 private:
  std::unique_ptr<NodeHandleImpl> impl_;
};

}  // namespace cm

#if defined MW_ROS_IPC || defined MW_ROS_ORIN
#include "cm/ros/node_handle.hpp"
#include "cm/ros/publisher.hpp"
#include "cm/ros/subscriber.hpp"
#endif

#ifdef MW_AP
#include "cm/ap/node_handle.hpp"
#include "cm/ap/publisher.hpp"
#include "cm/ap/subscriber.hpp"
#endif

#ifdef MW_RT
#include "cm/rt/node_handle.hpp"
#include "cm/rt/publisher.hpp"
#include "cm/rt/subscriber.hpp"
#endif

#ifdef MW_TZ
#include "cm/tz/node_handle.hpp"
#include "cm/tz/publisher.hpp"
#include "cm/tz/subscriber.hpp"
#endif

#ifdef MW_ROSM
#include "cm/ros_automsg/node_handle.hpp"
#include "cm/ros_automsg/publisher.hpp"
#include "cm/ros_automsg/subscriber.hpp"
#endif

#if defined MW_ART_IPC || defined MW_ART_ORIN
#include "cm/art/node_handle.hpp"
#include "cm/art/publisher.hpp"
#include "cm/art/subscriber.hpp"
#endif
