#pragma once

#include "adaptor_base.h"

#include "adaptor_control.h"
#include "adaptor_localization.h"
#include "adaptor_map.h"
#include "adaptor_perception.h"
#include "adaptor_fusion.h"
#include "adaptor_sensor.h"
#include "adaptor_interaction.h"
#include "adaptor_sim_chassis.h"
#include "adaptor_sim_control.h"
#include "adaptor_sim_localization.h"
#include "adaptor_gps.h"
#include "adaptor_imu.h"
#include "adaptor_sim_perception.h"

#include "cm/base/time.hpp"

// #include "cm/cm.h"

static bool g_cm_fource_stop = 0;

namespace cm {

static void try_exit_safely() {
  if (g_cm_fource_stop) {
    printf("g_cm_fource_stop is: %d\n", g_cm_fource_stop);
    printf("接收到退出信号，已执行完该周期，准备退出...\n");
    usleep(100000);
    printf("接收到退出信号，已执行完该周期，进程已安全退出!!!\n");
    g_cm_fource_stop = 0;
    exit(1);
  } else {
    // printf("g_cm_fource_stop is: %d\n", g_cm_fource_stop);
  }
}

// _Msg is type for caic, for example ::caic_sensor::caic_ins::Odometry
//_RTPtrMsg is protobuf type, proto::common::PString
template <typename _Msg, typename _PStringMsg, bool _Adaptive>
inline void callbackAdaptor(const _PStringMsg& string_msg, void (*callback)(const _Msg&),
                            std::string topic) {
  using RTType = typename AdaptorTraits<_Msg, true>::RTType;
  _Msg msg;  // TODO, objects pool;

  std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();

  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();

  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_parse_from_string");
    rt_msg->ParseFromString(string_msg->data());
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_convert_to_caic");
    AdaptorTraits<_Msg, _Adaptive>::convert(rt_msg, &msg);
  }
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_callback");
    callback(msg);
  }
  // try_exit_safely();
}

template <typename _Msg, typename _PStringMsg, bool _Adaptive>
inline void callbackAdaptorBoost(const _PStringMsg& string_msg,
                                 const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                                 std::string /*topic*/) {
  using RTType = typename AdaptorTraits<_Msg, _Adaptive>::RTType;
  _Msg msg;
  static std::shared_ptr<_Msg> msg_sp = std::make_shared<_Msg>(msg);

  std::shared_ptr<RTType> rt_msg = std::make_shared<RTType>();

  std::string msg_type = stoic::cm::alg::reflection<_Msg>::fullName();
  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_parse_from_string");
    rt_msg->ParseFromString(string_msg->data());
  }

  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_convert_to_caic");
    AdaptorTraits<_Msg, _Adaptive>::convert(rt_msg, msg_sp.get());
  }

  {
    stoic::cm::Performance perf(msg_type, sizeof(_Msg), "recv_callback");
    callback(msg_sp);
  }

  // try_exit_safely();
}

//template <>
//inline void callbackAdaptor<::caic_sensor::Image, RTImagePtr, true>(
//    const RTImagePtr& rt_msg, void (*callback)(const ::caic_sensor::Image&), std::string topic) {
//  std::string topic_name = topic;
//
//  static std::map<std::string, std::shared_ptr<::caic_sensor::Image>> image_all;
//  if (image_all.find(topic_name) == image_all.end()) {
//    std::shared_ptr<::caic_sensor::Image> msg_sp = std::make_shared<::caic_sensor::Image>();
//    image_all.insert(std::make_pair(topic_name, msg_sp));
//  }
//  AdaptorTraits<::caic_sensor::Image, true>::convert(rt_msg, image_all[topic_name]);
//  // init
//  if (tzcHelper_all.find(topic_name) == tzcHelper_all.end()) {
//    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::Image> tzcHelper(topic_name);
//    tzcHelper.InitTZC(8);
//    tzcHelper_all.insert(std::make_pair(topic_name, tzcHelper));
//  }
//
//  uint8_t* msg_addr = nullptr;
//  msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//
//  if (msg_addr == nullptr) {
//    usleep(3000);
//    msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//  }
//  if (msg_addr == nullptr) {
//    AERROR << "msg_addr is nullptr";
//  } else {
//    image_all[topic_name]->p_data = msg_addr;
//    callback(*(image_all[topic_name].get()));
//    tzcHelper_all.find(topic_name)->second.ReleaseReadLock();
//    try_exit_safely();
//  }
//}
//
//template <>
//inline void callbackAdaptorBoost<::caic_sensor::Image, RTImagePtr, true>(
//    const RTImagePtr& rt_msg,
//    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback,
//    std::string topic) {
//  std::string topic_name = topic;
//
//  static std::map<std::string, std::shared_ptr<::caic_sensor::Image>> image_all;
//  if (image_all.find(topic_name) == image_all.end()) {
//    std::shared_ptr<::caic_sensor::Image> msg_sp = std::make_shared<::caic_sensor::Image>();
//    image_all.insert(std::make_pair(topic_name, msg_sp));
//  }
//
//  AdaptorTraits<::caic_sensor::Image, true>::convert(rt_msg, image_all[topic_name]);
//
//  // init
//  if (tzcHelper_all.find(topic_name) == tzcHelper_all.end()) {
//    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::Image> tzcHelper(topic_name);
//    tzcHelper.InitTZC(8);
//    tzcHelper_all.insert(std::make_pair(topic_name, tzcHelper));
//  }
//
//
//  uint8_t* msg_addr = nullptr;
//  msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//  if (msg_addr == nullptr) {
//    usleep(3000);
//    msg_addr = (uint8_t*)tzcHelper_all.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//  }
//  if (msg_addr == nullptr) {
//    AERROR << "msg_addr is nullptr";
//  } else {
//    // printf("topic_name is: %s\n", topic_name.c_str());
//    image_all[topic_name]->p_data = msg_addr;
//    callback(image_all[topic_name]);
//    tzcHelper_all.find(topic_name)->second.ReleaseReadLock();
//    try_exit_safely();
//  }
//}
//
//// point cloud
//// TODO
//template <>
//inline void callbackAdaptor<::caic_sensor::PointCloudTypeArray, RTPointCloudPtr, true>(
//    const RTPointCloudPtr& rt_msg, void (*callback)(const ::caic_sensor::PointCloudTypeArray&),
//    std::string topic) {
//  std::string topic_name = topic;
//  // printf("callbackAdaptor, topic_name is: %s\n", topic_name.c_str());
//
//  ::caic_sensor::PointCloudTypeArray msg;  // TODO, objects pool;
//  memset((void*)msg.points.data(), 0, (size_t)sizeof(caic_sensor::PointXYZIRTL) * msg.size);
//
//  AdaptorTraits<::caic_sensor::PointCloudTypeArray, true>::convert(rt_msg, &msg);
//
//  // init
//  if (tzcHelper_all_pc.find(topic_name) == tzcHelper_all_pc.end()) {
//    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::PointCloudTypeArray> tzcHelper(topic_name);
//    tzcHelper.InitTZC(8);
//    tzcHelper_all_pc.insert(std::make_pair(topic_name, tzcHelper));
//  }
//
//  void* msg_addr = nullptr;
//  msg_addr = (void*)tzcHelper_all_pc.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//
//  if (msg_addr == nullptr) {
//    usleep(3000);
//    msg_addr = (void*)tzcHelper_all_pc.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//  }
//
//  if (msg_addr == nullptr) {
//    AERROR << "msg_addr is nullptr";
//  } else {
//    size_t size = rt_msg->width() * rt_msg->height() * sizeof(::caic_sensor::PointXYZIRTL);
//    // printf("callbackAdaptor SHM, memary size is: %d\n", size);
//    memcpy((void*)msg.points.data(), msg_addr, size);
//    callback(msg);
//    tzcHelper_all_pc.find(topic_name)->second.ReleaseReadLock();
//  }
//}
//
//template <>
//inline void callbackAdaptorBoost<::caic_sensor::PointCloudTypeArray, RTPointCloudPtr, true>(
//    const RTPointCloudPtr& rt_msg,
//    const boost::function<void(std::shared_ptr<::caic_sensor::PointCloudTypeArray>)>& callback,
//    std::string topic) {
//  std::string topic_name = topic;
//  // printf("callbackAdaptor, topic_name is: %s\n", topic_name.c_str());
//
//  ::caic_sensor::PointCloudTypeArray msg;
//  std::shared_ptr<::caic_sensor::PointCloudTypeArray> msg_sp =
//      std::make_shared<::caic_sensor::PointCloudTypeArray>(msg);
//
//  memset((void*)msg_sp->points.data(), 0, (size_t)sizeof(caic_sensor::PointXYZIRTL) * msg_sp->size);
//
//  AdaptorTraits<::caic_sensor::PointCloudTypeArray, true>::convert(rt_msg, msg_sp.get());
//
//  // init
//  if (tzcHelper_all_pc.find(topic_name) == tzcHelper_all_pc.end()) {
//    printf("tzcHelper_all_pc init.\n");
//    autoplt::tzc::TZCHelper<stoic::cm::proto::sensor::PointCloudTypeArray> tzcHelper(topic_name);
//    tzcHelper.InitTZC(8);
//    tzcHelper_all_pc.insert(std::make_pair(topic_name, tzcHelper));
//  }
//
//  void* msg_addr = nullptr;
//  msg_addr = (void*)tzcHelper_all_pc.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//
//  if (msg_addr == nullptr) {
//    usleep(3000);
//    msg_addr = (void*)tzcHelper_all_pc.find(topic_name)->second.GetDataPtrFromTZC(rt_msg);
//  }
//
//  if (msg_addr == nullptr) {
//    AERROR << "msg_addr is nullptr";
//  } else {
//    int size = rt_msg->width() * rt_msg->height() * sizeof(::caic_sensor::PointXYZIRTL);
//    // printf("callbackAdaptor SHM, rt_msg timestamp is: %ld, memary size is: %d\n",
//    //        rt_msg->timestamp(), size);
//    memcpy((void*)msg_sp->points.data(), msg_addr, size);
//    callback(msg_sp);
//    tzcHelper_all_pc.find(topic_name)->second.ReleaseReadLock();
//  }
//}

}  // namespace cm
