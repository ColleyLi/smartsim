#pragma once

#include <chrono>

#include "ara/exec/execution_client.h"
#include "ascend_hal.h"
#include "caic_interface.h"
#include "cm/ap/msgs/adaptor.h"
#include "cm/node_handle.h"

using Str = ::stoic::msgs::common::Str;
using Vec3 = ::stoic::proto::geometry::Vector3;
using TPsd = ::stoic::proto::smartview::TargetPSD;
using USS = apollo::drivers::ChassisDetail;
using Mes = ::google::protobuf::Message;
using Pcd = ::caic_sensor::PointCloudTypeArray;
using Radar = ::apollo::drivers::ContiRadar;
using Odom = ::caic_sensor::caic_ins::Odometry;
using CorrImu = ::caic_sensor::caic_ins::CorrImu;
using Inspvax = ::caic_sensor::caic_ins::Inspavx;
using GpsFix = ::caic_sensor::caic_ins::GpsFix;
using Inspva = ::caic_sensor::caic_ins::Inspva;
using BestGnssPos = ::caic_sensor::caic_ins::BestGnssPos;
using Imu = ::caic_sensor::Imu;
using CanBus = ::caic_sensor::Canbus;
using Control = ::caic_control::ControlCommand;

namespace cm {
struct NodeHandleImpl {
  NodeHandleImpl(const std::string&) {}

  template <typename _Msg>
  void Callback(void (*callback)(const _Msg&), std::string topic);

  template <typename _Msg, bool _Adaptive>
  void Callback(void (*callback)(const _Msg&), std::string topic);

  template <typename _Msg>
  void CallbackBoost(const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                     std::string topic);

  template <typename _Msg, bool _Adaptive>
  void CallbackBoost(const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                     std::string topic);

  template <typename _Msg>
  void SrvCallback(void (*callback)(const _Msg&), std::string topic);

  template <typename _Msg, bool _Adaptive>
  void SrvCallback(void (*callback)(const _Msg&), std::string topic);

  template <typename _Msg>
  void SrvCallbackBoost(const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                        std::string topic);

  template <typename _Msg, bool _Adaptive>
  void SrvCallbackBoost(const boost::function<void(std::shared_ptr<_Msg>)>& callback,
                        std::string topic);

  std::unordered_map<std::string, RawDataProxyPtr> raw_data_proxy_ptr_map;
  std::unordered_map<std::string, PcdProxyPtr> pcd_proxy_ptr_map;
  std::unordered_map<std::string, ImageProxyPtr> de_image_proxy_ptr_map;
  std::unordered_map<std::string, RadarProxyPtr> radar_proxy_ptr_map;
  std::unordered_map<std::string, InsProxyPtr> ins_proxy_ptr_map;
  std::unordered_map<std::string, ImuProxyPtr> imu_proxy_ptr_map;
  std::unordered_map<std::string, ChassisProxyPtr> chassis_proxy_ptr_map;
  std::unordered_map<std::string, StrProxyPtr> str_proxy_ptr_map;

  std::unordered_map<std::string, RawDataSkeletonPtr> raw_data_skeleton_ptr_map;
  std::unordered_map<std::string, PcdSkeletonPtr> pcd_skeleton_ptr_map;
  std::unordered_map<std::string, ImageSkeletonPtr> image_skeleton_ptr_map;
  std::unordered_map<std::string, RadarSkeletonPtr> radar_skeleton_ptr_map;
  std::unordered_map<std::string, InsSkeletonPtr> ins_skeleton_ptr_map;
  std::unordered_map<std::string, ImuSkeletonPtr> imu_skeleton_ptr_map;
  std::unordered_map<std::string, ChassisSkeletonPtr> chassis_skeleton_ptr_map;
  std::unordered_map<std::string, ControlSkeletonPtr> control_skeleton_ptr_map;
  std::unordered_map<std::string, StrSkeletonPtr> str_skeleton_ptr_map;

  std::unordered_map<std::string, ara::com::ServiceHandleContainer<RawDataProxy::HandleType>>
      raw_data_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<StrProxy::HandleType>>
      str_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<PcdProxy::HandleType>>
      pcd_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<ImageProxy::HandleType>>
      de_image_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<RadarProxy::HandleType>>
      radar_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<InsProxy::HandleType>>
      ins_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<ImuProxy::HandleType>>
      imu_handles_map;
  std::unordered_map<std::string, ara::com::ServiceHandleContainer<ChassisProxy::HandleType>>
      chassis_handles_map;

  std::unordered_map<std::string, ara::com::FindServiceHandle> srv_handle_map;

  std::mutex raw_proxy_mutex;
  std::mutex pcd_proxy_mutex;
  std::mutex image_proxy_mutex;
  std::mutex radar_proxy_mutex;
  std::mutex odom_proxy_mutex;
  std::mutex corr_imu_proxy_mutex;
  std::mutex inspvax_proxy_mutex;
  std::mutex imu_proxy_mutex;
  std::mutex chassis_proxy_mutex;
  std::mutex str_sv_proxy_mutex;

  ara::exec::ExecutionClient exec_client_image;
};

template <>
inline void NodeHandleImpl::Callback<Vec3>(void (*callback)(const Vec3&), std::string topic) {
  using APType = typename AdaptorTraits<Vec3, true>::APType;
  raw_data_proxy_ptr_map[topic]->rawdataEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<Vec3, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<Pcd>(void (*callback)(const Pcd&), std::string topic) {
  using APType = typename AdaptorTraits<Pcd, true>::APType;
  pcd_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<Pcd, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<::caic_sensor::Image>(void (*callback)(const ::caic_sensor::Image&), std::string topic) {
  using APType = typename AdaptorTraits<::caic_sensor::Image, true>::APType;
  de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<::caic_sensor::Image, APType, true>(*ap_msg, callback);
        (void)halMbufFree(reinterpret_cast<Mbuf*>(ap_msg->RawData));
      });
}

// template <>
// inline void NodeHandleImpl::Callback<Radar>(void (*callback)(const Radar&), std::string topic) {
//   using APType = typename AdaptorTraits<Radar, true>::APType;
//   radar_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
//       [this, callback](ara::com::SamplePtr<APType const> ptr) {
//         auto ap_msg = ptr.Get();
//         callbackAdaptor<Radar, APType, true>(*ap_msg, callback);
//       });
// }

template <>
inline void NodeHandleImpl::Callback<Odom>(void (*callback)(const Odom&), std::string topic) {
  using APType = typename AdaptorTraits<Odom, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<Odom, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<CorrImu>(void (*callback)(const CorrImu&), std::string topic) {
  using APType = typename AdaptorTraits<CorrImu, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<CorrImu, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<Inspvax>(void (*callback)(const Inspvax&), std::string topic) {
  using APType = typename AdaptorTraits<Inspvax, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<Inspvax, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<Imu>(void (*callback)(const Imu&), std::string topic) {
  using APType = typename AdaptorTraits<Imu, true>::APType;
  imu_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<Imu, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::Callback<CanBus>(void (*callback)(const CanBus&), std::string topic) {
  using APType = typename AdaptorTraits<CanBus, true>::APType;
  chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<CanBus, APType, true>(*ap_msg, callback);
      });
}

template <typename _Msg, bool _Adaptive>
inline void NodeHandleImpl::Callback(void (*callback)(const _Msg&), std::string topic) {
  using APType = typename AdaptorTraits<_Msg, false>::APType;
  str_proxy_ptr_map[topic]->stringEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptor<_Msg, APType, false>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<Vec3>(
    const boost::function<void(std::shared_ptr<Vec3>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<Vec3, true>::APType;
  raw_data_proxy_ptr_map[topic]->rawdataEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<Vec3, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<Pcd>(
    const boost::function<void(std::shared_ptr<Pcd>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<Pcd, true>::APType;
  pcd_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<Pcd, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<::caic_sensor::Image>(
    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<::caic_sensor::Image, true>::APType;
  de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<::caic_sensor::Image, APType, true>(*ap_msg, callback);
        (void)halMbufFree(reinterpret_cast<Mbuf*>(ap_msg->RawData));
      });
}

// template <>
// inline void NodeHandleImpl::CallbackBoost<Radar>(
//     const boost::function<void(std::shared_ptr<Radar>)>& callback, std::string topic) {
//   using APType = typename AdaptorTraits<Radar, true>::APType;
//   radar_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
//       [this, callback](ara::com::SamplePtr<APType const> ptr) {
//         auto ap_msg = ptr.Get();
//         callbackAdaptorBoost<Radar, APType, true>(*ap_msg, callback);
//       });
// }

template <>
inline void NodeHandleImpl::CallbackBoost<Odom>(
    const boost::function<void(std::shared_ptr<Odom>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<Odom, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<Odom, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<CorrImu>(
    const boost::function<void(std::shared_ptr<CorrImu>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<CorrImu, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<CorrImu, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<Inspvax>(
    const boost::function<void(std::shared_ptr<Inspvax>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<Inspvax, true>::APType;
  ins_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<Inspvax, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<Imu>(
    const boost::function<void(std::shared_ptr<Imu>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<Imu, true>::APType;
  imu_proxy_ptr_map[topic]->mdcEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<Imu, APType, true>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::CallbackBoost<CanBus>(
    const boost::function<void(std::shared_ptr<CanBus>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<CanBus, true>::APType;
  chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<CanBus, APType, true>(*ap_msg, callback);
      });
}

template <typename _Msg, bool _Adaptive>
inline void NodeHandleImpl::CallbackBoost(
    const boost::function<void(std::shared_ptr<_Msg>)>& callback, std::string topic) {
  using APType = typename AdaptorTraits<_Msg, _Adaptive>::APType;
  str_proxy_ptr_map[topic]->stringEvent.GetNewSamples(
      [this, callback](ara::com::SamplePtr<APType const> ptr) {
        auto ap_msg = ptr.Get();
        callbackAdaptorBoost<_Msg, APType, _Adaptive>(*ap_msg, callback);
      });
}

template <>
inline void NodeHandleImpl::SrvCallback<Vec3>(void (*callback)(const Vec3&), std::string topic) {
  for (auto& handle : raw_data_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(raw_proxy_mutex);
    auto resultToken = RawDataProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      raw_data_proxy_ptr_map[topic] =
          std::make_shared<RawDataProxy>(std::move(resultToken).Value());
      raw_data_proxy_ptr_map[topic]->rawdataEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<Vec3>(callback, topic); });
      const size_t maxCount = 1U;
      raw_data_proxy_ptr_map[topic]->rawdataEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<Pcd>(void (*callback)(const Pcd&), std::string topic) {
  for (auto& handle : pcd_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(pcd_proxy_mutex);
    auto resultToken = PcdProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      pcd_proxy_ptr_map[topic] = std::make_shared<PcdProxy>(std::move(resultToken).Value());
      pcd_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<Pcd>(callback, topic); });
      const size_t maxCount = 1U;
      pcd_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<::caic_sensor::Image>(void (*callback)(const ::caic_sensor::Image&),
                                                 std::string topic) {
  if (de_image_proxy_ptr_map[topic] == nullptr) {
    for (auto& handle : de_image_handles_map[topic]) {
      std::lock_guard<std::mutex> lock(image_proxy_mutex);
      // ara::core::StringView instance_id_sv = handle.GetInstanceId().ToString();
      // int32_t instance_id = (instance_id_sv.at(0) - '0') * 10 + (instance_id_sv.at(1) - '0');
      // std::cout << "instance_id: " << instance_id << std::endl;
      de_image_proxy_ptr_map[topic] = std::make_shared<ImageProxy>(handle);
      de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<::caic_sensor::Image>(callback, topic); });
      const size_t maxCount = 1U;
      de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.Subscribe(maxCount);
      // Mbuf need to report execution state
      auto res = exec_client_image.ReportExecutionState(ara::exec::ExecutionState::kRunning);
    }
  }
}

// template <>
// inline void NodeHandleImpl::SrvCallback<Radar>(void (*callback)(const Radar&), std::string topic)
// {
//   for (auto& handle : radar_handles_map[topic]) {
//     std::lock_guard<std::mutex> lock(radar_proxy_mutex);
//     auto resultToken = RadarProxy::Preconstruct(handle);
//     if (resultToken.HasValue()) {
//       radar_proxy_ptr_map[topic] = std::make_shared<RadarProxy>(std::move(resultToken).Value());
//       radar_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
//           [this, callback, topic]() { Callback<Radar>(callback, topic); });
//       const size_t maxCount = 1U;
//       radar_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
//       break;
//     }
//   }
// }

template <>
inline void NodeHandleImpl::SrvCallback<Odom>(void (*callback)(const Odom&), std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(odom_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<Odom>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<CorrImu>(void (*callback)(const CorrImu&),
                                                 std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(corr_imu_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<CorrImu>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<Inspvax>(void (*callback)(const Inspvax&),
                                                 std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(inspvax_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<Inspvax>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<Imu>(void (*callback)(const Imu&), std::string topic) {
  for (auto& handle : imu_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(imu_proxy_mutex);
    auto resultToken = ImuProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      imu_proxy_ptr_map[topic] = std::make_shared<ImuProxy>(std::move(resultToken).Value());
      imu_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<Imu>(callback, topic); });
      const size_t maxCount = 1U;
      imu_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallback<CanBus>(void (*callback)(const CanBus&),
                                                std::string topic) {
  for (auto& handle : chassis_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(chassis_proxy_mutex);
    auto resultToken = ChassisProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      chassis_proxy_ptr_map[topic] = std::make_shared<ChassisProxy>(std::move(resultToken).Value());
      chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<CanBus>(callback, topic); });
      const size_t maxCount = 1U;
      chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <typename _Msg, bool _Adaptive>
inline void NodeHandleImpl::SrvCallback(void (*callback)(const _Msg&), std::string topic) {
  for (auto& handle : str_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(str_sv_proxy_mutex);
    auto resultToken = StrProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      str_proxy_ptr_map[topic] = std::make_shared<StrProxy>(std::move(resultToken).Value());
      str_proxy_ptr_map[topic]->stringEvent.SetReceiveHandler(
          [this, callback, topic]() { Callback<_Msg, false>(callback, topic); });
      const size_t maxCount = 1U;
      str_proxy_ptr_map[topic]->stringEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<Vec3>(
    const boost::function<void(std::shared_ptr<Vec3>)>& callback, std::string topic) {
  for (auto& handle : raw_data_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(raw_proxy_mutex);
    auto resultToken = RawDataProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      raw_data_proxy_ptr_map[topic] =
          std::make_shared<RawDataProxy>(std::move(resultToken).Value());
      raw_data_proxy_ptr_map[topic]->rawdataEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<Vec3>(callback, topic); });
      const size_t maxCount = 1U;
      raw_data_proxy_ptr_map[topic]->rawdataEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<Pcd>(
    const boost::function<void(std::shared_ptr<Pcd>)>& callback, std::string topic) {
  for (auto& handle : pcd_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(pcd_proxy_mutex);
    auto resultToken = PcdProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      pcd_proxy_ptr_map[topic] = std::make_shared<PcdProxy>(std::move(resultToken).Value());
      pcd_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<Pcd>(callback, topic); });
      const size_t maxCount = 1U;
      pcd_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<::caic_sensor::Image>(
    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback, std::string topic) {
  if (de_image_proxy_ptr_map[topic] == nullptr) {
    for (auto& handle : de_image_handles_map[topic]) {
      std::lock_guard<std::mutex> lock(image_proxy_mutex);
      de_image_proxy_ptr_map[topic] = std::make_shared<ImageProxy>(handle);
      de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<::caic_sensor::Image>(callback, topic); });
      const size_t maxCount = 1U;
      de_image_proxy_ptr_map[topic]->cameraDecodedMbufEvent.Subscribe(maxCount);
      // Mbuf need to report execution state
      auto res = exec_client_image.ReportExecutionState(ara::exec::ExecutionState::kRunning);
    }
  }
}

// template <>
// inline void NodeHandleImpl::SrvCallbackBoost<Radar>(
//     const boost::function<void(std::shared_ptr<Radar>)>& callback, std::string topic) {
//   for (auto& handle : radar_handles_map[topic]) {
//     std::lock_guard<std::mutex> lock(radar_proxy_mutex);
//     auto resultToken = RadarProxy::Preconstruct(handle);
//     if (resultToken.HasValue()) {
//       radar_proxy_ptr_map[topic] = std::make_shared<RadarProxy>(std::move(resultToken).Value());
//       radar_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
//           [this, callback, topic]() { CallbackBoost<Radar>(callback, topic); });
//       const size_t maxCount = 1U;
//       radar_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
//       break;
//     }
//   }
// }

template <>
inline void NodeHandleImpl::SrvCallbackBoost<Odom>(
    const boost::function<void(std::shared_ptr<Odom>)>& callback, std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(odom_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<Odom>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<CorrImu>(
    const boost::function<void(std::shared_ptr<CorrImu>)>& callback, std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(corr_imu_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<CorrImu>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<Inspvax>(
    const boost::function<void(std::shared_ptr<Inspvax>)>& callback, std::string topic) {
  for (auto& handle : ins_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(inspvax_proxy_mutex);
    auto resultToken = InsProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      ins_proxy_ptr_map[topic] = std::make_shared<InsProxy>(std::move(resultToken).Value());
      ins_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<Inspvax>(callback, topic); });
      const size_t maxCount = 1U;
      ins_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<Imu>(
    const boost::function<void(std::shared_ptr<Imu>)>& callback, std::string topic) {
  for (auto& handle : imu_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(imu_proxy_mutex);
    auto resultToken = ImuProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      imu_proxy_ptr_map[topic] = std::make_shared<ImuProxy>(std::move(resultToken).Value());
      imu_proxy_ptr_map[topic]->mdcEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<Imu>(callback, topic); });
      const size_t maxCount = 1U;
      imu_proxy_ptr_map[topic]->mdcEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <>
inline void NodeHandleImpl::SrvCallbackBoost<CanBus>(
    const boost::function<void(std::shared_ptr<CanBus>)>& callback, std::string topic) {
  for (auto& handle : chassis_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(chassis_proxy_mutex);
    auto resultToken = ChassisProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      chassis_proxy_ptr_map[topic] = std::make_shared<ChassisProxy>(std::move(resultToken).Value());
      chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<CanBus>(callback, topic); });
      const size_t maxCount = 1U;
      chassis_proxy_ptr_map[topic]->CaicSensorCanbusEvent.Subscribe(maxCount);
      break;
    }
  }
}

template <typename _Msg, bool _Adaptive>
inline void NodeHandleImpl::SrvCallbackBoost(
    const boost::function<void(std::shared_ptr<_Msg>)>& callback, std::string topic) {
  for (auto& handle : str_handles_map[topic]) {
    std::lock_guard<std::mutex> lock(str_sv_proxy_mutex);
    auto resultToken = StrProxy::Preconstruct(handle);
    if (resultToken.HasValue()) {
      str_proxy_ptr_map[topic] = std::make_shared<StrProxy>(std::move(resultToken).Value());
      str_proxy_ptr_map[topic]->stringEvent.SetReceiveHandler(
          [this, callback, topic]() { CallbackBoost<_Msg, _Adaptive>(callback, topic); });
      const size_t maxCount = 1U;
      str_proxy_ptr_map[topic]->stringEvent.Subscribe(maxCount);
      break;
    }
  }
}

NodeHandle::NodeHandle(const std::string& name) { impl_ = std::make_unique<NodeHandleImpl>(name); }

NodeHandle::~NodeHandle() = default;

template <typename _Msg, bool _Adaptive = false>
inline Publisher NodeHandle::advertise(const std::string& topic, size_t /*queue_size*/) {
  auto resultToken = StrSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->str_skeleton_ptr_map[topic] =
        std::make_shared<StrSkeleton>(std::move(resultToken).Value());
    impl_->str_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(
      std::make_unique<PublisherImpl>(std::move(impl_->str_skeleton_ptr_map[topic]), topic));
}

template <>
inline Publisher NodeHandle::advertise<Vec3, true>(const std::string& topic,
                                                   size_t /*queue_size*/) {
  auto resultToken = RawDataSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                                   ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->raw_data_skeleton_ptr_map[topic] =
        std::make_shared<RawDataSkeleton>(std::move(resultToken).Value());
    impl_->raw_data_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(RawDataSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<Pcd, true>(const std::string& topic, size_t /*queue_size*/) {
  auto resultToken = PcdSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->pcd_skeleton_ptr_map[topic] =
        std::make_shared<PcdSkeleton>(std::move(resultToken).Value());
    impl_->pcd_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(PcdSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<::caic_sensor::Image, true>(const std::string& topic,
                                                      size_t /*queue_size*/) {
  auto resultToken = ImageSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                                 ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->image_skeleton_ptr_map[topic] =
        std::make_shared<ImageSkeleton>(std::move(resultToken).Value());
    impl_->image_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(ImageSkeletonPtr())));
}

// template <>
// inline Publisher NodeHandle::advertise<Radar, true>(const std::string& topic,
//                                                     size_t /*queue_size*/) {
//   auto resultToken = RadarSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
//                                                  ara::com::MethodCallProcessingMode::kEvent);
//   if (resultToken.HasValue()) {
//     impl_->radar_skeleton_ptr_map[topic] =
//         std::make_shared<RadarSkeleton>(std::move(resultToken).Value());
//     impl_->radar_skeleton_ptr_map[topic]->OfferService();
//   }
//   return Publisher(std::make_unique<PublisherImpl>(std::move(RadarSkeletonPtr())));
// }

template <>
inline Publisher NodeHandle::advertise<Odom, true>(const std::string& topic,
                                                   size_t /*queue_size*/) {
  auto resultToken = InsSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->ins_skeleton_ptr_map[topic] =
        std::make_shared<InsSkeleton>(std::move(resultToken).Value());
    impl_->ins_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(InsSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<CorrImu, true>(const std::string& topic,
                                                      size_t /*queue_size*/) {
  auto resultToken = InsSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->ins_skeleton_ptr_map[topic] =
        std::make_shared<InsSkeleton>(std::move(resultToken).Value());
    impl_->ins_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(InsSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<Inspvax, true>(const std::string& topic,
                                                      size_t /*queue_size*/) {
  auto resultToken = InsSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->ins_skeleton_ptr_map[topic] =
        std::make_shared<InsSkeleton>(std::move(resultToken).Value());
    impl_->ins_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(InsSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<Imu, true>(const std::string& topic, size_t /*queue_size*/) {
  auto resultToken = ImuSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                               ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->imu_skeleton_ptr_map[topic] =
        std::make_shared<ImuSkeleton>(std::move(resultToken).Value());
    impl_->imu_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(std::make_unique<PublisherImpl>(std::move(ImuSkeletonPtr())));
}

template <>
inline Publisher NodeHandle::advertise<Control, true>(const std::string& topic,
                                                      size_t /*queue_size*/) {
  auto resultToken = ControlSkeleton::Preconstruct(ara::core::InstanceSpecifier(topic.c_str()),
                                                   ara::com::MethodCallProcessingMode::kEvent);
  if (resultToken.HasValue()) {
    impl_->control_skeleton_ptr_map[topic] =
        std::make_shared<ControlSkeleton>(std::move(resultToken).Value());
    impl_->control_skeleton_ptr_map[topic]->OfferService();
  }
  return Publisher(
      std::make_unique<PublisherImpl>(std::move(impl_->control_skeleton_ptr_map[topic])));
}

template <>
inline Subscriber NodeHandle::subscribe<Vec3, true>(const std::string& topic, size_t /*queue_size*/,
                                                    void (*callback)(const Vec3&)) {
  impl_->srv_handle_map[topic] = RawDataProxy::StartFindService(
      [this, callback, topic](
          const ara::com::ServiceHandleContainer<RawDataProxy::HandleType>& handles,
          const ara::com::FindServiceHandle& /*handler*/) {
        impl_->raw_data_handles_map[topic] = handles;
        impl_->SrvCallback<Vec3>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));

  return Subscriber(std::make_unique<SubscriberImpl>(std::move(RawDataProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<Pcd, true>(const std::string& topic, size_t /*queue_size*/,
                                                   void (*callback)(const Pcd&)) {
  impl_->srv_handle_map[topic] = PcdProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<PcdProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->pcd_handles_map[topic] = handles;
        impl_->SrvCallback<Pcd>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(PcdProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<::caic_sensor::Image, true>(const std::string& topic,
                                                       size_t /*queue_size*/,
                                                       void (*callback)(const ::caic_sensor::Image&)) {
  impl_->srv_handle_map[topic] = ImageProxy::StartFindService(
      [this, callback, topic](
          const ara::com::ServiceHandleContainer<ImageProxy::HandleType>& handles,
          const ara::com::FindServiceHandle& /*handler*/) {
        impl_->de_image_handles_map[topic] = handles;
        impl_->SrvCallback<::caic_sensor::Image>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ImageProxyPtr())));
}

// template <>
// inline Subscriber NodeHandle::subscribe<Radar, true>(const std::string& topic,
//                                                      size_t /*queue_size*/,
//                                                      void (*callback)(const Radar&)) {
//   impl_->srv_handle_map[topic] = RadarProxy::StartFindService(
//       [this, callback, topic](
//           const ara::com::ServiceHandleContainer<RadarProxy::HandleType>& handles,
//           const ara::com::FindServiceHandle& /*handler*/) {
//         impl_->radar_handles_map[topic] = handles;
//         impl_->SrvCallback<Radar>(callback, topic);
//       },
//       ara::core::InstanceSpecifier(topic.c_str()));
//   return Subscriber(std::make_unique<SubscriberImpl>(std::move(RadarProxyPtr())));
// }

template <>
inline Subscriber NodeHandle::subscribe<Odom, true>(const std::string& topic, size_t /*queue_size*/,
                                                    void (*callback)(const Odom&)) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallback<Odom>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<CorrImu, true>(const std::string& topic,
                                                       size_t /*queue_size*/,
                                                       void (*callback)(const CorrImu&)) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallback<CorrImu>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<Inspvax, true>(const std::string& topic,
                                                       size_t /*queue_size*/,
                                                       void (*callback)(const Inspvax&)) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallback<Inspvax>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<Imu, true>(const std::string& topic, size_t /*queue_size*/,
                                                   void (*callback)(const Imu&)) {
  std::string imu_topic = "/imu_sub";
  impl_->srv_handle_map[topic] = ImuProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<ImuProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->imu_handles_map[topic] = handles;
        impl_->SrvCallback<Imu>(callback, topic);
      },
      ara::core::InstanceSpecifier(imu_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ImuProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<CanBus, true>(const std::string& topic,
                                                      size_t /*queue_size*/,
                                                      void (*callback)(const CanBus&)) {
  impl_->srv_handle_map[topic] = ChassisProxy::StartFindService(
      [this, callback, topic](
          const ara::com::ServiceHandleContainer<ChassisProxy::HandleType>& handles,
          const ara::com::FindServiceHandle& /*handler*/) {
        impl_->chassis_handles_map[topic] = handles;
        impl_->SrvCallback<CanBus>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ChassisProxyPtr())));
}

template <typename _Msg, bool _Adaptive = false>
inline Subscriber NodeHandle::subscribe(const std::string& topic, size_t /*queue_size*/,
                                        void (*callback)(const _Msg&)) {
  if constexpr (std::is_same_v<_Msg, DetSV> || std::is_same_v<_Msg, TldSV> ||
                std::is_same_v<_Msg, CtlSV> || std::is_same_v<_Msg, LocSV> ||
                std::is_same_v<_Msg, PercSV> || std::is_same_v<_Msg, VisSV> ||
                std::is_same_v<_Msg, PredSV> || std::is_same_v<_Msg, PlanSV> ||
                std::is_same_v<_Msg, RoutSV> || std::is_same_v<_Msg, PcdSV> ||
                std::is_same_v<_Msg, USS>) {
    impl_->srv_handle_map[topic] = StrProxy::StartFindService(
        [this, callback, topic](
            const ara::com::ServiceHandleContainer<StrProxy::HandleType>& handles,
            const ara::com::FindServiceHandle& /*handler*/) {
          impl_->str_handles_map[topic] = handles;
          impl_->SrvCallback<_Msg, false>(callback, topic);
        },
        ara::core::InstanceSpecifier(topic.c_str()));
    return Subscriber(std::make_unique<SubscriberImpl>(std::move(StrProxyPtr())));
  } else {
    return Subscriber(std::make_unique<SubscriberImpl>());
  }
}

template <>
inline Subscriber NodeHandle::subscribe<Pcd, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<Pcd>)>& callback) {
  impl_->srv_handle_map[topic] = PcdProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<PcdProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->pcd_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<Pcd>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(PcdProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<::caic_sensor::Image, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<::caic_sensor::Image>)>& callback) {
  impl_->srv_handle_map[topic] = ImageProxy::StartFindService(
      [this, callback, topic](
          const ara::com::ServiceHandleContainer<ImageProxy::HandleType>& handles,
          const ara::com::FindServiceHandle& /*handler*/) {
        impl_->de_image_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<::caic_sensor::Image>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ImageProxyPtr())));
}

// template <>
// inline Subscriber NodeHandle::subscribe<Radar, true>(
//     const std::string& topic, size_t /*queue_size*/,
//     const boost::function<void(std::shared_ptr<Radar>)>& callback) {
//   impl_->srv_handle_map[topic] = RadarProxy::StartFindService(
//       [this, callback, topic](
//           const ara::com::ServiceHandleContainer<RadarProxy::HandleType>& handles,
//           const ara::com::FindServiceHandle& /*handler*/) {
//         impl_->radar_handles_map[topic] = handles;
//         impl_->SrvCallbackBoost<Radar>(callback, topic);
//       },
//       ara::core::InstanceSpecifier(topic.c_str()));
//   return Subscriber(std::make_unique<SubscriberImpl>(std::move(RadarProxyPtr())));
// }

template <>
inline Subscriber NodeHandle::subscribe<Odom, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<Odom>)>& callback) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<Odom>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<CorrImu, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<CorrImu>)>& callback) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<CorrImu>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<Inspvax, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<Inspvax>)>& callback) {
  std::string ins_topic = "/ins_sub";
  impl_->srv_handle_map[topic] = InsProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<InsProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->ins_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<Inspvax>(callback, topic);
      },
      ara::core::InstanceSpecifier(ins_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(InsProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<GpsFix, true>(
    const std::string& /*topic*/, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<GpsFix>)>& /*callback*/) {
  return Subscriber(std::make_unique<SubscriberImpl>());
}

template <>
inline Subscriber NodeHandle::subscribe<Inspva, true>(
    const std::string& /*topic*/, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<Inspva>)>& /*callback*/) {
  return Subscriber(std::make_unique<SubscriberImpl>());
}

template <>
inline Subscriber NodeHandle::subscribe<BestGnssPos, true>(
    const std::string& /*topic*/, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<BestGnssPos>)>& /*callback*/) {
  return Subscriber(std::make_unique<SubscriberImpl>());
}

template <>
inline Subscriber NodeHandle::subscribe<Imu, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<Imu>)>& callback) {
  std::string imu_topic = "/imu_sub";
  impl_->srv_handle_map[topic] = ImuProxy::StartFindService(
      [this, callback, topic](const ara::com::ServiceHandleContainer<ImuProxy::HandleType>& handles,
                              const ara::com::FindServiceHandle& /*handler*/) {
        impl_->imu_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<Imu>(callback, topic);
      },
      ara::core::InstanceSpecifier(imu_topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ImuProxyPtr())));
}

template <>
inline Subscriber NodeHandle::subscribe<CanBus, true>(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<CanBus>)>& callback) {
  impl_->srv_handle_map[topic] = ChassisProxy::StartFindService(
      [this, callback, topic](
          const ara::com::ServiceHandleContainer<ChassisProxy::HandleType>& handles,
          const ara::com::FindServiceHandle& /*handler*/) {
        impl_->chassis_handles_map[topic] = handles;
        impl_->SrvCallbackBoost<CanBus>(callback, topic);
      },
      ara::core::InstanceSpecifier(topic.c_str()));
  return Subscriber(std::make_unique<SubscriberImpl>(std::move(ChassisProxyPtr())));
}

template <typename _Msg, bool _Adaptive = false>
inline Subscriber NodeHandle::subscribe(
    const std::string& topic, size_t /*queue_size*/,
    const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  if constexpr (std::is_same_v<_Msg, DetSV> || std::is_same_v<_Msg, TldSV> ||
                std::is_same_v<_Msg, CtlSV> || std::is_same_v<_Msg, LocSV> ||
                std::is_same_v<_Msg, PercSV> || std::is_same_v<_Msg, VisSV> ||
                std::is_same_v<_Msg, SurrSV> || std::is_same_v<_Msg, PredSV> ||
                std::is_same_v<_Msg, PlanSV> || std::is_same_v<_Msg, RoutSV> ||
                std::is_same_v<_Msg, PcdSV> || std::is_same_v<_Msg, MarkSV> ||
                std::is_same_v<_Msg, ImgMarkSV> || std::is_same_v<_Msg, PsSV> ||
                std::is_same_v<_Msg, TPsd> || std::is_same_v<_Msg, ImgSV> ||
                std::is_same_v<_Msg, Str> || std::is_same_v<_Msg, USS> ||
                std::is_same_v<_Msg, Radar>) {
    impl_->srv_handle_map[topic] = StrProxy::StartFindService(
        [this, callback, topic](
            const ara::com::ServiceHandleContainer<StrProxy::HandleType>& handles,
            const ara::com::FindServiceHandle& /*handler*/) {
          impl_->str_handles_map[topic] = handles;
          impl_->SrvCallbackBoost<_Msg, false>(callback, topic);
        },
        ara::core::InstanceSpecifier(topic.c_str()));
    return Subscriber(std::make_unique<SubscriberImpl>(std::move(StrProxyPtr())));
  } else if constexpr (std::is_same_v<_Msg, ::caic_localization::LocalizationEstimation> ||
                       std::is_same_v<_Msg, ::caic_perception::PerceptionTrafficLights> ||
                       std::is_same_v<_Msg, ::caic_perception::PerceptionObjects> ||
                       std::is_same_v<_Msg, ::stoic::msgs::perception::ObjsPostType> ||
                       std::is_same_v<_Msg, ::stoic::msgs::perception::LaneLinePostType> ||
                       std::is_same_v<_Msg, ::caic_prediction::PredictionObjects> ||
                       std::is_same_v<_Msg, ::caic_planning::PlanningResult>) {
    impl_->srv_handle_map[topic] = StrProxy::StartFindService(
        [this, callback, topic](
            const ara::com::ServiceHandleContainer<StrProxy::HandleType>& handles,
            const ara::com::FindServiceHandle& /*handler*/) {
          impl_->str_handles_map[topic] = handles;
          impl_->SrvCallbackBoost<_Msg, true>(callback, topic);
        },
        ara::core::InstanceSpecifier(topic.c_str()));
    return Subscriber(std::make_unique<SubscriberImpl>(std::move(StrProxyPtr())));
  } else {
    return Subscriber(std::make_unique<SubscriberImpl>());
  }
}

bool NodeHandle::getParam(const std::string&, std::string&) {
  // return impl_->instance.getParam(name, value);
  return true;
}

}  // namespace cm
