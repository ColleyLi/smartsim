#pragma once

#include "caic_interface.h"
#include "cm/publisher.h"
#include "common/log/Logger.h"

using Str = ::stoic::msgs::common::Str;
using Vec3 = ::stoic::proto::geometry::Vector3;
using Mes = ::google::protobuf::Message;
using Pcd = ::caic_sensor::PointCloudTypeArray;
using Radar = ::apollo::drivers::ContiRadar;
using Odom = ::caic_sensor::caic_ins::Odometry;
using CorrImu = ::caic_sensor::caic_ins::CorrImu;
using Inspvax = ::caic_sensor::caic_ins::Inspavx;
using Imu = ::caic_sensor::Imu;
using Control = ::caic_control::ControlCommand;
using TPsd = ::stoic::proto::smartview::TargetPSD;

namespace cm {

static const char* LOG_TAG = "cm";

struct PublisherImpl {
  PublisherImpl() {}
  PublisherImpl(RawDataSkeletonPtr&& raw_publisher)
      : pub_raw(std::forward<RawDataSkeletonPtr>(raw_publisher)) {}
  PublisherImpl(PcdSkeletonPtr&& pcd_publisher)
      : pub_pcd(std::forward<PcdSkeletonPtr>(pcd_publisher)) {}
  PublisherImpl(ImageSkeletonPtr&& image_publisher)
      : pub_image(std::forward<ImageSkeletonPtr>(image_publisher)) {}
  PublisherImpl(RadarSkeletonPtr&& radar_publisher)
      : pub_radar(std::forward<RadarSkeletonPtr>(radar_publisher)) {}
  PublisherImpl(InsSkeletonPtr&& ins_publisher)
      : pub_ins(std::forward<InsSkeletonPtr>(ins_publisher)) {}
  PublisherImpl(ImuSkeletonPtr&& imu_publisher)
      : pub_imu(std::forward<ImuSkeletonPtr>(imu_publisher)) {}
  PublisherImpl(ControlSkeletonPtr&& ctl_publisher)
      : pub_ctl(std::forward<ControlSkeletonPtr>(ctl_publisher)) {}
  PublisherImpl(StrSkeletonPtr&& str_publisher, const std::string& str_topic = "") {
    pub_str_map[str_topic] = std::forward<StrSkeletonPtr>(str_publisher);
  }

  RawDataSkeletonPtr pub_raw;
  PcdSkeletonPtr pub_pcd;
  ImageSkeletonPtr pub_image;
  RadarSkeletonPtr pub_radar;
  InsSkeletonPtr pub_ins;
  ImuSkeletonPtr pub_imu;
  ControlSkeletonPtr pub_ctl;
  std::unordered_map<std::string, StrSkeletonPtr> pub_str_map;
};

Publisher::Publisher(std::unique_ptr<PublisherImpl>&& impl)
    : impl_(std::forward<std::unique_ptr<PublisherImpl>>(impl)) {}

Publisher::~Publisher() = default;

template <>
inline void Publisher::publish<Vec3, true>(const Vec3& msg) {
  using APType = typename AdaptorTraits<Vec3, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_raw->rawdataEvent.Allocate();
  AdaptorTraits<Vec3, true>::convert(msg, data.get());
  impl_->pub_raw->rawdataEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<Pcd, true>(const Pcd& msg) {
  using APType = typename AdaptorTraits<Pcd, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_pcd->mdcEvent.Allocate();
  AdaptorTraits<Pcd, true>::convert(msg, data.get());
  impl_->pub_pcd->mdcEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<::caic_sensor::Image, true>(const ::caic_sensor::Image& msg) {
  using APType = typename AdaptorTraits<::caic_sensor::Image, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_image->cameraDecodedMbufEvent.Allocate();
  AdaptorTraits<::caic_sensor::Image, true>::convert(msg, data.get());
  impl_->pub_image->cameraDecodedMbufEvent.Send(std::move(data));
}

// template <>
// inline void Publisher::publish<Radar, true>(const Radar& msg) {
//   using APType = typename AdaptorTraits<Radar, true>::APType;
//   using APTypePtr = std::unique_ptr<APType>;
//   APTypePtr data = impl_->pub_radar->mdcEvent.Allocate();
//   AdaptorTraits<Radar, true>::convert(msg, data.get());
//   impl_->pub_radar->mdcEvent.Send(std::move(data));
// }

template <>
inline void Publisher::publish<Odom, true>(const Odom& msg) {
  using APType = typename AdaptorTraits<Odom, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_ins->mdcEvent.Allocate();
  AdaptorTraits<Odom, true>::convert(msg, data.get());
  impl_->pub_ins->mdcEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<CorrImu, true>(const CorrImu& msg) {
  using APType = typename AdaptorTraits<CorrImu, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_ins->mdcEvent.Allocate();
  AdaptorTraits<CorrImu, true>::convert(msg, data.get());
  impl_->pub_ins->mdcEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<Inspvax, true>(const Inspvax& msg) {
  using APType = typename AdaptorTraits<Inspvax, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_ins->mdcEvent.Allocate();
  AdaptorTraits<Inspvax, true>::convert(msg, data.get());
  impl_->pub_ins->mdcEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<Imu, true>(const Imu& msg) {
  using APType = typename AdaptorTraits<Imu, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_imu->mdcEvent.Allocate();
  AdaptorTraits<Imu, true>::convert(msg, data.get());
  impl_->pub_imu->mdcEvent.Send(std::move(data));
}

template <>
inline void Publisher::publish<Control, true>(const Control& msg) {
  using APType = typename AdaptorTraits<Control, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  APTypePtr data = impl_->pub_ctl->CaicControlEvent.Allocate();
  AdaptorTraits<Control, true>::convert(msg, data.get());
  impl_->pub_ctl->CaicControlEvent.Send(std::move(data));
  LOG_INFO("-- [CTL] publish msg --");
}

template <>
inline void Publisher::publish<::caic_localization::LocalizationEstimation, true>(
    const ::caic_localization::LocalizationEstimation& msg) {
  using APType = typename AdaptorTraits<::caic_localization::LocalizationEstimation, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string loc_topic = "/loc_pub";
  APTypePtr data = impl_->pub_str_map[loc_topic]->stringEvent.Allocate();
  AdaptorTraits<::caic_localization::LocalizationEstimation, true>::convert(msg, data.get());
  impl_->pub_str_map[loc_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [LOC] publish msg --");
}

template <>
inline void Publisher::publish<::caic_perception::PerceptionTrafficLights, true>(
    const ::caic_perception::PerceptionTrafficLights& msg) {
  using APType = typename AdaptorTraits<::caic_perception::PerceptionTrafficLights, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string tld_topic = "/tld_pub";
  APTypePtr data = impl_->pub_str_map[tld_topic]->stringEvent.Allocate();
  AdaptorTraits<::caic_perception::PerceptionTrafficLights, true>::convert(msg, data.get());
  impl_->pub_str_map[tld_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [TLD] publish msg --");
}

template <>
inline void Publisher::publish<::caic_perception::PerceptionObjects, true>(
    const ::caic_perception::PerceptionObjects& msg) {
  using APType = typename AdaptorTraits<::caic_perception::PerceptionObjects, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string perc_topic = "/perc_pub";
  std::string rear_vis_topic = "/rear_vis_pub";
  std::string fus_topic = "/fus_pub";
  if (impl_->pub_str_map.find(perc_topic) != impl_->pub_str_map.end()) {
    APTypePtr data = impl_->pub_str_map[perc_topic]->stringEvent.Allocate();
    AdaptorTraits<::caic_perception::PerceptionObjects, true>::convert(msg, data.get());
    impl_->pub_str_map[perc_topic]->stringEvent.Send(std::move(data));
    LOG_INFO("-- [PERC] publish msg --");
  } else if (impl_->pub_str_map.find(rear_vis_topic) != impl_->pub_str_map.end()) {
    APTypePtr data = impl_->pub_str_map[rear_vis_topic]->stringEvent.Allocate();
    AdaptorTraits<::caic_perception::PerceptionObjects, true>::convert(msg, data.get());
    impl_->pub_str_map[rear_vis_topic]->stringEvent.Send(std::move(data));
    LOG_INFO("-- [PERC_REAR_VIS] publish msg --");
  } else if (impl_->pub_str_map.find(fus_topic) != impl_->pub_str_map.end()) {
    APTypePtr data = impl_->pub_str_map[fus_topic]->stringEvent.Allocate();
    AdaptorTraits<::caic_perception::PerceptionObjects, true>::convert(msg, data.get());
    impl_->pub_str_map[fus_topic]->stringEvent.Send(std::move(data));
    LOG_INFO("-- [FUS] publish msg --");
  }
}

template <>
inline void Publisher::publish<::caic_object_fusion::FusionObjects, true>(
    const ::caic_object_fusion::FusionObjects& msg) {
  using APType = typename AdaptorTraits<::caic_object_fusion::FusionObjects, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string fus_topic = "/fus_pub";
  if (impl_->pub_str_map.find(fus_topic) != impl_->pub_str_map.end()) {
    APTypePtr data = impl_->pub_str_map[fus_topic]->stringEvent.Allocate();
    AdaptorTraits<::caic_object_fusion::FusionObjects, true>::convert(msg, data.get());
    impl_->pub_str_map[fus_topic]->stringEvent.Send(std::move(data));
    LOG_INFO("-- [FUS] publish msg --");
  }
}

template <>
inline void Publisher::publish<::stoic::msgs::perception::ObjsPostType, true>(
    const ::stoic::msgs::perception::ObjsPostType& msg) {
  using APType = typename AdaptorTraits<::stoic::msgs::perception::ObjsPostType, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string vis_topic = "/vis_pub";
  APTypePtr data = impl_->pub_str_map[vis_topic]->stringEvent.Allocate();
  AdaptorTraits<::stoic::msgs::perception::ObjsPostType, true>::convert(msg, data.get());
  impl_->pub_str_map[vis_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [PERC_VIS] publish msg --");
}

template <>
inline void Publisher::publish<::stoic::msgs::perception::LaneLinePostType, true>(
    const ::stoic::msgs::perception::LaneLinePostType& msg) {
  using APType = typename AdaptorTraits<::stoic::msgs::perception::LaneLinePostType, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string lane_line_topic = "/lane_line_pub";
  APTypePtr data = impl_->pub_str_map[lane_line_topic]->stringEvent.Allocate();
  AdaptorTraits<::stoic::msgs::perception::LaneLinePostType, true>::convert(msg, data.get());
  impl_->pub_str_map[lane_line_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [PERC_LANE_LINE] publish msg --");
}

template <>
inline void Publisher::publish<::caic_prediction::PredictionObjects, true>(
    const ::caic_prediction::PredictionObjects& msg) {
  using APType = typename AdaptorTraits<::caic_prediction::PredictionObjects, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string pred_topic = "/pred_pub";
  APTypePtr data = impl_->pub_str_map[pred_topic]->stringEvent.Allocate();
  AdaptorTraits<::caic_prediction::PredictionObjects, true>::convert(msg, data.get());
  impl_->pub_str_map[pred_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [PRED] publish msg --");
}

template <>
inline void Publisher::publish<::caic_planning::PlanningResult, true>(
    const ::caic_planning::PlanningResult& msg) {
  using APType = typename AdaptorTraits<::caic_planning::PlanningResult, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string plan_topic = "/plan_pub";
  APTypePtr data = impl_->pub_str_map[plan_topic]->stringEvent.Allocate();
  AdaptorTraits<::caic_planning::PlanningResult, true>::convert(msg, data.get());
  impl_->pub_str_map[plan_topic]->stringEvent.Send(std::move(data));
  LOG_INFO("-- [PLAN] publish msg --");
}

template <>
inline void Publisher::publish<Str, true>(const Str& msg) {
  using APType = typename AdaptorTraits<Str, true>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string sv_topic;
  std::string switch_mode_topic = "/switch_mode_pub";
  std::string switch_mode_repub_topic = "/switch_mode_repub";
  std::string plan_node_input_status_topic = "/planning_node_input_status_pub";
  std::string plan_node_input_status_repub_topic = "/planning_node_input_status_repub";
  std::string loc_switch_mode_topic = "/loc_switch_mode_pub";
  if (impl_->pub_str_map.find(switch_mode_topic) != impl_->pub_str_map.end()) {
    sv_topic = switch_mode_topic;
  } else if (impl_->pub_str_map.find(switch_mode_repub_topic) != impl_->pub_str_map.end()) {
    sv_topic = switch_mode_repub_topic;
  } else if (impl_->pub_str_map.find(plan_node_input_status_topic) != impl_->pub_str_map.end()) {
    sv_topic = plan_node_input_status_topic;
  } else if (impl_->pub_str_map.find(plan_node_input_status_repub_topic) !=
             impl_->pub_str_map.end()) {
    sv_topic = plan_node_input_status_repub_topic;
  } else if (impl_->pub_str_map.find(loc_switch_mode_topic) != impl_->pub_str_map.end()) {
    sv_topic = loc_switch_mode_topic;
  }
  APTypePtr data = impl_->pub_str_map[sv_topic]->stringEvent.Allocate();
  AdaptorTraits<Str, true>::convert(msg, data.get());
  impl_->pub_str_map[sv_topic]->stringEvent.Send(std::move(data));
}

template <typename _Msg, bool _Adaptive = false>
inline void Publisher::publish(const _Msg& msg) {
  using APType = typename AdaptorTraits<_Msg, false>::APType;
  using APTypePtr = std::unique_ptr<APType>;
  std::string sv_topic;
  if constexpr (std::is_same_v<_Msg, DetSV>) {
    sv_topic = "/det_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, TldSV>) {
    sv_topic = "/tld_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, CtlSV>) {
    sv_topic = "/ctl_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, LocSV>) {
    std::string loc_sv_topic = "/loc_sv_pub";
    std::string vio_loc_topic = "/vio_loc_pub";
    if (impl_->pub_str_map.find(loc_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = loc_sv_topic;
    } else if (impl_->pub_str_map.find(vio_loc_topic) != impl_->pub_str_map.end()) {
      sv_topic = vio_loc_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, PredSV>) {
    sv_topic = "/pred_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, PercSV>) {
    std::string perc_sv_topic = "/perc_sv_pub";
    std::string lane_line_sv_topic = "/lane_line_sv_pub";
    std::string front_radar_sv_topic = "/front_radar_sv_pub";
    std::string fus_sv_topic = "/fus_sv_pub";
    std::string fus1_sv_topic = "/fus1_sv_pub";
    std::string fus2_sv_topic = "/fus2_sv_pub";
    std::string fus3_sv_topic = "/fus3_sv_pub";
    std::string fus4_sv_topic = "/fus4_sv_pub";
    std::string fus5_sv_topic = "/fus5_sv_pub";
    if (impl_->pub_str_map.find(perc_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = perc_sv_topic;
    } else if (impl_->pub_str_map.find(lane_line_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = lane_line_sv_topic;
    } else if (impl_->pub_str_map.find(fus_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus_sv_topic;
    } else if (impl_->pub_str_map.find(fus1_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus1_sv_topic;
    } else if (impl_->pub_str_map.find(fus2_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus2_sv_topic;
    } else if (impl_->pub_str_map.find(fus3_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus3_sv_topic;
    } else if (impl_->pub_str_map.find(fus4_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus4_sv_topic;
    } else if (impl_->pub_str_map.find(fus5_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus5_sv_topic;
    } else if (impl_->pub_str_map.find(front_radar_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = front_radar_sv_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, VisSV>) {
    std::string vis_sv_topic = "/vis_sv_pub";
    std::string rear_vis_sv_topic = "/rear_vis_sv_pub";
    if (impl_->pub_str_map.find(vis_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = vis_sv_topic;
    } else if (impl_->pub_str_map.find(rear_vis_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = rear_vis_sv_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, SurrSV>) {
    std::string surr_topic = "/surround_pub";
    std::string surr_sv_topic = "/surround_sv_pub";
    if (impl_->pub_str_map.find(surr_topic) != impl_->pub_str_map.end()) {
      sv_topic = surr_topic;
    } else if (impl_->pub_str_map.find(surr_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = surr_sv_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, PlanSV>) {
    sv_topic = "/plan_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, RoutSV>) {
    sv_topic = "/rout_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, PcdSV>) {
    sv_topic = "/pcd_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, MarkSV>) {
    std::string fus1_match_sv_topic = "/fus1_match_sv_pub";
    std::string fus2_match_sv_topic = "/fus2_match_sv_pub";
    std::string fus3_match_sv_topic = "/fus3_match_sv_pub";
    std::string fus4_match_sv_topic = "/fus4_match_sv_pub";
    std::string fus5_match_sv_topic = "/fus5_match_sv_pub";
    std::string fus6_match_sv_topic = "/fus6_match_sv_pub";
    std::string fus7_match_sv_topic = "/fus7_match_sv_pub";
    std::string fus8_match_sv_topic = "/fus8_match_sv_pub";
    std::string fus9_match_sv_topic = "/fus9_match_sv_pub";
    std::string fus10_match_sv_topic = "/fus10_match_sv_pub";
    std::string plan_node_raw_park_traj_vis_topic = "/planning_node_rawparktraj_vis_pub";
    std::string plan_node_park_boundary_topic = "/planning_node_parkboundary_pub";
    std::string plan_node_vision_parking_spaces_topic = "/planning_node_vision_parking_spaces_pub";
    std::string plan_node_freespace_topic = "/planning_node_freespace_pub";
    std::string plan_node_ego_center_topic = "/planning_node_ego_center_pub";
    std::string plan_node_parking_space_id_topic = "/planning_node_parking_space_id_pub";
    std::string loc_ft_ipm_map_topic = "/loc_ft_ipm_map_pub";
    std::string loc_path_topic = "/loc_path_pub";
    std::string uss_obstacle_topic = "/uss_obstacle_sv_pub";
    if (impl_->pub_str_map.find(fus1_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus1_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus2_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus2_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus3_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus3_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus4_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus4_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus5_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus5_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus6_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus6_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus7_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus7_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus8_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus8_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus9_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus9_match_sv_topic;
    } else if (impl_->pub_str_map.find(fus10_match_sv_topic) != impl_->pub_str_map.end()) {
      sv_topic = fus10_match_sv_topic;
    } else if (impl_->pub_str_map.find(plan_node_raw_park_traj_vis_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = plan_node_raw_park_traj_vis_topic;
    } else if (impl_->pub_str_map.find(plan_node_park_boundary_topic) != impl_->pub_str_map.end()) {
      sv_topic = plan_node_park_boundary_topic;
    } else if (impl_->pub_str_map.find(plan_node_vision_parking_spaces_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = plan_node_vision_parking_spaces_topic;
    } else if (impl_->pub_str_map.find(plan_node_freespace_topic) != impl_->pub_str_map.end()) {
      sv_topic = plan_node_freespace_topic;
    } else if (impl_->pub_str_map.find(plan_node_ego_center_topic) != impl_->pub_str_map.end()) {
      sv_topic = plan_node_ego_center_topic;
    } else if (impl_->pub_str_map.find(plan_node_parking_space_id_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = plan_node_parking_space_id_topic;
    } else if (impl_->pub_str_map.find(loc_ft_ipm_map_topic) != impl_->pub_str_map.end()) {
      sv_topic = loc_ft_ipm_map_topic;
    } else if (impl_->pub_str_map.find(loc_path_topic) != impl_->pub_str_map.end()) {
      sv_topic = loc_path_topic;
    } else if (impl_->pub_str_map.find(uss_obstacle_topic) != impl_->pub_str_map.end()) {
      sv_topic = uss_obstacle_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, ImgMarkSV>) {
    std::string perc_surround_space_result_topic = "/perception_surround_space_result_pub";
    std::string perc_surround_ipm_mask_topic = "/perception_surround_ipm_mask_pub";
    std::string perc_surround_freespace_object_topic = "/perception_surround_freespace_object_pub";
    std::string perc_surround_sur_mul_det_result_topic =
        "/perception_surround_sur_mul_det_result_pub";
    std::string perc_surround_parking_space_frame_topic =
        "/perception_surround_parking_space_frame_pub";
    if (impl_->pub_str_map.find(perc_surround_space_result_topic) != impl_->pub_str_map.end()) {
      sv_topic = perc_surround_space_result_topic;
    } else if (impl_->pub_str_map.find(perc_surround_ipm_mask_topic) != impl_->pub_str_map.end()) {
      sv_topic = perc_surround_ipm_mask_topic;
    } else if (impl_->pub_str_map.find(perc_surround_freespace_object_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = perc_surround_freespace_object_topic;
    } else if (impl_->pub_str_map.find(perc_surround_sur_mul_det_result_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = perc_surround_sur_mul_det_result_topic;
    } else if (impl_->pub_str_map.find(perc_surround_parking_space_frame_topic) !=
               impl_->pub_str_map.end()) {
      sv_topic = perc_surround_parking_space_frame_topic;
    }
  } else if constexpr (std::is_same_v<_Msg, PsSV>) {
    sv_topic = "/clicked_point_sv_pub";
  } else if constexpr (std::is_same_v<_Msg, ImgMarkSV>) {
    std::string loc_ft_img_topic = "/loc_ft_img_pub";
    std::string loc_ipm_img_topic = "/loc_ipm_img_pub";
    if (impl_->pub_str_map.find(loc_ft_img_topic) != impl_->pub_str_map.end()) {
      sv_topic = loc_ft_img_topic;
    } else if (impl_->pub_str_map.find(loc_ipm_img_topic) != impl_->pub_str_map.end()) {
      sv_topic = loc_ipm_img_topic;
    }
  }
  APTypePtr data = impl_->pub_str_map[sv_topic]->stringEvent.Allocate();
  AdaptorTraits<_Msg, false>::convert(msg, data.get());
  impl_->pub_str_map[sv_topic]->stringEvent.Send(std::move(data));
}

}  // namespace cm
