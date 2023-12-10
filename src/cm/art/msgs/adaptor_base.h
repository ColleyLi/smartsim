#pragma once

#include <cmath>
#include "caic_interface.h"
#include "sim_interface.h"
#include "cm/alg/pool/object_pool.hpp"
#include "proto/common/meta.pb.h"
#include "proto/common/string.pb.h"
#include "proto/fusion/caic_world_module.pb.h"
#include "proto/perception/perception_lane.pb.h"
#include "proto/perception/perception_bevlane.pb.h"
#include "proto/perception/perception_surroundvision.pb.h"
#include "proto/prediction/prediction.pb.h"
#include "proto/sensor/canbus.pb.h"
#include "proto/sensor/corrimu.pb.h"
#include "proto/sensor/image.pb.h"
#include "proto/sensor/imu.pb.h"
#include "proto/sensor/inspvax.pb.h"
#include "proto/sensor/odometry.pb.h"
#include "proto/sensor/bestgnsspos.pb.h"
#include "proto/sensor/point_cloud32.pb.h"
#include "proto/control/control_cmd.pb.h"
#include "proto/drivers/chassis.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/interaction/interaction.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/smartsil/sim_control.pb.h"
#include "proto/smartsil/sim_chassis.pb.h"
#include "proto/smartsil/sim_localization.pb.h"
#include "proto/smartsil/gps.pb.h"
#include "proto/smartsil/imu.pb.h"
#include "proto/smartsil/sim_perception_obstacles.pb.h"
#include "proto/smartsil/sim_traffic_light.pb.h"

namespace cm {
static constexpr double S_2_US = 1000000.0;

static constexpr size_t M_POOL_SIZE = 1'000'000;
static constexpr size_t M_POINT_SIZE = 1'000'000;

}  // namespace cm

namespace cm {

const int im_2m_w = 1920;
const int im_2m_h = 1080;
const int im_8m_w = 3840;
const int im_8m_h = 2160;
const int im_rgb_c = 3;

using CaicOdometry = ::caic_sensor::caic_ins::Odometry;

using RTPString = stoic::cm::proto::common::PString;
using RTPStringPtr = std::shared_ptr<RTPString>;

using RTImage = stoic::cm::proto::sensor::Image;
using RTImagePtr = std::shared_ptr<stoic::cm::proto::sensor::Image>;

using RTPointCloud = stoic::cm::proto::sensor::PointCloudTypeArray;
using RTPointCloudPtr = std::shared_ptr<stoic::cm::proto::sensor::PointCloudTypeArray>;

template <typename _Msg, bool _Adaptive = false>
struct AdaptorTraits {
  using RTType = _Msg;
  using RTPtrType = std::shared_ptr<_Msg>;
  static void convert(const _Msg& rt_msg, _Msg& msg) { msg = rt_msg; }
  static void convert(const RTPtrType& msg, _Msg* rt_msg) {
    memcpy(rt_msg, msg.get(), sizeof(_Msg));
  }
};

}  // namespace cm
