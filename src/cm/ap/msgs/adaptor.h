#pragma once

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ascend_hal.h"
#include "caic_interface.h"
#include "cm/alg/alg.h"
#include "cm/ap/generated/ara/camera/cameradecodedmbufserviceinterface_proxy.h"
#include "cm/ap/generated/ara/camera/cameradecodedmbufserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/camera/impl_type_cameradecodedmbufstruct.h"
#include "cm/ap/generated/ara/gnss/gnssinfoserviceinterface_proxy.h"
#include "cm/ap/generated/ara/gnss/gnssinfoserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/gnss/impl_type_gnssinfo.h"
#include "cm/ap/generated/ara/imu/impl_type_imuinfo.h"
#include "cm/ap/generated/ara/imu/imuinfoserviceinterface_proxy.h"
#include "cm/ap/generated/ara/imu/imuinfoserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/ins/impl_type_insinfo.h"
#include "cm/ap/generated/ara/ins/insinfoserviceinterface_proxy.h"
#include "cm/ap/generated/ara/ins/insinfoserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/lidar/impl_type_lidarpointcloud.h"
#include "cm/ap/generated/ara/lidar/lidarserviceinterface_proxy.h"
#include "cm/ap/generated/ara/lidar/lidarserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/rawdata/impl_type_rawdatastruct.h"
#include "cm/ap/generated/ara/rawdata/rawdataserviceinterface_proxy.h"
#include "cm/ap/generated/ara/rawdata/rawdataserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/rtrack/impl_type_radartrackarray.h"
#include "cm/ap/generated/ara/rtrack/radartrackserviceinterface_proxy.h"
#include "cm/ap/generated/ara/rtrack/radartrackserviceinterface_skeleton.h"
#include "cm/ap/generated/ara/string/impl_type_stringstruct.h"
#include "cm/ap/generated/ara/string/stringserviceinterface_proxy.h"
#include "cm/ap/generated/ara/string/stringserviceinterface_skeleton.h"
#include "cm/ap/generated/caic_control_mdc/caiccontrolserviceinterface_proxy.h"
#include "cm/ap/generated/caic_control_mdc/caiccontrolserviceinterface_skeleton.h"
#include "cm/ap/generated/caic_control_mdc/impl_type_controlcommand.h"
#include "cm/ap/generated/caic_sensor_mdc/caicsensorcanbusserviceinterface_proxy.h"
#include "cm/ap/generated/caic_sensor_mdc/caicsensorcanbusserviceinterface_skeleton.h"
#include "cm/ap/generated/caic_sensor_mdc/impl_type_canbus.h"
#include "cm/ap/generated/impl_type_string.h"
#include "common/caic_assert.h"
#include "common/magic.h"
#include "google/protobuf/message.h"
#include "msgs/msgs.h"
#include "proto/geometry/vector3.pb.h"
#include "proto/sensor/image.pb.h"
#include "proto/sensor/point_cloud32.pb.h"
#include "proto/smartview/marker.pb.h"
#include "submodule/caic_interface/include/caic_interface/caic_object_fusion.h"
#include "third_party/apollo/proto/control/control_cmd.pb.h"
#include "third_party/apollo/proto/drivers/chassis_detail.pb.h"
#include "third_party/apollo/proto/drivers/conti_radar.pb.h"
#include "third_party/apollo/proto/drivers/corner_radar.pb.h"
#include "third_party/apollo/proto/localization/localization.pb.h"
#include "third_party/apollo/proto/perception/perception_obstacle.pb.h"
#include "third_party/apollo/proto/perception/perception_surroundvision.pb.h"
#include "third_party/apollo/proto/perception/perception_vision_obstacle.pb.h"
#include "third_party/apollo/proto/perception/traffic_light_detection.pb.h"
#include "third_party/apollo/proto/planning/planning.pb.h"
#include "third_party/apollo/proto/prediction/prediction_obstacle.pb.h"
#include "third_party/apollo/proto/routing/routing.pb.h"

namespace stoic::cm {

static constexpr double map_offset_x = 671147.79;
static constexpr double map_offset_y = 3536226.82;
static constexpr double rad_to_deg = 180.0 / M_PI;
static constexpr double deg_to_rad = M_PI / 180.0;
static constexpr double g = 9.80665;
static constexpr double heading_refined = 0.95 * deg_to_rad;
static Eigen::Matrix3d R_ENU_NED = [] {
  Eigen::Matrix3d m;
  m << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  return m;
}();
static Eigen::Matrix3d R_FRD_RFU = [] {
  Eigen::Matrix3d m;
  m << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  return m;
}();

using RawDataSkeleton = ::ara::rawdata::skeleton::RawDataServiceInterfaceSkeleton;
using RawDataSkeletonPtr = std::shared_ptr<RawDataSkeleton>;
using RawDataProxy = ::ara::rawdata::proxy::RawDataServiceInterfaceProxy;
using RawDataProxyPtr = std::shared_ptr<RawDataProxy>;

using StrSkeleton = ::ara::string::skeleton::StringServiceInterfaceSkeleton;
using StrSkeletonPtr = std::shared_ptr<StrSkeleton>;
using StrProxy = ::ara::string::proxy::StringServiceInterfaceProxy;
using StrProxyPtr = std::shared_ptr<StrProxy>;

using PcdSkeleton = ::ara::lidar::skeleton::LidarServiceInterfaceSkeleton;
using PcdSkeletonPtr = std::shared_ptr<PcdSkeleton>;
using PcdProxy = ::ara::lidar::proxy::LidarServiceInterfaceProxy;
using PcdProxyPtr = std::shared_ptr<PcdProxy>;

using ImageSkeleton = ::mdc::cam::camera::skeleton::CameraDecodedMbufServiceInterfaceSkeleton;
using ImageSkeletonPtr = std::shared_ptr<ImageSkeleton>;
using ImageProxy = ::mdc::cam::camera::proxy::CameraDecodedMbufServiceInterfaceProxy;
using ImageProxyPtr = std::shared_ptr<ImageProxy>;

using RadarSkeleton = ::ara::rtrack::skeleton::RadarTrackServiceInterfaceSkeleton;
using RadarSkeletonPtr = std::shared_ptr<RadarSkeleton>;
using RadarProxy = ::ara::rtrack::proxy::RadarTrackServiceInterfaceProxy;
using RadarProxyPtr = std::shared_ptr<RadarProxy>;

using InsSkeleton = ::ara::ins::skeleton::InsInfoServiceInterfaceSkeleton;
using InsSkeletonPtr = std::shared_ptr<InsSkeleton>;
using InsProxy = ::ara::ins::proxy::InsInfoServiceInterfaceProxy;
using InsProxyPtr = std::shared_ptr<InsProxy>;

using ChassisSkeleton = caic_sensor_mdc::skeleton::CaicSensorCanbusServiceInterfaceSkeleton;
using ChassisSkeletonPtr = std::shared_ptr<ChassisSkeleton>;
using ChassisProxy = ::caic_sensor_mdc::proxy::CaicSensorCanbusServiceInterfaceProxy;
using ChassisProxyPtr = std::shared_ptr<ChassisProxy>;

using ControlSkeleton = caic_control_mdc::skeleton::CaicControlServiceInterfaceSkeleton;
using ControlSkeletonPtr = std::shared_ptr<ControlSkeleton>;

using ImuSkeleton = ::ara::imu::skeleton::ImuInfoServiceInterfaceSkeleton;
using ImuSkeletonPtr = std::shared_ptr<ImuSkeleton>;
using ImuProxy = ::ara::imu::proxy::ImuInfoServiceInterfaceProxy;
using ImuProxyPtr = std::shared_ptr<ImuProxy>;

// using GnssSkeleton = ::ara::gnss::skeleton::GnssInfoServiceInterfaceSkeleton;
// using GnssSkeletonPtr = std::shared_ptr<GnssSkeleton>;
// using GnssProxy = ::ara::gnss::proxy::GnssInfoServiceInterfaceProxy;
// using GnssProxyPtr = std::shared_ptr<GnssProxy>;

template <typename _Msg, bool _Adaptive = false>
struct AdaptorTraits;

// template <>
// struct AdaptorTraits<msgs::sensor::GNSS, true> {
//   using Type = msgs::sensor::GNSS;
//   using APType = ::ara::gnss::GnssInfo;

//   static void convert(const APType& ap_msg, Type* msg) {
//     msg->header.seq = ap_msg.header.seq;
//     msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
//     msg->header.frame_id = ap_msg.header.frame_id;
//     msg->pose.position.x = ap_msg.pose.pose.position.x;
//     msg->pose.position.y = ap_msg.pose.pose.position.y;
//     msg->pose.position.z = ap_msg.pose.pose.position.z;
//     msg->pose.orientation.x = ap_msg.pose.pose.orientation.x;
//     msg->pose.orientation.y = ap_msg.pose.pose.orientation.y;
//     msg->pose.orientation.z = ap_msg.pose.pose.orientation.z;
//     msg->pose.orientation.w = ap_msg.pose.pose.orientation.w;
//     for (size_t i = 0; i < 36; ++i) {
//       msg->pose_cov(i / 6, i % 6) = ap_msg.pose.covariance[i];
//     }
//     msg->twist.linear.x = ap_msg.twist.twist.linear.x;
//     msg->twist.linear.y = ap_msg.twist.twist.linear.y;
//     msg->twist.linear.z = ap_msg.twist.twist.linear.z;
//     msg->twist.angular.x = ap_msg.twist.twist.angular.x;
//     msg->twist.angular.y = ap_msg.twist.twist.angular.y;
//     msg->twist.angular.z = ap_msg.twist.twist.angular.z;
//     for (size_t i = 0; i < 36; ++i) {
//       msg->twist_cov(i / 6, i % 6) = ap_msg.twist.covariance[i];
//     }
//   }

//   static void convert(const Type&, ROSType*) {
//     ASSERT(false && "GNSS deserialize not implemented.");
//   }
// };

// template <std::size_t N>
// struct AdaptorTraits<msgs::sensor::PointCloudBase<PointXYZIRTLf, N>, true> {
//   using Type = msgs::sensor::PointCloudBase<PointXYZIRTLf, N>;
//   using APType = ::ara::lidar::LidarPointCloud;

//   static void convert(const APType& ap_msg, Type* msg) {
//     msg->header.seq = ap_msg.header.seq;
//     msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
//     msg->header.frame_id = ap_msg.header.frameId;
//     msg->size = ap_msg.width * ap_msg.height;
//     for (size_t i = 0; i < msg->size; i++) {
//       uint8_t intensity = 0;
//       double timestamp;
//       Eigen::Map<Eigen::VectorXf> xyz_in(
//           reinterpret_cast<float*>(const_cast<unsigned char*>(&ap_msg.data[ap_msg.pointStep *
//           i])), 16);
//       Eigen::Map<Eigen::VectorXf> xyz_out(reinterpret_cast<float*>(&msg->points[i]), 16);
//       xyz_out = xyz_in;
//       msg->points[i].intensity = (uint16_t)intensity;
//       std::memcpy(&msg->points[i].ring, &ap_msg.data[ap_msg.pointStep * i + 18], 2);
//       std::memcpy(&timestamp, &ap_msg.data[ap_msg.pointStep * i + 24], 8);
//       msg->points[i].timestamp_2us = (uint16_t)timestamp;
//     }
//   }

//   static void convert(const Type&, APType*) {
//     ASSERT(false && "PointCloud deserialize not implemented.");
//   }
// };

template <>
struct AdaptorTraits<::caic_sensor::PointCloudTypeArray, true> {
  using Type = ::caic_sensor::PointCloudTypeArray;
  using APType = ::ara::lidar::LidarPointCloud;

  static void convert(const APType& ap_msg, Type* msg) {
    int32_t timestamp_frame_tail;
    std::memcpy(&timestamp_frame_tail,
                &ap_msg.data[ap_msg.pointStep * msg->width * msg->height + 12], 4);
    // msg->header.seq = ap_msg.header.seq;
    // msg->header.frame_id = ap_msg.header.frameId;
    msg->timestamp =
        ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS + timestamp_frame_tail;
    msg->width = ap_msg.width;
    msg->height = ap_msg.height;
    uint16_t intensity;
    int32_t timestamp;
    for (uint32_t i = 0; i < msg->width * msg->height; i++) {
      Eigen::Map<Eigen::VectorXf> xyz_in(
          reinterpret_cast<float*>(const_cast<unsigned char*>(&ap_msg.data[ap_msg.pointStep * i])),
          12);
      Eigen::Map<Eigen::VectorXf> xyz_out(reinterpret_cast<float*>(&msg->points[i]), 12);
      xyz_out = xyz_in;
      std::memcpy(&timestamp, &ap_msg.data[ap_msg.pointStep * i + 12], 4);
      msg->points[i].timestamp_2us = (uint16_t)((timestamp_frame_tail - timestamp) / 2000);
      std::memcpy(&intensity, &ap_msg.data[ap_msg.pointStep * i + 28], 2);
      msg->points[i].intensity = (uint8_t)intensity;
      std::memcpy(&msg->points[i].ring, &ap_msg.data[ap_msg.pointStep * i + 30], 2);
    }
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "PointCloud deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::Image, true> {
  using Type = ::caic_sensor::Image;
  using APType = ::ara::camera::CameraDecodedMbufStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.CameraHeader.Seq;
    msg->header.stamp =
        ap_msg.CameraHeader.Stamp.Sec * S_2_US + ap_msg.CameraHeader.Stamp.Nsec / S_2_MS;
    msg->width = ap_msg.Width;
    msg->height = ap_msg.Height;

    Mbuf* pMbuf = reinterpret_cast<Mbuf*>(ap_msg.RawData);
    uint64_t len;
    if (halMbufGetDataPtr(pMbuf, reinterpret_cast<void**>(&msg->p_data), &len) != 0) {
      std::cout << "Image halMbufGetDataPtr failed!" << std::endl;
      return;
    }
    halMbufGetDataLen(pMbuf, &len);
    msg->size = len;
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "Image deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::RadarObjectArray, true> {
  using Type = ::caic_sensor::RadarObjectArray;
  using APType = ::ara::rtrack::RadarTrackArray;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp.sec;
    msg->header.frame_id = ap_msg.header.frame_id;
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "radar deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::Imu, true> {
  using Type = ::caic_sensor::Imu;
  using APType = ::ara::imu::ImuInfo;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
    msg->header.frame_id = "FRD";
    msg->linear_acceleration_info.linear_acceleration.x = ap_msg.acceleration.x;
    msg->linear_acceleration_info.linear_acceleration.y = -ap_msg.acceleration.y;
    msg->linear_acceleration_info.linear_acceleration.z = -ap_msg.acceleration.z;
    msg->linear_acceleration_info.available = caic_std::LinearAccelerationWithCovariance::VALUE;

    msg->angular_velocity_info.angular_velocity.x = ap_msg.angularVelocity.x;
    msg->angular_velocity_info.angular_velocity.y = -ap_msg.angularVelocity.y;
    msg->angular_velocity_info.angular_velocity.z = -ap_msg.angularVelocity.z;
    msg->angular_velocity_info.available = caic_std::AngularVelocityWithCovariance::VALUE;

    msg->available = caic_sensor::Imu::IMU_LINEAR_ACCELERATION_INFO |
                     caic_sensor::Imu::IMU_ANGULAR_VELOCITY_INFO;
  }

  static void convert(const Type&, APType*) { ASSERT(false && "Imu deserialize not implemented."); }
};

template <>
struct AdaptorTraits<::caic_sensor::caic_ins::Odometry, true> {
  using Type = ::caic_sensor::caic_ins::Odometry;
  using APType = ::ara::ins::InsInfo;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
    msg->header.frame_id = "ENU";
    msg->child_frame_id = "RFU";

    // pose_with_covariance
    msg->pose_with_covariance.position.x = ap_msg.utmPosition.x - map_offset_x;
    msg->pose_with_covariance.position.y = ap_msg.utmPosition.y - map_offset_y;
    msg->pose_with_covariance.position.z = ap_msg.utmPosition.z;

    double roll = ap_msg.attitude.x;
    double pitch = ap_msg.attitude.y;
    double heading = ap_msg.attitude.z - heading_refined;
    if (heading < 0) {
      heading += 2 * M_PI;
    }
    //把[0, 2pi]转为[-pi, pi]
    double yaw = (heading > M_PI ? heading - 2 * M_PI : heading);

    // zyx,外旋
    Eigen::Matrix3d R_NED_FRD;
    R_NED_FRD = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R_ENU_RFU = R_ENU_NED * R_NED_FRD * R_FRD_RFU;
    Eigen::Quaterniond quat(R_ENU_RFU);
    quat.normalize();
    msg->pose_with_covariance.quaternion.w = quat.w();
    msg->pose_with_covariance.quaternion.x = quat.x();
    msg->pose_with_covariance.quaternion.y = quat.y();
    msg->pose_with_covariance.quaternion.z = quat.z();
    msg->pose_with_covariance.covariance.data[0] = std::pow(ap_msg.sdPosition.x, 2);
    msg->pose_with_covariance.covariance.data[7] = std::pow(ap_msg.sdPosition.y, 2);
    msg->pose_with_covariance.covariance.data[14] = std::pow(ap_msg.sdPosition.z, 2);
    msg->pose_with_covariance.covariance.data[21] = std::pow(ap_msg.sdAttitude.x, 2);
    msg->pose_with_covariance.covariance.data[28] = std::pow(ap_msg.sdAttitude.y, 2);
    msg->pose_with_covariance.covariance.data[35] = std::pow(ap_msg.sdAttitude.z, 2);
    msg->pose_with_covariance.available = caic_std::PoseWithCovariance::POSITION |
                                          caic_std::PoseWithCovariance::ORIENTATION |
                                          caic_std::PoseWithCovariance::COVARINACE;

    // twist_with_covariance
    msg->twist_with_covariance.linear.x = ap_msg.linearVelocity.x;
    msg->twist_with_covariance.linear.y = ap_msg.linearVelocity.y;
    msg->twist_with_covariance.linear.z = ap_msg.linearVelocity.z;
    msg->twist_with_covariance.covariance.data[0] = std::pow(ap_msg.sdVelocity.x, 2);
    msg->twist_with_covariance.covariance.data[7] = std::pow(ap_msg.sdVelocity.y, 2);
    msg->twist_with_covariance.covariance.data[14] = std::pow(ap_msg.sdVelocity.z, 2);
    msg->twist_with_covariance.available =
        caic_std::TwistWithCovariance::LINEAR | caic_std::TwistWithCovariance::COVARINACE;

    msg->available = caic_sensor::caic_ins::Odometry::ODOMETRY_CHILD_FRAME_ID |
                     caic_sensor::caic_ins::Odometry::ODOMETRY_POSE_WITH_COVARIANCE |
                     caic_sensor::caic_ins::Odometry::ODOMETRY_TWIST_WITH_COVARIANCE;
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "odom deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::caic_ins::CorrImu, true> {
  using Type = ::caic_sensor::caic_ins::CorrImu;
  using APType = ::ara::ins::InsInfo;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
    msg->header.frame_id = "ENU";

    double roll = ap_msg.attitude.x;
    double pitch = ap_msg.attitude.y;
    double heading = ap_msg.attitude.z - heading_refined;
    if (heading < 0) {
      heading += 2 * M_PI;
    }
    //把[0, 2pi]转为[-pi, pi]
    double yaw = (heading > M_PI ? heading - 2 * M_PI : heading);

    // zyx,外旋
    Eigen::Matrix3d R_NED_FRD;
    R_NED_FRD = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R_ENU_RFU = R_ENU_NED * R_NED_FRD * R_FRD_RFU;
    Eigen::Quaterniond quat(R_ENU_RFU);
    quat.normalize();
    msg->orientation_info.quaternion.w = quat.w();
    msg->orientation_info.quaternion.x = quat.x();
    msg->orientation_info.quaternion.y = quat.y();
    msg->orientation_info.quaternion.z = quat.z();
    msg->orientation_info.quaternion_covariance.data[0] = std::pow(ap_msg.sdAttitude.x, 2);
    msg->orientation_info.quaternion_covariance.data[4] = std::pow(ap_msg.sdAttitude.y, 2);
    msg->orientation_info.quaternion_covariance.data[8] = std::pow(ap_msg.sdAttitude.z, 2);
    msg->orientation_info.available = caic_std::OrientationWithCovariance::VALUE |
                                      caic_std::OrientationWithCovariance::QUATERNION_COVARIANCE;

    Eigen::Vector3d acc_with_g;

    // RFU,但有重力
    acc_with_g.x() = -ap_msg.acceleration.y;
    acc_with_g.y() = ap_msg.acceleration.x;
    acc_with_g.z() = ap_msg.acceleration.z;

    Eigen::Vector3d acc_without_g;
    acc_without_g = acc_with_g - R_ENU_RFU.inverse() * Eigen::Vector3d(0, 0, g);

    msg->linear_acceleration_info.linear_acceleration.x = acc_without_g.x();
    msg->linear_acceleration_info.linear_acceleration.y = acc_without_g.y();
    msg->linear_acceleration_info.linear_acceleration.z = acc_without_g.z();
    msg->linear_acceleration_info.available = caic_std::LinearAccelerationWithCovariance::VALUE;

    msg->angular_velocity_info.angular_velocity.x = -ap_msg.angularVelocity.y;
    msg->angular_velocity_info.angular_velocity.y = ap_msg.angularVelocity.x;
    msg->angular_velocity_info.angular_velocity.z = ap_msg.angularVelocity.z;
    msg->angular_velocity_info.available = caic_std::AngularVelocityWithCovariance::VALUE;

    msg->available = caic_sensor::caic_ins::CorrImu::CORR_IMU_ORIENTATION_INFO |
                     caic_sensor::caic_ins::CorrImu::CORR_IMU_LINEAR_ACCELERATION_INFO |
                     caic_sensor::caic_ins::CorrImu::CORR_IMU_ANGULAR_VELOCITY_INFO;
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "corrimu deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::caic_ins::Inspavx, true> {
  using Type = ::caic_sensor::caic_ins::Inspavx;
  using APType = ::ara::ins::InsInfo;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp.sec * S_2_US + ap_msg.header.stamp.nsec / S_2_MS;
    msg->header.frame_id = "ENU";

    msg->status = caic_sensor::caic_ins::InsStatus::INS_INACTIVE;
    msg->pos_type = caic_sensor::caic_ins::InsPoseType::NONE;
    msg->ext_sol_status = 0u;

    // 0：None
    // 1: 姿态初始化（航向未初始化）
    // 2: 组合导航
    if (ap_msg.solutionStatus == 2) {
      // 48_L1_INT_L1 固定解
      // 49_WIDE_INT_宽巷固定解
      // 50_NARROW_INT_窄巷固定解
      if (ap_msg.positionType == 48 || ap_msg.positionType == 49 || ap_msg.positionType == 50) {
        msg->status = caic_sensor::caic_ins::InsStatus::INS_SOLUTION_GOOD;
        msg->pos_type = caic_sensor::caic_ins::InsPoseType::INS_RTKFIXED;
        msg->ext_sol_status = 64u;
      }
    }

    msg->position.latitude_stdev = ap_msg.sdPosition.y;
    msg->position.longitude_stdev = ap_msg.sdPosition.x;
    msg->orientation.azimuth_stdev = ap_msg.sdAttitude.z * rad_to_deg;

    msg->available = caic_sensor::caic_ins::Inspavx::INSPV_X_INS_STATUS |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_POS_TYPE |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_EXT_SOL_STATUS |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_POSITION |
                     caic_sensor::caic_ins::Inspavx::INSPV_X_ORIENATION;

    // LOG_INFO << "rec ap msg time:" << msg->header.stamp
    //         << ",inspavx solution status: " << ap_msg.solutionStatus
    //         << ",pos type: " << ap_msg.positionType;
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "inspvax deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_control::ControlCommand, true> {
  using Type = ::caic_control::ControlCommand;
  using APType = ::caic_control_mdc::ControlCommand;

  static void convert(const Type& msg, APType* ap_msg) {
    ap_msg->header.seq = msg.header.seq;
    ap_msg->header.stamp = msg.header.stamp;
    ap_msg->header.frame_id = msg.header.frame_id;
    ap_msg->throttle = msg.throttle;
    ap_msg->brake = msg.brake;
    ap_msg->steering_target = msg.steering_target;
    ap_msg->steering_rate = msg.steering_rate;
    ap_msg->steering_torque = msg.steering_torque;
    ap_msg->speed = msg.speed;
    ap_msg->acceleration = msg.acceleration;
    ap_msg->parking_distance = msg.parking_distance;
    if (msg.engine_on_off == caic_control::EngineReqSt::ENGINE_ON_REQUEST) {
      ap_msg->engine_on_off = true;
    } else {
      ap_msg->engine_on_off = false;
    }
    ap_msg->is_estop = msg.is_estop;
    ap_msg->stand_still = msg.stand_still;
    ap_msg->rearview_mirror = msg.rearview_mirror;
    ap_msg->driving_mode = static_cast<decltype(ap_msg->driving_mode)>(msg.driving_mode);
    ap_msg->gear_state = static_cast<decltype(ap_msg->gear_state)>(msg.gear_state);
    ap_msg->vehicle_signal.turn_signal =
        static_cast<decltype(ap_msg->vehicle_signal.turn_signal)>(msg.vehicle_signal.turn_signal);
  }
  static void convert(const APType&, Type*) {
    ASSERT(false && " control command deserialize not implemented.");
  }
};

template <>
struct AdaptorTraits<::caic_sensor::Canbus, true> {
  using Type = ::caic_sensor::Canbus;
  using APType = ::caic_sensor_mdc::Canbus;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->header.seq = ap_msg.header.seq;
    msg->header.stamp = ap_msg.header.stamp * 1000;
    msg->header.frame_id = ap_msg.header.frame_id;
    msg->chassis_info.esp.vehicle_speed = ap_msg.chassis_info.esp.vehicle_speed;
    msg->chassis_info.esp.wheel_pulse_info.fl = ap_msg.chassis_info.esp.wheel_speed_info.fl;
    msg->chassis_info.esp.wheel_pulse_info.fr = ap_msg.chassis_info.esp.wheel_speed_info.fr;
    msg->chassis_info.esp.wheel_pulse_info.rl = ap_msg.chassis_info.esp.wheel_speed_info.rl;
    msg->chassis_info.esp.wheel_pulse_info.rr = ap_msg.chassis_info.esp.wheel_speed_info.rr;
    msg->chassis_info.yrs.acceleration_x = ap_msg.chassis_info.yrs.acceleration_x;
    msg->driving_mode = static_cast<decltype(msg->driving_mode)>(ap_msg.driving_mode);
    msg->chassis_info.gear_info =
        static_cast<decltype(msg->chassis_info.gear_info)>(ap_msg.chassis_info.gear_info);
    msg->chassis_info.epb.epb_working_state =
        static_cast<decltype(msg->chassis_info.epb.epb_working_state)>(
            ap_msg.chassis_info.epb.EPBWorkingState);
    msg->chassis_info.eps.steering_wheel_info.angle =
        ap_msg.chassis_info.eps.steering_wheel_info.angle;
    msg->chassis_info.eps.steering_wheel_info.angle_sign =
        ap_msg.chassis_info.eps.steering_wheel_info.angle_sign;
    msg->chassis_info.eps.steering_wheel_info.speed =
        ap_msg.chassis_info.eps.steering_wheel_info.speed;
    msg->chassis_info.eps.steering_wheel_info.speed_sign =
        ap_msg.chassis_info.eps.steering_wheel_info.speed_sign;
    msg->chassis_info.esp.wheel_speed_info.rl = ap_msg.chassis_info.esp.wheel_speed_info.rl;
    msg->chassis_info.esp.wheel_speed_info.rr = ap_msg.chassis_info.esp.wheel_speed_info.rr;
    msg->chassis_info.esp.driving_direction =
        static_cast<decltype(msg->chassis_info.esp.driving_direction)>(
            ap_msg.chassis_info.esp.driving_direction);
  }

  static void convert(const Type&, APType*) {
    ASSERT(false && "canbus deserialize not implemented.");
  }
};
template <>
struct AdaptorTraits<::apollo::drivers::ContiRadar, true> {
  using Type = ::apollo::drivers::ContiRadar;
  using APType = ::ara::string::StringStruct;
  static void convert(const APType& ap_msg, Type* msg) { msg->ParseFromString(ap_msg.string); }
  static void convert(const Type& msg, APType* ap_msg) { msg.SerializeToString(&ap_msg->string); }
};
// template <>
// struct AdaptorTraits<::apollo::drivers::ContiRadar, true> {
//   using Type = ::apollo::drivers::ContiRadar;
//   using APType = ::ara::rtrack::RadarTrackArray;
//   using Measurementstatus_00Type = ::apollo::drivers::ContiRadarObs_Measurementstatus_00Type;
//   using Movementstatus_00Type = ::apollo::drivers::ContiRadarObs_Movementstatus_00Type;
//   using Isnew_00Type = ::apollo::drivers::ContiRadarObs_Isnew_00Type;

//   static void convert(const APType& ap_msg, Type* msg) {
//     // TODO:待实现从apmsg到apollo msg转换
//     //时间戳为ms
//     static int exist_probability[8] = {0, 25, 50, 75, 90, 95, 99, 100};
//     double stamp = ap_msg.header.stamp.sec * 1e3 + ap_msg.header.stamp.nsec / 1e6;
//     msg->mutable_object_list_status_151()->set_timestamp_odh(stamp);
//     int num_of_obj = ap_msg.trackList.size();
//     msg->mutable_contiobs()->Clear();
//     for (int i = 0; i < num_of_obj; i++) {
//       ara::rtrack::RadarTrack ap_msg_obj = ap_msg.trackList[i];
//       ::apollo::drivers::ContiRadarObs* object = msg->add_contiobs();
//       object->set_objectid_00(ap_msg_obj.id);
//       object->set_longdistance_00(ap_msg_obj.x);
//       object->set_latdistance_00(ap_msg_obj.y);
//       object->set_absolutelongvelo_00(ap_msg_obj.vx);
//       object->set_absolutelatvelo_00(ap_msg_obj.vy);
//       object->set_absolutelongacc_00(ap_msg_obj.ax);
//       object->set_absolutelatacc_00(ap_msg_obj.ay);
//       object->set_longdistancestd_00(ap_msg_obj.xRms);    /*x~x*/
//       object->set_corrcoeff_x_y_00(DBL_MAX);              /*x~y*/
//       object->set_corrcoeff_x_vx_00(DBL_MAX);             /*x~vx*/
//       object->set_corrcoeff_x_vy_00(DBL_MAX);             /*x~vy*/
//       object->set_corrcoeff_x_y_00(DBL_MAX);              /*y~x*/
//       object->set_lateraldistancestd_00(ap_msg_obj.yRms); /*y~y*/
//       object->set_corrcoeff_y_vx_00(DBL_MAX);             /*y~vx*/
//       object->set_corrcoeff_y_vy_00(DBL_MAX);             /*y~vy*/
//       object->set_corrcoeff_x_vx_00(DBL_MAX);             /*vx~x*/
//       object->set_corrcoeff_y_vx_00(DBL_MAX);             /*vx~y*/
//       object->set_abslongvelostd_00(ap_msg_obj.vxRms);    /*vx~vx*/
//       object->set_corrcoeff_vx_vy_00(DBL_MAX);            /*vx~vy*/
//       object->set_corrcoeff_x_vy_00(DBL_MAX);             /*vy~x*/
//       object->set_corrcoeff_y_vy_00(DBL_MAX);             /*vy~y*/
//       object->set_corrcoeff_vx_vy_00(DBL_MAX);            /*vy~vx*/
//       object->set_abslatvelostd_00(ap_msg_obj.vyRms);     /*vy~vy*/
//       for (int j = 0; j < 8; j++) {
//         if (ap_msg_obj.existProbability == j) {
//           object->set_existenceprob_00(exist_probability[j]);
//           break;
//         }
//       }
//       object->set_abslongaccstd_00(ap_msg_obj.axRms);
//       object->set_abslongaccstd_00(ap_msg_obj.ayRms);
//       int track_state_sign = ap_msg_obj.trackState;
//       if (track_state_sign == 1) {
//         object->set_measurementstatus_00(
//             Measurementstatus_00Type::
//                 ContiRadarObs_Measurementstatus_00Type_MEASUREMENTSTATUS_00_NEW);
//       } else if (track_state_sign == 2) {
//         object->set_measurementstatus_00(
//             Measurementstatus_00Type::
//                 ContiRadarObs_Measurementstatus_00Type_MEASUREMENTSTATUS_00_MEASURED);
//       } else if (track_state_sign == 3 || track_state_sign == 4 || track_state_sign == 5) {
//         object->set_measurementstatus_00(
//             Measurementstatus_00Type::
//                 ContiRadarObs_Measurementstatus_00Type_MEASUREMENTSTATUS_00_PREDICTED);
//       } else {
//         object->set_measurementstatus_00(
//             Measurementstatus_00Type::
//                 ContiRadarObs_Measurementstatus_00Type_MEASUREMENTSTATUS_00_INVALID);
//       }

//       int move_state_sign = ap_msg_obj.movProperty;
//       if (move_state_sign == 0 || move_state_sign == 2 || move_state_sign == 6 ||
//           move_state_sign == 14) {
//         object->set_movementstatus_00(
//             Movementstatus_00Type::ContiRadarObs_Movementstatus_00Type_MOVEMENTSTATUS_00_MOVING);
//       } else if (move_state_sign == 1 || move_state_sign == 3 || move_state_sign == 5) {
//         object->set_movementstatus_00(
//             Movementstatus_00Type::
//                 ContiRadarObs_Movementstatus_00Type_MOVEMENTSTATUS_00_STATIONARY);
//       } else {
//         object->set_movementstatus_00(
//             Movementstatus_00Type::ContiRadarObs_Movementstatus_00Type_MOVEMENTSTATUS_00_UNDEFINED);
//       }
//       object->set_radarcrosssection_00(DBL_MAX);
//       object->set_yawangle_00(DBL_MAX);
//       object->set_yawstd_00(DBL_MAX);
//       object->set_objectage_00(DBL_MAX);
//       object->set_isnew_00(Isnew_00Type::ContiRadarObs_Isnew_00Type_ISNEW_00_TRUE);
//     }
//   }

//   static void convert(const Type&, APType*) {
//     ASSERT(false && "radar deserialize not implemented.");
//   }
// };

template <>
struct AdaptorTraits<::stoic::proto::geometry::Vector3, true> {
  using Type = ::stoic::proto::geometry::Vector3;
  using APType = ::ara::rawdata::RawDataStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    msg->ParseFromArray(&ap_msg.rawdata[0], ap_msg.rawdata.size());
  }

  static void convert(const Type& msg, APType* ap_msg) {
    auto len = msg.ByteSizeLong();
    ap_msg->rawdata.resize(len);
    msg.SerializeToArray(&ap_msg->rawdata[0], len);
  }
};

template <>
struct AdaptorTraits<::caic_localization::LocalizationEstimation, true> {
  using Type = ::caic_localization::LocalizationEstimation;
  using ProtoType = ::apollo::localization::LocalizationEstimate;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
    msg->msf_status.gnsspos_position_type = static_cast<caic_localization::GnssPositionType>(
        proto_msg.msf_status().gnsspos_position_type());
    msg->pose.pose_info.position.x = proto_msg.pose().position().x();
    msg->pose.pose_info.position.y = proto_msg.pose().position().y();
    msg->pose.pose_info.position.z = proto_msg.pose().position().z();
    msg->pose.pose_info.quaternion.x = proto_msg.pose().orientation().qx();
    msg->pose.pose_info.quaternion.y = proto_msg.pose().orientation().qy();
    msg->pose.pose_info.quaternion.z = proto_msg.pose().orientation().qz();
    msg->pose.pose_info.quaternion.w = proto_msg.pose().orientation().qw();
    msg->pose.heading = proto_msg.pose().heading();
    msg->pose.linear_velocity_info.linear_velocity.x = proto_msg.pose().linear_velocity().x();
    msg->pose.linear_velocity_info.linear_velocity.y = proto_msg.pose().linear_velocity().y();
    msg->pose.linear_velocity_info.linear_velocity.z = proto_msg.pose().linear_velocity().z();
    msg->pose.angle.roll = proto_msg.pose().euler_angles().x();
    msg->pose.angle.pitch = proto_msg.pose().euler_angles().y();
    msg->pose.angle.yaw = proto_msg.pose().euler_angles().z();
    msg->pose.linear_acceleration_info.linear_acceleration.x =
        proto_msg.pose().linear_acceleration().x();
    msg->pose.linear_acceleration_info.linear_acceleration.y =
        proto_msg.pose().linear_acceleration().y();
    msg->pose.linear_acceleration_info.linear_acceleration.z =
        proto_msg.pose().linear_acceleration().z();
    msg->pose.angular_velocity_info.angular_velocity.x = proto_msg.pose().angular_velocity().x();
    msg->pose.angular_velocity_info.angular_velocity.y = proto_msg.pose().angular_velocity().y();
    msg->pose.angular_velocity_info.angular_velocity.z = proto_msg.pose().angular_velocity().z();
    msg->pose.angular_velocity_vrf_info.angular_velocity.x =
        proto_msg.pose().angular_velocity_vrf().x();
    msg->pose.angular_velocity_vrf_info.angular_velocity.y =
        proto_msg.pose().angular_velocity_vrf().y();
    msg->pose.angular_velocity_vrf_info.angular_velocity.z =
        proto_msg.pose().angular_velocity_vrf().z();
    msg->pose.pose_info.covariance.data[0] = pow(proto_msg.uncertainty().position_std_dev().x(), 2);
    msg->pose.pose_info.covariance.data[7] = pow(proto_msg.uncertainty().position_std_dev().y(), 2);
    msg->pose.pose_info.covariance.data[14] =
        pow(proto_msg.uncertainty().position_std_dev().z(), 2);
    msg->pose.pose_info.covariance.data[28] =
        pow(proto_msg.uncertainty().orientation_std_dev().x(), 2);
    msg->pose.pose_info.covariance.data[21] =
        pow(proto_msg.uncertainty().orientation_std_dev().y(), 2);
    msg->pose.pose_info.covariance.data[35] =
        pow(proto_msg.uncertainty().orientation_std_dev().z(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[0] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().x(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[4] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().y(), 2);
    msg->pose.linear_velocity_info.linear_velocity_covariance.data[8] =
        pow(proto_msg.uncertainty().linear_velocity_std_dev().z(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[0] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().x(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[4] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().y(), 2);
    msg->pose.linear_acceleration_info.linear_acceleration_covariance.data[8] =
        pow(proto_msg.uncertainty().linear_acceleration_std_dev().z(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[0] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().x(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[4] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().y(), 2);
    msg->pose.angular_velocity_info.angular_velocity_covariance.data[8] =
        pow(proto_msg.uncertainty().angular_velocity_std_dev().z(), 2);
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    static long long localization_seq_num_ = 0;
    std::string module_name = "localization_proto";
    auto* header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_module_name(module_name);
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
    header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));

    proto_msg->set_measurement_time(timestamp * 1.0 / S_2_US);
    auto mutable_msf_status = proto_msg->mutable_msf_status();
    mutable_msf_status->set_gnsspos_position_type(
        static_cast<apollo::localization::GnssPositionType>(msg.msf_status.gnsspos_position_type));

    auto mutable_pose = proto_msg->mutable_pose();
    // position
    mutable_pose->mutable_position()->set_x(msg.pose.pose_info.position.x);
    mutable_pose->mutable_position()->set_y(msg.pose.pose_info.position.y);
    mutable_pose->mutable_position()->set_z(msg.pose.pose_info.position.z);

    // orientation
    mutable_pose->mutable_orientation()->set_qx(msg.pose.pose_info.quaternion.x);
    mutable_pose->mutable_orientation()->set_qy(msg.pose.pose_info.quaternion.y);
    mutable_pose->mutable_orientation()->set_qz(msg.pose.pose_info.quaternion.z);
    mutable_pose->mutable_orientation()->set_qw(msg.pose.pose_info.quaternion.w);

    double heading = msg.pose.heading;

    mutable_pose->set_heading(heading);

    // linear velocity
    mutable_pose->mutable_linear_velocity()->set_x(msg.pose.linear_velocity_info.linear_velocity.x);
    mutable_pose->mutable_linear_velocity()->set_y(msg.pose.linear_velocity_info.linear_velocity.y);
    mutable_pose->mutable_linear_velocity()->set_z(msg.pose.linear_velocity_info.linear_velocity.z);

    mutable_pose->mutable_euler_angles()->set_x(msg.pose.angle.roll);
    mutable_pose->mutable_euler_angles()->set_y(msg.pose.angle.pitch);
    mutable_pose->mutable_euler_angles()->set_z(msg.pose.angle.yaw);

    mutable_pose->mutable_linear_acceleration()->set_x(
        msg.pose.linear_acceleration_info.linear_acceleration.x);
    mutable_pose->mutable_linear_acceleration()->set_y(
        msg.pose.linear_acceleration_info.linear_acceleration.y);
    mutable_pose->mutable_linear_acceleration()->set_z(
        msg.pose.linear_acceleration_info.linear_acceleration.z);
    mutable_pose->mutable_linear_acceleration_vrf()->set_x(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.x);
    mutable_pose->mutable_linear_acceleration_vrf()->set_y(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.y);
    mutable_pose->mutable_linear_acceleration_vrf()->set_z(
        msg.pose.linear_acceleration_vrf_info.linear_acceleration.z);

    // angular_velocity
    mutable_pose->mutable_angular_velocity()->set_x(
        msg.pose.angular_velocity_info.angular_velocity.x);
    mutable_pose->mutable_angular_velocity()->set_y(
        msg.pose.angular_velocity_info.angular_velocity.y);
    mutable_pose->mutable_angular_velocity()->set_z(
        msg.pose.angular_velocity_info.angular_velocity.z);

    mutable_pose->mutable_angular_velocity_vrf()->set_x(
        msg.pose.angular_velocity_vrf_info.angular_velocity.x);
    mutable_pose->mutable_angular_velocity_vrf()->set_y(
        msg.pose.angular_velocity_vrf_info.angular_velocity.y);
    mutable_pose->mutable_angular_velocity_vrf()->set_z(
        msg.pose.angular_velocity_vrf_info.angular_velocity.z);

    auto mutable_uncertainty = proto_msg->mutable_uncertainty();
    // position_std_dev
    mutable_uncertainty->mutable_position_std_dev()->set_x(
        sqrt(msg.pose.pose_info.covariance.data[0]));
    mutable_uncertainty->mutable_position_std_dev()->set_y(
        sqrt(msg.pose.pose_info.covariance.data[7]));
    mutable_uncertainty->mutable_position_std_dev()->set_z(
        sqrt(msg.pose.pose_info.covariance.data[14]));

    // orientation_std_dev
    mutable_uncertainty->mutable_orientation_std_dev()->set_x(
        sqrt(msg.pose.pose_info.covariance.data[28]));
    mutable_uncertainty->mutable_orientation_std_dev()->set_y(
        sqrt(msg.pose.pose_info.covariance.data[21]));
    mutable_uncertainty->mutable_orientation_std_dev()->set_z(
        sqrt(msg.pose.pose_info.covariance.data[35]));

    // linear_velocity_std_dev
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_x(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[0]));
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_y(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[4]));
    mutable_uncertainty->mutable_linear_velocity_std_dev()->set_z(
        sqrt(msg.pose.linear_velocity_info.linear_velocity_covariance.data[8]));

    // linear_acceleration_std_dev
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_x(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[0]));
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_y(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[4]));
    mutable_uncertainty->mutable_linear_acceleration_std_dev()->set_z(
        sqrt(msg.pose.linear_acceleration_info.linear_acceleration_covariance.data[8]));

    //
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_x(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[0]));
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_y(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[4]));
    mutable_uncertainty->mutable_angular_velocity_std_dev()->set_z(
        sqrt(msg.pose.angular_velocity_info.angular_velocity_covariance.data[8]));

    std::string localization_state_message = "proto_msg state is good";
    proto_msg->set_state_message(localization_state_message);
  }
};

template <>
struct AdaptorTraits<::stoic::msgs::localization::LaneLineOdometry, true> {
  using Type = ::stoic::msgs::localization::LaneLineOdometry;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    // TODO:
  }

  static void convert(const Type& msg, APType* ap_msg) {
    // TODO:
  }
};

template <>
struct AdaptorTraits<::caic_perception::PerceptionTrafficLights, true> {
  using Type = ::caic_perception::PerceptionTrafficLights;
  using ProtoType = ::apollo::perception::TrafficLightDetection;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
    for (int i = 0; i < proto_msg.traffic_light().size(); ++i) {
      msg->traffic_lights[i].id = proto_msg.traffic_light(i).id();
      msg->traffic_lights[i].confidence = proto_msg.traffic_light(i).confidence();
      msg->traffic_lights[i].color =
          static_cast<caic_perception::TrafficLight::Color>(proto_msg.traffic_light(i).color());
      msg->traffic_lights[i].blink = proto_msg.traffic_light(i).blink();
      msg->traffic_lights[i].remaining_time = proto_msg.traffic_light(i).remaining_time();
    }
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
    for (caic_perception::TrafficLight tl : msg.traffic_lights) {
      apollo::perception::TrafficLight* tl_proto_rlt = proto_msg->add_traffic_light();
      tl_proto_rlt->set_id(tl.id);
      tl_proto_rlt->set_confidence(tl.confidence);
      tl_proto_rlt->set_color(convert_color(tl.color));
      tl_proto_rlt->set_blink(tl.blink);
      tl_proto_rlt->set_remaining_time(tl.remaining_time);
    }
  }

  static apollo::perception::TrafficLight_Color convert_color(
      const caic_perception::TrafficLight::Color& caic_color) {
    apollo::perception::TrafficLight_Color apollo_color =
        apollo::perception::TrafficLight_Color::TrafficLight_Color_UNKNOWN;
    switch (caic_color) {
      case caic_perception::TrafficLight::Color::RED:
        apollo_color = apollo::perception::TrafficLight_Color::TrafficLight_Color_RED;
        break;
      case caic_perception::TrafficLight::Color::GREEN:
        apollo_color = apollo::perception::TrafficLight_Color::TrafficLight_Color_GREEN;
        break;
      case caic_perception::TrafficLight::Color::YELLOW:
        apollo_color = apollo::perception::TrafficLight_Color::TrafficLight_Color_YELLOW;
        break;
      case caic_perception::TrafficLight::Color::BLACK:
        apollo_color = apollo::perception::TrafficLight_Color::TrafficLight_Color_BLACK;
        break;
      case caic_perception::TrafficLight::Color::UNKNOWN:
      default:
        apollo_color = apollo::perception::TrafficLight_Color::TrafficLight_Color_UNKNOWN;
    }
    return apollo_color;
  }
};

template <>
struct AdaptorTraits<::caic_perception::PerceptionObjects, true> {
  using Type = ::caic_perception::PerceptionObjects;
  using ProtoType = ::apollo::perception::PerceptionObstacles;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // TODO:待补充完整
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
  }
};

template <>
struct AdaptorTraits<::caic_object_fusion::FusionObjects, true> {
  using Type = ::caic_object_fusion::FusionObjects;
  using ProtoType = ::apollo::perception::PerceptionObstacles;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // TODO:待补充完整
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
  }
};

template <>
struct AdaptorTraits<::stoic::msgs::perception::ObjsPostType, true> {
  using Type = ::stoic::msgs::perception::ObjsPostType;
  using ProtoType = ::apollo::perception::vision::PerceptionObstacles;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // TODO:待补充完整
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
  }
};

template <>
struct AdaptorTraits<::stoic::msgs::perception::LaneLinePostType, true> {
  using Type = ::stoic::msgs::perception::LaneLinePostType;
  using ProtoType = ::apollo::perception::PerceptionObstacles;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // TODO:待补充完整
    msg->header.stamp = proto_msg.header().timestamp_sec() * S_2_US;
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
  }
};

template <>
struct AdaptorTraits<::caic_prediction::PredictionObjects, true> {
  using Type = ::caic_prediction::PredictionObjects;
  using ProtoType = ::apollo::prediction::PredictionObstacles;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    if (nullptr == msg) return;

    msg->prediction_objects.clear();

    double s_2_us_ratio = 1'000'000;

    if (proto_msg.has_header()) {
      msg->header.stamp = proto_msg.header().timestamp_sec() * s_2_us_ratio;
      msg->header.frame_id = proto_msg.header().frame_id();
      msg->header.seq = proto_msg.header().sequence_num();
    }
    msg->meta.start_timestamp_us = proto_msg.start_timestamp() * s_2_us_ratio;
    msg->meta.end_timestamp_us = proto_msg.end_timestamp() * s_2_us_ratio;
    msg->meta.sensor_timestamp_us = proto_msg.header().lidar_timestamp() * s_2_us_ratio;

    for (const ::apollo::prediction::PredictionObstacle& prediction_obstacle :
         proto_msg.prediction_obstacle()) {
      ::caic_prediction::PredictionObject pred_obj;

      pred_obj.id = prediction_obstacle.perception_obstacle().id();
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_ID;

      pred_obj.is_static = prediction_obstacle.is_static();
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_IS_STATIC;

      pred_obj.timestamp = prediction_obstacle.timestamp() * s_2_us_ratio;
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_TIMESTAMP;

      pred_obj.predicted_period = prediction_obstacle.predicted_period();
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_PREDICTED_PERIOD;

      ::caic_prediction::ObstaclePriority priority_msg =
          ::caic_prediction::ObstaclePriority::NORMAL;

      if (::apollo::prediction::ObstaclePriority::CAUTION ==
          prediction_obstacle.priority().priority()) {
        priority_msg = ::caic_prediction::ObstaclePriority::CAUTION;
      } else if (::apollo::prediction::ObstaclePriority::IGNORE ==
                 prediction_obstacle.priority().priority()) {
        priority_msg = ::caic_prediction::ObstaclePriority::IGNORE;
      } else {
        priority_msg = ::caic_prediction::ObstaclePriority::NORMAL;
      }
      pred_obj.priority = priority_msg;
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_PRIORITY;

      pred_obj.trajectories_in_world.clear();
      for (const ::apollo::prediction::Trajectory& trajectory : prediction_obstacle.trajectory()) {
        ::caic_prediction::Trajectory traj;
        traj.probability = trajectory.probability();

        int k = 0;
        for (const ::apollo::geometry::TrajectoryPoint& trajectory_point_proto :
             trajectory.trajectory_point()) {
          traj.trajectory_points[k].path_point.x = trajectory_point_proto.path_point().x();
          traj.trajectory_points[k].path_point.y = trajectory_point_proto.path_point().y();
          traj.trajectory_points[k].path_point.z = trajectory_point_proto.path_point().z();

          traj.trajectory_points[k].path_point.theta = trajectory_point_proto.path_point().theta();
          traj.trajectory_points[k].path_point.kappa = trajectory_point_proto.path_point().kappa();

          traj.trajectory_points[k].path_point.s = trajectory_point_proto.path_point().s();
          traj.trajectory_points[k].path_point.dkappa =
              trajectory_point_proto.path_point().dkappa();
          traj.trajectory_points[k].path_point.ddkappa =
              trajectory_point_proto.path_point().ddkappa();

          traj.trajectory_points[k].path_point.lane_id =
              trajectory_point_proto.path_point().lane_id();
          traj.trajectory_points[k].path_point.x_derivative =
              trajectory_point_proto.path_point().x_derivative();
          traj.trajectory_points[k].path_point.y_derivative =
              trajectory_point_proto.path_point().y_derivative();

          traj.trajectory_points[k].v = trajectory_point_proto.v();
          traj.trajectory_points[k].a = trajectory_point_proto.a();

          traj.trajectory_points[k].relative_time_us =
              trajectory_point_proto.relative_time() * s_2_us_ratio;
          traj.trajectory_points[k].da = trajectory_point_proto.da();
          traj.trajectory_points[k].steer = trajectory_point_proto.steer();
          k++;
          if ((std::size_t)k >= traj.trajectory_points.size()) break;
        }
        traj.size = k;
        pred_obj.trajectories_in_world.emplace_back(traj);
      }
      pred_obj.available |= pred_obj.PREDICTION_OBJECT_TRAJECTORIES_IN_WORLD;

      msg->prediction_objects.emplace_back(pred_obj);
    }
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    if (nullptr == proto_msg) return;
    proto_msg->Clear();
    // header
    double us_2_s_ratio = 1.0 / 1'000'000;
    // double us_2_ms_ratio = 1.0;

    ::apollo::common::Header* header = proto_msg->mutable_header();
    header->set_timestamp_sec(msg.header.stamp * us_2_s_ratio);  // us -> s
    header->set_lidar_timestamp(msg.meta.sensor_timestamp_us * us_2_s_ratio);
    proto_msg->set_start_timestamp(msg.meta.start_timestamp_us * us_2_s_ratio);
    proto_msg->set_end_timestamp(msg.meta.end_timestamp_us * us_2_s_ratio);

    // 预测目标
    for (size_t i = 0; i < msg.prediction_objects.size(); ++i) {
      ::caic_prediction::PredictionObject pred_obj_struct = msg.prediction_objects[i];
      // 障碍物的id
      ::apollo::prediction::PredictionObstacle* pred_obj_proto =
          proto_msg->add_prediction_obstacle();
      // 设置障碍物id
      pred_obj_proto->mutable_perception_obstacle()->set_id(
          static_cast<int32_t>(pred_obj_struct.id));
      pred_obj_proto->set_timestamp(pred_obj_struct.timestamp * us_2_s_ratio);
      pred_obj_proto->set_predicted_period(pred_obj_struct.predicted_period);

      ::apollo::prediction::ObstaclePriority::Priority priority_proto =
          ::apollo::prediction::ObstaclePriority::NORMAL;
      if (::caic_prediction::ObstaclePriority::CAUTION == pred_obj_struct.priority) {
        priority_proto = ::apollo::prediction::ObstaclePriority::CAUTION;
      } else if (::caic_prediction::ObstaclePriority::IGNORE == pred_obj_struct.priority) {
        priority_proto = ::apollo::prediction::ObstaclePriority::IGNORE;
      } else {
        priority_proto = ::apollo::prediction::ObstaclePriority::NORMAL;
      }

      pred_obj_proto->mutable_priority()->set_priority(priority_proto);  // 优先级转换
      pred_obj_proto->set_is_static(pred_obj_struct.is_static);

      // 遍历轨迹
      for (const ::caic_prediction::Trajectory& trajectory_struct :
           pred_obj_struct.trajectories_in_world) {
        ::apollo::prediction::Trajectory* trajectory_proto = pred_obj_proto->add_trajectory();
        trajectory_proto->set_probability(trajectory_struct.probability);
        // 遍历轨迹点
        for (uint8_t idx = 0; idx < trajectory_struct.size; ++idx) {
          ::caic_std::TrajectoryPoint trajectory_point_struct =
              trajectory_struct.trajectory_points[idx];
          ::apollo::geometry::TrajectoryPoint* trajectory_point_proto =
              trajectory_proto->add_trajectory_point();
          trajectory_point_proto->set_a(trajectory_point_struct.a);
          trajectory_point_proto->set_da(trajectory_point_struct.da);
          trajectory_point_proto->set_relative_time(trajectory_point_struct.relative_time_us *
                                                    us_2_s_ratio);
          trajectory_point_proto->set_steer(trajectory_point_struct.steer);
          trajectory_point_proto->set_v(trajectory_point_struct.v);

          ::apollo::geometry::PathPoint* path_point_proto =
              trajectory_point_proto->mutable_path_point();
          path_point_proto->set_x(trajectory_point_struct.path_point.x);
          path_point_proto->set_y(trajectory_point_struct.path_point.y);
          path_point_proto->set_z(trajectory_point_struct.path_point.z);

          path_point_proto->set_theta(trajectory_point_struct.path_point.theta);
          path_point_proto->set_kappa(trajectory_point_struct.path_point.kappa);
          path_point_proto->set_s(trajectory_point_struct.path_point.s);
          path_point_proto->set_dkappa(trajectory_point_struct.path_point.dkappa);
          path_point_proto->set_ddkappa(trajectory_point_struct.path_point.ddkappa);
          path_point_proto->set_lane_id(trajectory_point_struct.path_point.lane_id);
          path_point_proto->set_x_derivative(trajectory_point_struct.path_point.x_derivative);
          path_point_proto->set_y_derivative(trajectory_point_struct.path_point.y_derivative);
        }
      }
    }
  }
};

template <>
struct AdaptorTraits<::caic_planning::PlanningResult, true> {
  using Type = ::caic_planning::PlanningResult;
  using ProtoType = ::apollo::planning::ADCTrajectory;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) {
    std::shared_ptr<ProtoType> proto_msg = std::make_shared<ProtoType>();
    proto_msg->ParseFromString(ap_msg.string);
    convert_proto(*proto_msg, msg);
  }

  static void convert(const Type& msg, APType* ap_msg) {
    ProtoType proto_msg;
    convert_proto(msg, &proto_msg);
    proto_msg.SerializeToString(&ap_msg->string);
  }

  static void convert_proto(const ProtoType& proto_msg, Type* msg) {
    // TODO:待补充完整
    msg->header.stamp = proto_msg.header().timestamp_sec();
  }

  static void convert_proto(const Type& msg, ProtoType* proto_msg) {
    proto_msg->Clear();
    // header
    auto header = proto_msg->mutable_header();
    double timestamp = msg.header.stamp;
    header->set_timestamp_sec(timestamp * 1.0 / S_2_US);
  }
};

template <>
struct AdaptorTraits<::stoic::msgs::common::Str, true> {
  using Type = ::stoic::msgs::common::Str;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) { msg->str = ap_msg.string; }

  static void convert(const Type& msg, APType* ap_msg) { ap_msg->string = msg.str; }
};

template <typename _T>
struct AdaptorTraits<_T, false> {
  using Type = _T;
  using APType = ::ara::string::StringStruct;

  static void convert(const APType& ap_msg, Type* msg) { msg->ParseFromString(ap_msg.string); }

  static void convert(const Type& msg, APType* ap_msg) { msg.SerializeToString(&ap_msg->string); }
};

template <typename _T>
struct AdaptorTraits<_T, true> {
  using Type = _T;
  using APType = String;

  static void convert(const APType&, Type*) {}

  static void convert(const Type&, APType*) {}
};

template <typename _Msg, typename _APMsg, bool _Adaptive>
inline void callbackAdaptor(const _APMsg& ap_msg, void (*callback)(const _Msg&)) {
  _Msg msg;
  AdaptorTraits<_Msg, _Adaptive>::convert(ap_msg, &msg);
  callback(msg);
}

template <typename _Msg, typename _APMsg, bool _Adaptive>
void callbackAdaptorBoost(const _APMsg& ap_msg,
                          const boost::function<void(std::shared_ptr<_Msg>)>& callback) {
  if constexpr (std::is_same_v<_APMsg, ::ara::string::StringStruct>) {
    std::shared_ptr<_Msg> msg = std::make_shared<_Msg>();
    AdaptorTraits<_Msg, _Adaptive>::convert(ap_msg, msg.get());
    callback(msg);
  } else {
    static stoic::cm::alg::ObjectPool<_Msg> pool(9);
    std::shared_ptr<_Msg> msg = pool.malloc();
    AdaptorTraits<_Msg, _Adaptive>::convert(ap_msg, msg.get());
    callback(msg);
  }
}

}  // namespace stoic::cm
