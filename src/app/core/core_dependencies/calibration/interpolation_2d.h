// Copyright 2023 The Xu Cong Authors. All Rights Reserved.

#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

/**
 * @namespace smartsim::control
 * @brief smartsim::control
 */
namespace smartsim {
namespace core {
namespace control {
/**
 * @class Interpolation2D
 *
 * @brief linear interpolation from key (double, double) to one double value.
 */
class Interpolation2D {
 public:
  typedef std::vector<std::tuple<double, double, double>> DataType;
  typedef std::pair<double, double> KeyType;

  Interpolation2D() = default;

  /**
   * @brief initialize Interpolation2D internal table
   * @param xyz passing interpolation initialization table data
   * @return true if init is ok.
   */
  bool Init(const DataType &xyz);

  /**
   * @brief linear interpolate from 2D key (double, double) to one double value.
   * @param xyz passing interpolation initialization table data
   * @return true if init is ok.
   */
  double Interpolate(const KeyType &xy) const;

 private:
  double InterpolateYz(const std::map<double, double> &yz_table,
                       double y) const;

  double InterpolateValue(const double value_before, const double dist_before,
                          const double value_after,
                          const double dist_after) const;

  std::map<double, std::map<double, double>> xyz_;
};

}  // namespace control
}  // namespace core
}  // namespace smartcar
