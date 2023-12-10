#ifndef SMARTSIL_SRC_COMMON_UTIL_YAML_UTIL_H_
#define SMARTSIL_SRC_COMMON_UTIL_YAML_UTIL_H_

#include "yaml-cpp/yaml.h"
#include <string>

namespace smartsil::common {
class YamlUtil {
 public:
  /**
   * @brief Get a string value from the given yaml[key].
   * @return Whether the field exists and is a valid string.
   */
  static YAML::Node GetValue(const YAML::Node &yaml, const std::string &key);
};

} // namespace smartsil::common

#endif //SMARTSIL_SRC_COMMON_UTIL_YAML_UTIL_H_