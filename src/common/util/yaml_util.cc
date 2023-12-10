#include "common/util/yaml_util.h"
#include <iostream>

namespace smartsil::common {

YAML::Node YamlUtil::GetValue(const YAML::Node &yaml, const std::string &key) {
  if (yaml.IsMap() && yaml[key]) {
    return yaml[key];
  }

  if (yaml.IsMap()) {
    for (const auto& entry : yaml) {
      const YAML::Node& value = entry.second;
      YAML::Node result = GetValue(value, key);
      if (!result.IsNull()) {
        return result;
      }
    }
  } else if (yaml.IsSequence()) {
    for (const auto& element : yaml) {
      YAML::Node result = GetValue(element, key);
      if (!result.IsNull()) {
        return result;
      }
    }
  }

  return YAML::Node();

}

} // namespace smartsil::common
