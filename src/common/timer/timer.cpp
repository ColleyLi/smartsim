#pragma once

#include <iostream>
#include "common/time/timer.h"

namespace smartsil {

Performance::Performance(const std::string& str) {
  description_ << str;
  is_printf_ = 1;
}

Performance::Performance(const std::string& str, const bool& is_printf) {
  description_ << str;
  is_printf_ = is_printf;
}

}  // namespace smartsil