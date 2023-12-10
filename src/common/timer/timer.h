#pragma once

#if defined MW_ROS_IPC || defined MW_ROS_ORIN
#include <ros/rate.h>
#include <ros/time.h>
#endif

#include <time.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace smartsil {

// time since epoch
using TimeUS = int64_t;  // microsecond
using TimeMS = int64_t;  // millisecond
using TimeSEC = double;  // second
using TimeDelta = int32_t;

using Time = TimeUS;

static constexpr int32_t kMilliPerSecond = 1'000'000;
static constexpr int32_t kMicroPerSecond = 1'000;

struct Timer {
  static TimeUS now_us() {
    std::chrono::microseconds microsec = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeUS(microsec.count());
  }

  static TimeMS now_ms() {
    std::chrono::milliseconds millisec = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeMS(millisec.count());
  }

  static TimeSEC now_sec() {
    std::chrono::seconds sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return TimeSEC(sec.count());
  }

  static void sleep_us(const TimeUS& duration) {
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
  }

  static void sleep_ms(const TimeMS& duration) {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
  }

  static Time now() {
    Time now;
#if defined MW_ROS_IPC || defined MW_ROS_ORIN
    now = ros::Time::now().toSec() * 1000000;
#endif

#ifdef MW_AP
    struct timespec tp {};
    (void)clock_gettime(CLOCK_REALTIME, &tp);
    now = static_cast<Time>(tp.tv_sec * 1000000 + tp.tv_nsec * 1e-3);
#endif

#ifdef MW_RT
    static autoplt::simtime::SimClockSource sim;
    auto cur_sim_time = sim.GetSimNow();
    now = static_cast<Time>(cur_sim_time * 1e-3);
#endif

    return now;
  }
};

#if defined MW_ROS_IPC || defined MW_ROS_ORIN
typedef ::ros::Rate Rate;
#else
class Rate {
 public:
  Rate(const size_t& rate) : rate_(rate) {
    gap_ = static_cast<double>(kMilliPerSecond) / rate;
    expect_time_ = Timer::now_us() + gap_;
  }

  void sleep() {
    TimeUS duration = expect_time_ - Timer::now_us();
    if (duration > 0) {
      Timer::sleep_us(duration);
    }
    expect_time_ = Timer::now_us() + gap_;
  }

 private:
  size_t rate_;
  TimeUS gap_;
  TimeUS expect_time_;
};
#endif

class TicToc {
 public:
  TicToc() { tic(); }

  inline void tic() { start_ = Timer::now_us(); }

  inline TimeDelta toc() { return Timer::now_us() - start_; }

 private:
  Time start_;
};

class Performance {
 public:
  // template <typename... Args>
  // Performance(Args&&... args) {
  //   ((description_ << std::forward<Args>(args) << " "), ...);
  //   is_printf_ = 1;
  // }
  Performance(const std::string& str);

  Performance(const std::string& str, const bool& is_printf);

  inline void tic() { timer_.tic(); }

  inline void toc() {
    TimeDelta duration = timer_.toc();
    std::streamsize ss = std::cout.precision();
    if (duration < 100 && is_printf_) {  // PRQA S 4400 # performance used.
      // std::cerr << "WARNING: Performance tic toc is not suitable for low cost function."
      //           << std::endl;
    }
    if (is_printf_) {
      std::cout << std::setprecision(2);  // PRQA S 4400 # performance used.
      std::cout << description_.str() << "cost : " << std::fixed << duration / 1'000.0
                << " ms"  // PRQA S 4400 # performance used.
                << std::endl;
      std::cout << std::setprecision(ss);
    }
    // std::cout << description_.str() << "cost : " << duration << " us" << std::endl;
  }

  ~Performance() { toc(); }

 private:
  std::stringstream description_;
  TicToc timer_;

  bool is_printf_;
};

using PERF = Performance;

template <typename T>
inline Time getAverageTime(T& frame) {
  Time average = 0;
  for (auto& ele : frame->msg_mp_) {
    average += ele.second.header.stamp;
  }
  return average / (int64_t)frame->msg_mp_.size();
};

}  // namespace smartsil

