#pragma once

#if defined MW_ROS_IPC || MW_ROS_ORIN
#include <ros/rate.h>
#include <ros/time.h>
#endif

#include <signal.h>
#include <time.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include "cm/node_handle.h"

#include "cm/base/time.hpp"

namespace cm {
class SignalHandle {
 public:
  static SignalHandle* instance();
  static std::once_flag init_flag;

  void SetState(bool state);
  bool GetState();

  void Destroy();

 private:
  SignalHandle();
  ~SignalHandle();

  static SignalHandle* pSignal;
  std::mutex m_mutex;
  bool m_state;
};

#if defined MW_AP
//中断信号响应函数
static void sigHandler(int sigNum) {
  std::cout << "Catch Signal: " << sigNum << std::endl;
  SignalHandle::instance()->SetState(false);
  std::cout << "ctrl-c caught, exiting..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
#endif

#if defined MW_ART_IPC || MW_ART_ORIN
static void sighandler_cm_art_ctrlc(int signum) {
  // printf("communication模块捕获到退出信号: %d. 程序即将退出.\n", signum);
  g_cm_fource_stop = 1;
}
#endif

#if defined MW_ART_IPC || MW_ART_ORIN
static void sighandler_cm_art_kill15(int signum) {
  // printf("communication模块捕获到退出信号: %d. 程序即将退出.\n", signum);
  g_cm_fource_stop = 1;
}
#endif

template <typename... Args>
inline void init(int32_t argc, char** argv, Args&&... args) {
#if defined MW_ROS_IPC || MW_ROS_ORIN || MW_ROSM
  ros::init(argc, argv, args...);  // ROS节点初始化
#elif defined MW_AP
  // suppress unused warnings
  (void)argc;
  (void)argv;
  (void)(sizeof...(args));
  signal(SIGINT, sigHandler);
  signal(SIGTSTP, sigHandler);
  std::cout << "InitStoicheia AP..." << std::endl;
#elif defined MW_RT
  autoplt::ADSNode::Init(argv[0]);
#elif defined MW_TZ
  std::cout << "TZ do not use init ... " << std::endl;
#elif defined MW_ART_IPC || defined MW_ART_ORIN
  apollo::cyber::Init(argv[0]);
  signal(SIGINT, sighandler_cm_art_ctrlc);
  signal(SIGTERM, sighandler_cm_art_kill15);
#endif
}

inline void uninit() {
#if defined MW_AP
  SignalHandle::instance()->Destroy();
  std::cout << "UninitStoicheia AP." << std::endl;
#endif
}

inline bool ok() {
#if defined MW_ROS_IPC || MW_ROS_ORIN || defined MW_ROSM
  return ros::ok();
#elif defined MW_AP
  return SignalHandle::instance()->GetState();
#elif defined MW_RT
  return apollo::cyber::OK();
#elif defined MW_TZ
  return true;
#elif defined MW_ART_IPC || defined MW_ART_ORIN
  return apollo::cyber::OK();
#endif
}

inline void spin() {
#if (defined MW_ROS_IPC) || (defined MW_ROS_ORIN) || ((defined MW_ROSM))
  ros::waitForShutdown();
#elif (defined MW_AP) || (defined MW_TZ)
  uint32_t duration = 100u;
  while (ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
  }
#elif defined MW_RT
  apollo::cyber::WaitForShutdown();
#elif defined MW_ART_IPC || defined MW_ART_ORIN
  apollo::cyber::WaitForShutdown();
#endif
}

}  // namespace cm
