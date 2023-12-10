#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>

#include "bridge/common/VtdRDBIcd.h"
#include "common/proto/config/bridge_config.pb.h"
#include "common/proto/perception/vtd_perception_obtacles.pb.h"

namespace smartsim {
namespace bridge {

class UDPBridgeReceiverVtdObstacles {
  public:
    explicit UDPBridgeReceiverVtdObstacles(ros::NodeHandle& nh);
    ~UDPBridgeReceiverVtdObstacles() = default;
    bool recv(int sockfd);
    bool run();

  private:
    bool IsTimeout(double time_stamp);

  private:
    ros::NodeHandle& nh_;
    ros::Publisher publisher_;

    unsigned int bind_port_ = 0;
    std::string proto_name_ = "";
    std::string topic_name_ = "";
    bool enable_timeout_ = true;
    std::mutex mutex_;
    uint32_t FRAME_SIZE = 1024;
    int sock_fd;
    struct sockaddr_in client_addr;
    socklen_t sock_len = static_cast<socklen_t>(sizeof(sockaddr));

    smartsim::vtd::perception::VtdPerceptionObstacles vtd_perception_obstacles_;
    RDB_OBJECT_STATE perception_obstacles;
    
};

}  // namespace bridge
}  // namespace smartsim