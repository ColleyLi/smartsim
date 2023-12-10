#include <ros/ros.h>
#include <std_msgs/String.h>

#include "bridge/common/vehicle_control.h"
#include "bridge/common/bridge_gflags.h"

#include "common/websocket/websocket_server.hpp"
#include "proto/bridge/bridge_config.pb.h"
#include "proto/core/sim_control.pb.h"

namespace smartsim {
namespace bridge {

class UDPBridgeSenderControl : public WebSocketServer<VehicleControl> {
  public:
    explicit UDPBridgeSenderControl(ros::NodeHandle& nh);
    ~UDPBridgeSenderControl() = default;

    /**
     * @brief main logic of module, runs periodically trigger by timer.
     * 
     */
    void RunOnce();

  private:
    void OnSimControl(const std_msgs::String& msg);

  private:
    ros::NodeHandle& nh_;
    ros::Subscriber subscriber_;

    unsigned int remote_port_ = 0;
    std::string remote_ip_ = "";
    std::string topic_name_ = "";
    std::mutex mutex_;
    VehicleControl veh_ctrl_;
    bool is_control_ready_ = false;
    
    common::UdpConnection_t udpConnection_t;
};

}  // namespace bridge
}  // namespace smartsim