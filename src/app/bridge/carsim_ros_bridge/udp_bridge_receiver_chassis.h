#include <ros/ros.h>
#include <std_msgs/String.h>

#include "common/websocket/websocket_server.hpp"
#include "proto/core/sim_chassis.pb.h"
#include "proto/bridge/bridge_config.pb.h"

#include "bridge/common/bridge_gflags.h"
#include "bridge/common/vehicle_chassis.h"

namespace smartsim {
namespace bridge {

class UDPBridgeReceiverChassis : public common::WebSocketServer<VehicleChassis> {
  public:
    explicit UDPBridgeReceiverChassis(ros::NodeHandle& nh);
    ~UDPBridgeReceiverChassis() = default;
    
    /**
     * @brief main logic of module, runs periodically trigger by timer.
     * 
     */
    void RunOnce();

  private:
    /**
     * @brief publish sim chassis data.
     * 
     * @param carsim_chassis 
     */
    void PublishSimChassisPb(core::SimChassis* carsim_chassis);
    VehicleChassis veh_chassis_;

  private:
    ros::NodeHandle& nh_;
    ros::Publisher publisher_;

    unsigned int bind_port_ = 0;
    std::string topic_name_ = "";
    common::UdpConnection_t udpConnection_t;
    
};

}  // namespace bridge
}  // namespace smartsim