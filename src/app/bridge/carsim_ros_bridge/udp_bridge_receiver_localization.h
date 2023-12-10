#include <ros/ros.h>
#include <std_msgs/String.h>

#include "common/websocket/websocket_server.hpp"
#include "proto/core/sim_localization.pb.h"
#include "proto/bridge/bridge_config.pb.h"

#include "bridge/common/vehicle_localization.h"
#include "bridge/common/bridge_gflags.h"

namespace smartsim {
namespace bridge {

class UDPBridgeReceiverLocalization : public common::WebSocketServer<VehicleLocalization> {
  public:
    explicit UDPBridgeReceiverLocalization(ros::NodeHandle& nh);
    ~UDPBridgeReceiverLocalization() = default;

    /**
     * @brief main logic of module, runs periodically trigger by timer.
     * 
     */
    void RunOnce();

  private:
    /**
     * @brief publish sim loc data.
     * 
     * @param carsim_loc
     */
    void PublishSimLocalizationPb(core::SimLocalization* carsim_loc);
    VehicleLocalization veh_loc_;

  private:
    ros::NodeHandle& nh_;
    ros::Publisher publisher_;

    unsigned int bind_port_ = 0;
    std::string topic_name_ = "";
    common::UdpConnection_t udpConnection_t;
    
};

}  // namespace bridge
}  // namespace smartsim