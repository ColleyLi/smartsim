#ifndef SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_SEND_TRIGGER_H_
#define SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_SEND_TRIGGER_H_

#include "yaml-cpp/yaml.h"
#include "memory"
#include "sim_interface.h"

#include "common/time/timer.h"
#include "common/network/udpsocket_client.h"
#include "common/network/udpsocket_server.h"
#include "app/simulator/vtd_integration/vtd_dependencies/common/vtd_integration_gflags.h"

namespace smartsil::app::vtd_integration {

class SendTrigger : public UdpSocketClient<sensor_data::MSG_TRIGGER_t>,
                    public UdpSocketServer<vehicle_state::Ego> {
 public:
  SendTrigger();
  ~SendTrigger() {
    delete rdbTrigger;
  }
  void* sendTrigger(const double& simTime, const unsigned int& simFrame);
  void run();


 private:
  sensor_data::MSG_TRIGGER_t* rdbTrigger;

};

} // namespace smartsil::app::vtd_integration
#endif //SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_SEND_TRIGGER_H_

