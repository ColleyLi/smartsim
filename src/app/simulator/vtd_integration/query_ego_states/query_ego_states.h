#ifndef SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_
#define SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_

#include "yaml-cpp/yaml.h"
#include "memory"
#include "sim_interface.h"

#include "common/time/timer.h"
#include "common/network/udpsocket_server.h"
#include "common/network/udpsocket_client.h"
#include "app/simulator/vtd_integration/vtd_dependencies/util/yaml_util.h"
#include "app/simulator/vtd_integration/vtd_dependencies/common/vtd_integration_gflags.h"


namespace smartsil::app::vtd_integration {

class QueryVehicleStates : public UdpSocketServer<vehicle_state::Ego>,
                           public UdpSocketClient<sensor_data::MSG_EGO_STATES_t> {
 public:
  QueryVehicleStates();

  ~QueryVehicleStates() {
    delete egoData;
  }
  void* msgHandle(const vehicle_state::Ego & msg);
  void run();

  bool sDataReceived = false;

 private:
  sensor_data::MSG_EGO_STATES_t* egoData;
  YAML::Node node;
  std::string remote_ip;
  int bind_port;


};

} // namespace smartsil::app::vtd_integration
#endif //SMARTSIL_SRC_APP_SIMULATOR_VTD_INTEGRATION_QUERY_EGO_STATES_H_
