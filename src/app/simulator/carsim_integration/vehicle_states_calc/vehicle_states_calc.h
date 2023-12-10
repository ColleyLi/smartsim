// header files for standard C libraries
#pragma once

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if (defined(_WIN32) || defined(_WIN64))
#include <windows.h>
#endif

#include <yaml-cpp/yaml.h>
#include "sim_interface.h"

#include "common/network/udpsocket_client.h"
#include "common/network/udpsocket_server.h"
#include "common/util/yaml_util.h"
// vs_integration header file and prototypes for these function
#include "app/simulator/carsim_integration/carsim_dependencies/common/vs_deftypes.h" // VS types and definitions
#include "app/simulator/carsim_integration/carsim_dependencies/common/vs_api.h"      // VS API functions
#include "app/simulator/carsim_integration/carsim_dependencies/common/vs_api_road.h" // VS API functions
#include "app/simulator/carsim_integration/carsim_dependencies/message/msg_manager.h"
#include "app/simulator/carsim_integration/carsim_dependencies/common/vs_integration_gflags.h"


namespace smartsil::app {
namespace carsim_integration {

/* --------------------------------------------------------------------------------
   Set up variables for the model extension. For the path follower, define new units
   and parameters and set default values of the parameters that will be used if
   nothing is specified at run time. Note that you cannot use this section to access
   keywords related to optional components such as powertrains, since these have not
   yet been activated and defined. Such keywords can be accessed after
   vs_setdef_and_read().
--------------------------------------------------------------------------------- */
void external_setdef (void);

/* ---------------------------------------------------------------------------------
   Perform calculations involving the model extensions. This function is called from
   nine places as defined in vs_deftypes.h.
--------------------------------------------------------------------------------- */
void  external_calc (vs_real t, vs_ext_loc where);

/* ---------------------------------------------------------------------------------
   Write information into the current output echo file using the VS API function
   vs_write_to_echo_file. This function is called four times when generating the
   echo file as indicated with the argument where, which can have the values:
   VS_EXT_ECHO_TOP, VS_EXT_ECHO_SYPARS, VS_EXT_ECHO_PARS, and VS_EXT_ECHO_END
   (defined in vs_deftypes.h).
--------------------------------------------------------------------------------- */
void external_echo (vs_ext_loc where);

/* ---------------------------------------------------------------------------------
   Scan a line read from the current input parsfile. Return TRUE if the keyword
   is recognized, FALSE if not.

   keyword -> string with current ALL CAPS keyword to be tested
   buffer  -> string with rest of line from parsfile.
--------------------------------------------------------------------------------- */
vs_bool external_scan(char* keyword, char* buffer);

// define some static variables used for the controller
static vs_real *sTstart; // pointer to start time
static int sUseExternal, // option to use external model.
           sBikeSim, sTruckSim; // is this BikeSim or TruckSim?

// define carsim io variables used for simulation
carsim_inport vs_in;
carsim_outport vs_out;
carsim_init vs_init;

// some websocket used for cross-platform communication
UdpSocketClient<vehicle_state::Chassis> websocket_chassis;
UdpSocketClient<vehicle_state::Gps> websocket_gps;
UdpSocketClient<vehicle_state::Imu> websocket_imu;
UdpSocketServer<vehicle_control::VehicleControl> websocket_control;

YAML::Node config_node;
int control_port;
std::string ego_vehicle_ip;
int ego_vehicle_port;
std::string chassis_ip;
int chassis_port;
std::string gps_ip;
int gps_port;
std::string imu_ip;
int imu_port;


} // namespace vs_integration
} // namespace smartsil::app
