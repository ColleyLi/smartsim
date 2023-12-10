#pragma once

#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "sim_interface.h"
#include "common/network/udpsocket_client.h"

#include "app/simulator/vtd_integration/vtd_dependencies/pattern/task.hpp"
#include "app/simulator/vtd_integration/vtd_dependencies/common/RDBHandler.hh"
#include "app/simulator/vtd_integration/vtd_dependencies/common/viRDBIcd.h"
#include "app/simulator/vtd_integration/vtd_dependencies/udp_port.h"


namespace smartsil::app::vtd_integration {

class FrontLeftCameraDetected : public UdpSocketClient<sensor_data::MSG_SENSOR_FRONT_LEFT_t>,
                                public Task<FrontLeftCameraDetected> {
 public:
  template <typename... OptionArgs>
  FrontLeftCameraDetected(OptionArgs&&... option_args) :
    Task(std::forward<OptionArgs>(option_args)...) {}

  FrontLeftCameraDetected(const TaskOptions& task_options) : Task(task_options) {}

  void run() override;

 private:
  int openPort(int & descriptor, int portNo, const char* serverAddr);
  void initConnections();
  void initConnection(Connection_t &conn);
  void readConnection(Connection_t &conn, bool waitForMessage, bool verbose);

  void parseRDBMessage( Connection_t & conn, RDB_MSG_t* msg );
  void parseRDBMessageEntry( Connection_t & conn, const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );
  void handleRDBitem(const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t &item, bool isExtended, bool isSensor, int countObj);
  void sendObjects(const int &sendPort, const double &simTime, const unsigned int &simFrame, const int &countObj);

 private:
  RDB_OBJECT_STATE_t mObjState;
  RDB_OBJECT_STATE_t mOwnObject;
  sensor_data::MSG_SENSOR_FRONT_LEFT_t* sensor_data_ = new sensor_data::MSG_SENSOR_FRONT_LEFT_t;

  // connection instances
  Connection_t sConnection[MAX_CONNECTIONS];

};

    
} // namespace smartsil::app::vtd_integration

WORKFLOW_ADD_TASK(::smartsil::app::vtd_integration::FrontLeftCameraDetected)
