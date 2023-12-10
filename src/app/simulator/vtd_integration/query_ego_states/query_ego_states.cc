#include "app/simulator/vtd_integration/query_ego_states/query_ego_states.h"

namespace smartsil::app::vtd_integration {

QueryVehicleStates::QueryVehicleStates() {
  node = YAML::LoadFile(FLAGS_vtd_config);
  std::string key = "ego_vehicle_states";
  YAML::Node result = YamlUtil::GetValue(node, key);
  if (!result.IsNull() && result.IsMap()) {
    bind_port = result["bind_port"].as<int>();
  } else {
    LOG_ERROR << "Key '" << key << "' not found in the YAML data.";
  }
  connectToUdpClient(bind_port);
  connectToUdpServer("127.0.0.1", RDB_FEEDBACK_PORT);
}

void* QueryVehicleStates::msgHandle(const vehicle_state::Ego &msg) {
  egoData = new sensor_data::MSG_EGO_STATES_t;
  memset(egoData, 0, sizeof(sensor_data::MSG_EGO_STATES_t));

  // Header
  egoData->hdr.magicNo = RDB_MAGIC_NO;
  egoData->hdr.version = RDB_VERSION;
  egoData->hdr.headerSize = sizeof(RDB_MSG_HDR_t);
  egoData->hdr.dataSize = 5 * sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_OBJECT_STATE_t) + 4 * sizeof(RDB_WHEEL_t) + sizeof(RDB_TRIGGER_t);
  egoData->hdr.frameNo = msg.simFrame;
  egoData->hdr.simTime = msg.simTime;

  // StartOfFrame
  egoData->entrySOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entrySOF.dataSize = 0;
  egoData->entrySOF.elementSize = 0;
  egoData->entrySOF.pkgId = RDB_PKG_ID_START_OF_FRAME;
  egoData->entrySOF.flags = 0;

  // trigger
  egoData->entryTrigger.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryTrigger.dataSize   = sizeof(RDB_TRIGGER_t);
  egoData->entryTrigger.elementSize = sizeof(RDB_TRIGGER_t);
  egoData->entryTrigger.pkgId = RDB_PKG_ID_TRIGGER;
  egoData->entryTrigger.flags = 0;
  egoData->rdbTrigger.deltaT  = 0.02;
  egoData->rdbTrigger.frameNo = 1;

  // Ego states
  egoData->entryEgo.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryEgo.dataSize = sizeof(RDB_OBJECT_STATE_t);
  egoData->entryEgo.elementSize = sizeof(RDB_OBJECT_STATE_t);
  egoData->entryEgo.pkgId = RDB_PKG_ID_OBJECT_STATE;
  egoData->entryEgo.flags = RDB_PKG_FLAG_EXTENDED;

  sprintf(egoData->EgoState.base.name, "&s", "Ego");
  egoData->EgoState.base.id = 1;
  egoData->EgoState.base.category = RDB_OBJECT_CATEGORY_PLAYER;
  egoData->EgoState.base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
  egoData->EgoState.base.visMask = RDB_OBJECT_VIS_FLAG_ALL;
  egoData->EgoState.base.geo.dimX = 5.209;
  egoData->EgoState.base.geo.dimY = 2.20;
  egoData->EgoState.base.geo.dimZ = 1.731;
  egoData->EgoState.base.geo.offX = 0.8;
  egoData->EgoState.base.geo.offY = 0.0;
  egoData->EgoState.base.geo.offZ = 0.4;
  egoData->EgoState.base.pos.x = msg.pos_x;
  egoData->EgoState.base.pos.y = msg.pos_y;
  egoData->EgoState.base.pos.z = msg.pos_z;
  egoData->EgoState.base.pos.h = msg.yaw;
  egoData->EgoState.base.pos.p = msg.pitch;
  egoData->EgoState.base.pos.r = msg.roll;
  egoData->EgoState.base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->EgoState.base.pos.type = RDB_COORD_TYPE_INERTIAL;
  egoData->EgoState.base.pos.system = 0;
  egoData->EgoState.ext.speed.x = msg.linear_velocity_vrf_x;
  egoData->EgoState.ext.speed.y = msg.linear_velocity_vrf_y;
  egoData->EgoState.ext.speed.z = msg.linear_velocity_vrf_z;
  egoData->EgoState.ext.speed.h = msg.angular_velocity_vrf_z;
  egoData->EgoState.ext.speed.p = msg.angular_velocity_vrf_y;
  egoData->EgoState.ext.speed.r = msg.angular_velocity_vrf_x;
  egoData->EgoState.ext.accel.x = msg.linear_acceleration_vrf_x;
  egoData->EgoState.ext.accel.y = msg.linear_acceleration_vrf_y;
  egoData->EgoState.ext.accel.z = msg.linear_acceleration_vrf_z;
  egoData->EgoState.ext.accel.h = msg.aaz;
  egoData->EgoState.ext.accel.p = msg.aay;
  egoData->EgoState.ext.accel.r = msg.aax;
  egoData->EgoState.ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->EgoState.ext.speed.type = RDB_COORD_TYPE_INERTIAL;
  egoData->EgoState.ext.speed.system = 0;
  egoData->EgoState.ext.accel.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  egoData->EgoState.ext.accel.type = RDB_COORD_TYPE_INERTIAL;
  egoData->EgoState.ext.accel.system = 0;

  // Wheel state
  egoData->entryWheel.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryWheel.dataSize = 4 * sizeof(RDB_WHEEL_t);
  egoData->entryWheel.elementSize = sizeof(RDB_WHEEL_t);   // FOUR elements
  egoData->entryWheel.pkgId = RDB_PKG_ID_WHEEL;
  egoData->entryWheel.flags = RDB_PKG_FLAG_EXTENDED;
  egoData->wheel[0].base.playerId = 1;
  egoData->wheel[0].base.id = 0;
  egoData->wheel[0].base.radiusStatic = msg.RRE_L1;
  egoData->wheel[0].base.springCompression = 0;
  egoData->wheel[0].base.rotAngle = msg.Rot_L1;
  egoData->wheel[0].base.slip = 0.5;
  egoData->wheel[0].base.steeringAngle = msg.Steer_L1;
  egoData->wheel[0].ext.vAngular = msg.Avy_L1;

  egoData->wheel[1].base.playerId = 1;
  egoData->wheel[1].base.id = 1;
  egoData->wheel[1].base.radiusStatic = msg.RRE_R1;
  egoData->wheel[1].base.springCompression = 0;
  egoData->wheel[1].base.rotAngle = msg.Rot_R1;
  egoData->wheel[1].base.slip = 0.5;
  egoData->wheel[1].base.steeringAngle = msg.Steer_R1;
  egoData->wheel[1].ext.vAngular = msg.Avy_R1;

  egoData->wheel[2].base.playerId = 1;
  egoData->wheel[2].base.id = 2;
  egoData->wheel[2].base.radiusStatic = msg.RRE_R2;
  egoData->wheel[2].base.springCompression = 0;
  egoData->wheel[2].base.rotAngle = msg.Rot_R2;
  egoData->wheel[2].base.slip = 0.5;
  egoData->wheel[2].base.steeringAngle = msg.Steer_R2;
  egoData->wheel[2].ext.vAngular = msg.Avy_R2;

  egoData->wheel[3].base.playerId = 1;
  egoData->wheel[3].base.id = 3;
  egoData->wheel[3].base.radiusStatic = msg.RRE_L2;
  egoData->wheel[3].base.springCompression = 0;
  egoData->wheel[3].base.rotAngle = msg.Rot_L2;
  egoData->wheel[3].base.slip = 0.5;
  egoData->wheel[3].base.steeringAngle = msg.Steer_L2;
  egoData->wheel[3].ext.vAngular = msg.Avy_L2;

  //EndOfFrame
  egoData->entryEOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  egoData->entryEOF.dataSize = 0;
  egoData->entryEOF.elementSize = 0;
  egoData->entryEOF.pkgId = RDB_PKG_ID_END_OF_FRAME;
  egoData->entryEOF.flags = 0;

  // trigger flags
  sDataReceived = true;

  return static_cast<void*>(egoData);

// useage
//  void* voidPtr = queryVehicleStates(msg);
//  sensor_data::MSG_EGO_STATES_t* msg_ego_states = static_cast<sensor_data::MSG_EGO_STATES_t*>(voidPtr);

}

void QueryVehicleStates::run() {
  // recv ego vehicle states
  vehicle_state::Ego latest_ego;
  recvData(latest_ego);
  fprintf( stderr, "recv ego vehicle state\n" );
  fprintf( stderr, "    simTime = %.3lf, simFrame = %u\n", latest_ego.simTime, latest_ego.simFrame );
  fprintf( stderr, "    position = %.3lf / %.3lf / %.3lf\n", latest_ego.pos_x, latest_ego.pos_y, latest_ego.pos_z );

  //
  void* voidPtr = msgHandle(latest_ego);
  auto msg_ego = static_cast<sensor_data::MSG_EGO_STATES_t*>(voidPtr);
  sendData(*msg_ego);
}

} // namespace smartsil::app::vtd_integration

int main(int argc, char** argv) {
  smartsil::app::vtd_integration::QueryVehicleStates query_vehicle_states;
  while (true) {
    query_vehicle_states.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  }
}
