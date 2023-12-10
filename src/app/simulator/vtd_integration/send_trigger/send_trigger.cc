#include "app/simulator/vtd_integration/trigger/send_trigger.h"

namespace smartsil::app::vtd_integration {

SendTrigger::SendTrigger() {
  connectToUdpServer("127.0.0.1", RDB_FEEDBACK_PORT);
}

void* SendTrigger::sendTrigger(const double &simTime, const unsigned int &simFrame) {
  rdbTrigger = new sensor_data::MSG_TRIGGER_t;
  memset(rdbTrigger, 0, sizeof(sensor_data::MSG_TRIGGER_t));

  // fill the rdb trigger message
  // header
  rdbTrigger->hdr.magicNo    = RDB_MAGIC_NO;
  rdbTrigger->hdr.version    = RDB_VERSION;
  rdbTrigger->hdr.headerSize = sizeof(RDB_MSG_HDR_t);
  rdbTrigger->hdr.dataSize   = sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_TRIGGER_t);
  rdbTrigger->hdr.frameNo    = simFrame;
  rdbTrigger->hdr.simTime    = simTime;

  // second the object state
  rdbTrigger->entryTrigger.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  rdbTrigger->entryTrigger.dataSize   = sizeof(RDB_TRIGGER_t);
  rdbTrigger->entryTrigger.elementSize = sizeof(RDB_TRIGGER_t);
  rdbTrigger->entryTrigger.pkgId = RDB_PKG_ID_TRIGGER;
  rdbTrigger->entryTrigger.flags = 0;
  // third the trigger
  rdbTrigger->rdbTrigger.deltaT  = float(simTime);
  rdbTrigger->rdbTrigger.frameNo = simFrame;

  // EndOfFrame
  rdbTrigger->entryEOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  rdbTrigger->entryEOF.dataSize = 0;
  rdbTrigger->entryEOF.elementSize = 0;
  rdbTrigger->entryEOF.pkgId = RDB_PKG_ID_END_OF_FRAME;
  rdbTrigger->entryEOF.flags = 0;

  return static_cast<void*>(rdbTrigger);
}

void SendTrigger::run() {
  vehicle_state::Ego latest_ego;
  recvData(latest_ego);
  if (!latest_ego.active) {
    LOG_ERROR << "ego states is not ready!";
    return;
  } else {
    void *voidPtr = sendTrigger(FLAGS_trigger_period, 1);
    auto rdb_trigger = static_cast<sensor_data::MSG_TRIGGER_t *>(voidPtr);
    sendData(*rdb_trigger);
  }
}

} // namespace smartsil::app::vtd_integration

int main(int argc, char** argv) {
  smartsil::app::vtd_integration::SendTrigger send_trigger;
  while (true) {
    send_trigger.run();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  }
}
