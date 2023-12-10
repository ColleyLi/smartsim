#include "app/simulator/vtd_integration/bev_detected/frontleft_camera_detected.h"

#include "common/adapters/adapter_gflags.h"
#include "common/logging.h"

namespace smartsil::app::vtd_integration {

int FrontLeftCameraDetected::openPort(int &descriptor, int portNo, const char *serverAddr) {
  struct sockaddr_in server;
  struct hostent *host = NULL;

  //
  // Create the socket, and attempt to connect to the server
  //
  descriptor = socket(AF_INET, SOCK_DGRAM, 0);
  if (descriptor == -1) {
    fprintf( stderr, "openPort: socket() failed: %s\n", strerror(errno));
    return 0;
  }

  int opt = 1;
  if (setsockopt(descriptor, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt))) {
    fprintf( stderr, "setsockopt() failed: %s\n", strerror( errno ) );
  }

  server.sin_family      = AF_INET;
  server.sin_port        = htons(portNo);
  server.sin_addr.s_addr = inet_addr(serverAddr);

  if (server.sin_addr.s_addr == INADDR_NONE) {
    host = gethostbyname(serverAddr);
    if (host == NULL) {
      fprintf(stderr, "openPort: unable to resolve server: %s\n", serverAddr);
      return 0;
    }
    memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
  }

  // wait for connection
  if (bind(descriptor, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf( stderr, "bind failed: %s\n", strerror( errno ) );
  }

  fprintf(stderr, "port % d connected!\n", portNo);

  return 1;
}

void FrontLeftCameraDetected::initConnections() {
  memset( sConnection, 0, MAX_CONNECTIONS * sizeof(Connection_t));

  for ( int i = 0; i < MAX_CONNECTIONS; i++) {
    initConnection(sConnection[i]);
    sConnection[i].id = i;
  }

  sConnection[0].port = DEFAULT_PORT_TC;
  sConnection[1].port = FRONT_LEFT_SENSOR;
}

void FrontLeftCameraDetected::initConnection(Connection_t &conn) {
  strcpy(conn.serverAddr, "127.0.0.1");

  conn.desc = -1;
  conn.bufferSize = sizeof(RDB_MSG_t);
  conn.pData = (unsigned char*) calloc(1, conn.bufferSize);
}

void FrontLeftCameraDetected::readConnection(Connection_t &conn, bool waitForMessage, bool verbose) {
  // receive buffer
  static char* szBuffer = (char*) calloc(1, DEFAULT_BUFFER_SIZE);
  int ret = -1;
  bool bMsgComplete = false;
  if ( verbose )
    fprintf( stderr, "readConnection: start reading connection %d\n", conn.id );

  // read a complete message
  while (!bMsgComplete) {
    ret = recv(conn.desc, szBuffer, DEFAULT_BUFFER_SIZE, 0);

    if ( ret == -1 ) {
      printf( "recv() failed: %s\n", strerror( errno ) );
      break;
    }

    if ( verbose )
      fprintf( stderr, "readConnection: connection %d, ret = %d\n", conn.id, ret );
    if (ret > 0) {
      if ((conn.bytesInBuffer + ret) > conn.bufferSize) {
        conn.pData = (unsigned char*) realloc(conn.pData, conn.bytesInBuffer + ret);
        conn.bufferSize = conn.bytesInBuffer + ret;
      }

      memcpy(conn.pData + conn.bytesInBuffer, szBuffer, ret);
      conn.bytesInBuffer +=ret;

      if (conn.bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
        RDB_MSG_HDR_t* hdr = ( RDB_MSG_HDR_t*) conn.pData;

        if (hdr->magicNo != RDB_MAGIC_NO) {
          printf( "message receiving is out of sync; discarding data" );
          conn.bytesInBuffer = 0;
        }

        while (conn.bytesInBuffer >= (hdr->headerSize + hdr->dataSize)) {
          unsigned int msgSize = hdr->headerSize + hdr->dataSize;

          if (verbose)
            Framework::RDBHandler::printMessage((RDB_MSG_t*)conn.pData, true);

          // now parse the message
          parseRDBMessage(conn, (RDB_MSG_t*)conn.pData);

          // remove message from queue
          memmove(conn.pData, conn.pData + msgSize, conn.bytesInBuffer - msgSize);
          conn.bytesInBuffer -= msgSize;

          bMsgComplete = true;
        }
      }
    }

  };

  if (verbose)
    fprintf( stderr, "readConnection: finished reading connection %d\n", conn.id );
}

void FrontLeftCameraDetected::parseRDBMessage(Connection_t &conn, RDB_MSG_t *msg) {
  if (!msg)
    return;

  if (!msg->hdr.dataSize)
    return;

  RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)(((char*)msg) + msg->hdr.headerSize);
  uint32_t remainingBytes = msg->hdr.dataSize;

  while (remainingBytes) {
    parseRDBMessageEntry(conn, msg->hdr.simTime, msg->hdr.frameNo, entry);

    remainingBytes -= (entry->headerSize + entry->dataSize);

    if (remainingBytes)
      entry = (RDB_MSG_ENTRY_HDR_t*)(((char*)entry) + entry->headerSize + entry->dataSize);
  }
}

void FrontLeftCameraDetected::parseRDBMessageEntry(Connection_t &conn,
                                            const double &simTime,
                                            const unsigned int &simFrame,
                                            RDB_MSG_ENTRY_HDR_t *entryHdr) {
  if (!entryHdr)
    return;

  int noElements = entryHdr->elementSize ? (entryHdr->dataSize / entryHdr->elementSize) : 0;

  if (!noElements) {
    switch ( entryHdr->pkgId )
    {
      case RDB_PKG_ID_START_OF_FRAME:
        fprintf( stderr, "void parseRDBMessageEntry: connection %d: got start of frame\n",  conn.id );
        break;

      case RDB_PKG_ID_END_OF_FRAME:
        fprintf( stderr, "void parseRDBMessageEntry: connection %d: got end of frame\n", conn.id );

        // only connection to sensor shall send perc objects
//        if (conn.id == 1)
//          sendObjects(sendObjPort, simTime, simFrame, 0);
        break;

      default:
        return;
        break;
    }
    return;
  }

  int countObj = 0;
  unsigned char ident = 6;
  char* dataPtr = (char*) entryHdr;

  dataPtr += entryHdr->headerSize;
  while (noElements-- && countObj < 4) {
    bool printedMsg = true;

    RDB_OBJECT_STATE_t* objPtr = (RDB_OBJECT_STATE_t*)dataPtr;

    switch (entryHdr->pkgId)
    {
      case RDB_PKG_ID_OBJECT_STATE:
        if (conn.id == 1 && (entryHdr->flags & RDB_PKG_FLAG_EXTENDED) && (objPtr->base.category == RDB_OBJECT_CATEGORY_PLAYER)) {
          handleRDBitem(simTime, simFrame, *objPtr, entryHdr->flags & RDB_PKG_FLAG_EXTENDED, conn.id == 1u, countObj);
          countObj++;
        }
        break;

      default:
        printedMsg = false;
        break;
    }

    dataPtr += entryHdr->elementSize;
  }
}

void FrontLeftCameraDetected::handleRDBitem(const double &simTime,
                                     const unsigned int &simFrame,
                                     RDB_OBJECT_STATE_t &item,
                                     bool isExtended,
                                     bool isSensor,
                                     int countObj) {
  if (item.base.id == 1 && isExtended)
    memcpy(&mOwnObject, &item, sizeof(RDB_OBJECT_STATE_t));
  else if (isExtended && isSensor)
    memcpy(&mObjState, &item, sizeof(RDB_OBJECT_STATE_t));

  LOG_INFO << "handling 3d objects starting...";

  fprintf( stderr, "handle sensor [%d] object\n",FRONT_LEFT_SENSOR );
  fprintf( stderr, "  simTime = %.3lf, simFrame = %ld\n", simTime, simFrame );
  fprintf( stderr, "  object = %s, id = %d\n", mObjState.base.name, mObjState.base.id );
  fprintf( stderr, "  position = %.3lf / %.3lf / %.3lf\n", mObjState.base.pos.x, mObjState.base.pos.y, mObjState.base.pos.z );

  uint32_t ObjId = mObjState.base.id;
  if (countObj == 0) {
    fprintf( stderr, "handling first object state\n" );
    sendObjects(sendObjPort, simTime, simFrame, countObj);
  } else if (countObj == 1 && ObjId != sensor_data_->SensorFrontLeft[countObj-1].base.id) {
    fprintf( stderr, "handling second object state\n" );
    sendObjects(sendObjPort, simTime, simFrame, countObj);
  } else if (countObj == 2 && ObjId != sensor_data_->SensorFrontLeft[countObj-1].base.id) {
    fprintf( stderr, "handling third object state\n" );
    sendObjects(sendObjPort, simTime, simFrame, countObj);
  } else if (countObj == 3 && ObjId != sensor_data_->SensorFrontLeft[countObj-1].base.id) {
    fprintf( stderr, "handling four object state\n" );
    sendObjects(sendObjPort, simTime, simFrame, countObj);
  }

  LOG_INFO << "handling 3d objects finished...";

}

void FrontLeftCameraDetected::sendObjects(const int &sendPort, const double &simTime, const unsigned int &simFrame, const int &countObj) {
  connectToUdpServer(LOCAL_HOST, sendPort);

  // Header
//  sensor_data_->hdr.magicNo = RDB_MAGIC_NO;
//  sensor_data_->hdr.version = RDB_VERSION;
//  sensor_data_->hdr.headerSize = sizeof(sensor_data::MSG_EGO_STATES_t);
//  sensor_data_->hdr.dataSize = 4 * sizeof(RDB_MSG_ENTRY_HDR_t) + 4 * sizeof(RDB_OBJECT_STATE_t);
//  sensor_data_->hdr.frameNo = simFrame;
//  sensor_data_->hdr.simTime = simTime;
//
//  // StartOfFrame
//  sensor_data_->entrySOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
//  sensor_data_->entrySOF.dataSize = 0;
//  sensor_data_->entrySOF.elementSize = 0;
//  sensor_data_->entrySOF.pkgId = RDB_PKG_ID_START_OF_FRAME;
//  sensor_data_->entrySOF.flags = 0;
//
//  sensor_data_->entryObjectState.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
//  sensor_data_->entryObjectState.dataSize = sizeof(RDB_OBJECT_STATE_t);
//  sensor_data_->entryObjectState.elementSize = sizeof(RDB_OBJECT_STATE_t);
//  sensor_data_->entryObjectState.pkgId = RDB_PKG_ID_OBJECT_STATE;
//  sensor_data_->entryObjectState.flags = RDB_PKG_FLAG_EXTENDED;

  sensor_data_->SensorFrontLeft[countObj].base.id = mObjState.base.id;
  strcpy(sensor_data_->SensorFrontLeft[countObj].base.name, mObjState.base.name);
  sensor_data_->SensorFrontLeft[countObj].base.category = mObjState.base.category;
  sensor_data_->SensorFrontLeft[countObj].base.type = mObjState.base.type;
  sensor_data_->SensorFrontLeft[countObj].base.visMask = mObjState.base.visMask;
  sensor_data_->SensorFrontLeft[countObj].base.geo.dimX = mObjState.base.geo.dimX;
  sensor_data_->SensorFrontLeft[countObj].base.geo.dimY = mObjState.base.geo.dimY;
  sensor_data_->SensorFrontLeft[countObj].base.geo.dimZ = mObjState.base.geo.dimZ;
  sensor_data_->SensorFrontLeft[countObj].base.geo.offX = mObjState.base.geo.offX;
  sensor_data_->SensorFrontLeft[countObj].base.geo.offY = mObjState.base.geo.offY;
  sensor_data_->SensorFrontLeft[countObj].base.geo.offZ = mObjState.base.geo.offZ;
  sensor_data_->SensorFrontLeft[countObj].base.pos.x = mObjState.base.pos.x;
  sensor_data_->SensorFrontLeft[countObj].base.pos.y = mObjState.base.pos.y;
  sensor_data_->SensorFrontLeft[countObj].base.pos.z = mObjState.base.pos.z;
  sensor_data_->SensorFrontLeft[countObj].base.pos.h = mObjState.base.pos.h;
  sensor_data_->SensorFrontLeft[countObj].base.pos.p = mObjState.base.pos.p;
  sensor_data_->SensorFrontLeft[countObj].base.pos.r = mObjState.base.pos.r;
  sensor_data_->SensorFrontLeft[countObj].base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  sensor_data_->SensorFrontLeft[countObj].base.pos.type = RDB_COORD_TYPE_INERTIAL;
  sensor_data_->SensorFrontLeft[countObj].base.pos.system = 0;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.x = mObjState.ext.speed.x;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.y = mObjState.ext.speed.y;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.z = mObjState.ext.speed.z;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.h = mObjState.ext.speed.h;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.p = mObjState.ext.speed.p;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.r = mObjState.ext.speed.r;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.x = mObjState.ext.accel.x;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.y = mObjState.ext.accel.y;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.z = mObjState.ext.accel.z;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.h = mObjState.ext.accel.h;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.p = mObjState.ext.accel.p;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.r = mObjState.ext.accel.r;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.type = RDB_COORD_TYPE_INERTIAL;
  sensor_data_->SensorFrontLeft[countObj].ext.speed.system = 0;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.type = RDB_COORD_TYPE_INERTIAL;
  sensor_data_->SensorFrontLeft[countObj].ext.accel.system = 0;

//  //
//
//  //EndOfFrame
//  sensor_data_->entryEOF.headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
//  sensor_data_->entryEOF.dataSize = 0;
//  sensor_data_->entryEOF.elementSize = 0;
//  sensor_data_->entryEOF.pkgId = RDB_PKG_ID_END_OF_FRAME;
//  sensor_data_->entryEOF.flags = 0;

  sendData(*sensor_data_);
  closeSocket();

}

void FrontLeftCameraDetected::run() {
  LOG_INFO << "Method1";
  static bool sVerbose = false;

  // initialize the connections
  initConnections();

  memset(&mOwnObject, 0, sizeof(RDB_OBJECT_STATE_t));
  memset(&mObjState, 0, sizeof(RDB_OBJECT_STATE_t));

  // open sensor port
  if (!openPort(sConnection[1].desc, sConnection[1].port, sConnection[1].serverAddr))
    return;

  // receive data
//  for (;;)
//  {
//    readConnection(sConnection[1], false, sVerbose);
//  }
  readConnection(sConnection[1], false, sVerbose);
}

} // namespace smartsil::app::vtd_integration
