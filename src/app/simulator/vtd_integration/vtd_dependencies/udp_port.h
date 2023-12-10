#pragma once

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#define DEFAULT_PORT_TC     48190   /* for image port it should be 48192 */
#define DEFAULT_PORT_SENSOR 48199
#define FRONT_WIDE_SENSOR   48180
#define FRONT_RIGHT_SENSOR  48181
#define FRONT_LEFT_SENSOR   48182
#define REAR_SENSOR         48183
#define REAR_RIGHT_SENSOR   48184
#define REAR_LEFT_SENSOR    48185
#define DEFAULT_BUFFER_SIZE 204800
#define MAX_CONNECTIONS     2       /* maximum number of bi-directional UDP connections */
#define LOCAL_HOST          "127.0.0.1"

constexpr int sendObjPort   = 9000;
constexpr int sendObjPort1   = 9001;
constexpr int sendLightPort = 9001;

// type definition for connection handling
typedef struct
{
  int           id;               // unique connection ID
  char          serverAddr[128];  // Server to connect to
  int           port;             // Port on server to connect to
  int           desc;             // client (socket) descriptor
  unsigned int  bytesInBuffer;    // used size of receive buffer
  size_t        bufferSize;       // total size of receive buffer;
  unsigned char *pData;           // pointer to receive buffer
} Connection_t;
