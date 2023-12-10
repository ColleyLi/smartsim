#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <csignal>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "common/logging.h"

struct UdpConnection_server {
  int sock_fd;
  struct sockaddr_in clnt_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(clnt_addr));
  uint32_t FRAME_SIZE = 1024;
};

namespace smartsil {

template <typename D>
class UdpSocketServer {
 public:
  typedef D DataType;

  /**
   * @brief UDP socket initialized
   * @param port remote port
   */
  bool connectToUdpClient(const int& port) {
    udp_server_.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_server_.sock_fd < 0) {
      LOG_ERROR << "create socket failed!\n";
      return -1;
    }

    //中断函数
    signal(SIGINT, signalHandler);

    memset(&udp_server_.clnt_addr, 0, sizeof(udp_server_.clnt_addr));
    udp_server_.clnt_addr.sin_family = AF_INET;
    udp_server_.clnt_addr.sin_port = htons(port);
    udp_server_.clnt_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int opt = 1;
    setsockopt ( udp_server_.sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int ret = bind(udp_server_.sock_fd,(struct sockaddr *)&udp_server_.clnt_addr, udp_server_.sock_len);
    if (ret == -1) {
      LOG_ERROR << "bind socket failed";
      close(udp_server_.sock_fd);
      return false;
    }

    LOG_INFO << "connected to client success. port [" << port << "]";

    return true;

  }


  /**
   * @brief receive udp data
   * @param data The data of udp server received
   */
  const D& recvData(const DataType& data) {
    int nbytes = 0;
    int total_recv = 2 * udp_server_.FRAME_SIZE;
    char recvBuf[2048] = {0};
    memset(recvBuf, 0, sizeof(char) * total_recv);

    nbytes = recvfrom(udp_server_.sock_fd, recvBuf, sizeof(recvBuf), 0,
                      (sockaddr *)&udp_server_.clnt_addr, &udp_server_.sock_len);
    if (nbytes == -1 || nbytes > total_recv) {
      LOG_ERROR << "failed to receive data.";
    }
    LOG_INFO << "recv " << nbytes << " bytes from client.";

    memcpy((char*)(&data), recvBuf, sizeof(data));

    return data;
  }

  void closeSocket() {close(udp_server_.sock_fd);}

 private:
  static void signalHandler(int signum) {
    if (signum == SIGINT) {
      LOG_INFO << "Ctrl+C pressed. Closing socket and exiting.";
      static UdpSocketServer<D>* instance;
      close(instance->udp_server_.sock_fd);
    }
  }

 private:
  UdpConnection_server udp_server_;
};


} // namespace smartsil
