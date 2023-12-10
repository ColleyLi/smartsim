#pragma once

#include <mutex>
#include <string>
#include <thread>
#include <csignal>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

struct UdpConnection_client {
  int sock_fd;
  struct sockaddr_in serv_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(serv_addr));
};

namespace smartsil {

template <typename D>
class UdpSocketClient {
 public:
  typedef D DataType;

  /**
   * @brief UDP socket initialized
   * @param ip_addr remote ip address
   * @param port remote port
   */
  bool connectToUdpServer(const std::string& ip_addr, const int& port) {
    LOG_INFO << "connecting to server... ";
    udp_client_.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_client_.sock_fd < 0) {
      LOG_ERROR << "create socket failed!";
      return false;
    }
    memset(&udp_client_.serv_addr, 0, sizeof(udp_client_.serv_addr));
    udp_client_.serv_addr.sin_family = AF_INET;
    udp_client_.serv_addr.sin_port = htons(static_cast<uint16_t>(port));
    udp_client_.serv_addr.sin_addr.s_addr = inet_addr(ip_addr.c_str());

    if (ip_addr.empty() || port == 0) {
      LOG_ERROR << "remote info is invalid.";
      return false;
    }

    int res =
        connect(udp_client_.sock_fd, (struct sockaddr*)&udp_client_.serv_addr,udp_client_.sock_len);

    if (res < 0) {
      LOG_ERROR << "connect socket failed!";
      close(udp_client_.sock_fd);
    }

    LOG_INFO << "connected to server success. port [" << port << "]";

    return true;
  }

  /**
   * @brief send udp data
   * @param data data of udp client send to server
   */
  const D& sendData(const DataType& data) {
    ssize_t nbytes = sendto(udp_client_.sock_fd, &data, sizeof(data), 0,
                            (struct sockaddr*)&udp_client_.serv_addr, udp_client_.sock_len);
    if (nbytes == -1) {
      std::cerr << "sent msg failed." << std::endl;
      std::cerr << "Failed to send data. Error code: " << errno << std::endl;
      close(udp_client_.sock_fd);
    }
    LOG_INFO << "sent " << nbytes << " bytes to server.";

    return data;
  }

  void closeSocket() {close(udp_client_.sock_fd);}

 private:
  static void signalHandler(int signum) {
    if (signum == SIGINT) {
      LOG_INFO << "Ctrl+C pressed. Closing socket and exiting.";
      static UdpSocketClient<D>* instance;
      close(instance->udp_client_.sock_fd);
    }
  }

 private:
  UdpConnection_client udp_client_;
};


} // namespace
