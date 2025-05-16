#ifndef ZMQ_IPANDPORT_H
#define ZMQ_IPANDPORT_H


#include <cstdlib>

#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <thread>
#include <functional>
#include <string.h>
#include <sys/types.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "zmq.h"

#include "avos_udp/udp_cast_base.h"
#include "avos_udp/udp_cast_client.h"
#include "avos_udp/udp_cast_const.h"
#include "avos_udp/udp_cast_server.h"
#include "avos_udp/udp_message.h"
#include "./zmq_fileopt.h"
#include "avos_udp/util.h"

using namespace std;
using namespace avos;

namespace acu {
namespace zmq {
typedef std::function<void()> GetZmqIpandPort_type;
class CZmqIpAndProt {
 public:
    void initpara(std::string topic, std::string file_path);
    CZmqIpAndProt(std::string topic, GetZmqIpandPort_type &func, std::string ip, std::string port);

    CZmqIpAndProt(std::string topic, GetZmqIpandPort_type &func, std::string endpoint);
    CZmqIpAndProt(std::string topic, GetZmqIpandPort_type &func);

    std::string getEndPoint() {
        return mEndPoint;
    }

    bool isPortOnline() {
        return isPortOnline_(mStrIp, mS32port);
    }

    bool isDiscover() {
        return mbZmqDiscover;
    }

    int userfunc() {
        mFunc();
        return 0;
    }

 private:
    void send_zmqipport(const std::string &ip, int port, avos::UdpMessage &data);

    void get_zmqipport(const std::string &ip, int port, avos::UdpMessage &data);

    bool getIpandPort(std::string end_point, std::string &ip, int &port);
    bool isPortOnline_(const std::string &ip, int port);

    bool isPortAvailable(const char *ip, int port);

 private:
    std::string mStrTopic;
    std::string mStrIp;
    std::string mStrPort;
    int mS32port;
    std::string mEndPoint;
    bool mbZmqDiscover = false;

    std::shared_ptr<avos::UdpCastServer> mUdpCastServer;
    std::shared_ptr<avos::UdpCastClient> mUdpCastClient;

    GetZmqIpandPort_type mFunc;
    std::thread mReqIpportThread;
    std::string mStrCmd;
};

}
}


#endif