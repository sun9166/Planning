#ifndef UDPCASTBASE_H
#define UDPCASTBASE_H

#include "avos_udp/udp_cast_const.h"
#include <string>
#include <vector>
#include <map>

namespace avos
{

class UdpUniCast;
class UdpMultiCast;
class UdpMessage;
class AvosThreadPool;

class UdpCastBase
{
public:
    UdpCastBase();
    virtual ~UdpCastBase();

    int register_callback(const std::string &msg, MessageCallback callback);
    void unregister_callback(const std::string &msg, int index);

protected:
    void ReceiveMessage(const std::string &ip, int port, const char *buffer, long size);

private:
    std::map<std::string, MessageCallback> m_msgcbs;
    std::map<std::string, std::map<int, MessageCallback>> m_callbacks;
    std::map<std::string, int> m_indexs;
    AvosThreadPool *m_threads;
};

}

#endif
