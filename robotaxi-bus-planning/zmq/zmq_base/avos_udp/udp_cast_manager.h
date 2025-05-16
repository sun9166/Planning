#ifndef UDPCASTMANAGER_H
#define UDPCASTMANAGER_H

#include "avos_udp/udp_cast_server.h"
#include "avos_udp/udp_cast_client.h"

namespace avos
{

class UdpCastManager
{
public:
    static UdpCastServer *GetCastServer(const std::string &ip, int port);
    static UdpCastClient *GetCastClient(const std::string &ip, int port);
};

}

#endif
