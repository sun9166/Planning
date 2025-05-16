#ifndef UDPCASTSERVER_H
#define UDPCASTSERVER_H

#include "avos_udp/udp_cast_base.h"

namespace avos
{

class UdpCastServer: public UdpCastBase
{
public:
    UdpCastServer(const std::string &ip, int port);
    virtual ~UdpCastServer();

    void send_data(const std::string &ip, int port, const UdpMessage &data);

private:
    void findme(MESSAGECALLBACKPARAMS);

private:
    UdpUniCast *m_unicast;
    UdpMultiCast *m_multicast;
};

}

#endif
