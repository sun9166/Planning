#ifndef UDPCASTCLIENT_H
#define UDPCASTCLIENT_H

#include "avos_udp/udp_cast_base.h"

namespace avos
{

class UdpCastClient: public UdpCastBase
{
public:
    UdpCastClient(const std::string &ip, int port);
    virtual ~UdpCastClient();

    void send_data(const UdpMessage &data);
    void send_data(const std::string &ip, int port, const UdpMessage &data);

private:
    void whois();
    void iam(MESSAGECALLBACKPARAMS);

private:
    UdpUniCast *m_unicast;
    std::string m_ip;
    int m_port;
};

}

#endif
