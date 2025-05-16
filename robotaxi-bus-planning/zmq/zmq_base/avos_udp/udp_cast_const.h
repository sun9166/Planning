#ifndef UDPCASTCONST_H
#define UDPCASTCONST_H

#include <string>
#include <functional>

static const std::string FIND_ME = "WHOIS";
static const std::string SAY_HI = "IAM";


namespace avos
{

class UdpMessage;
class UdpCastServer;
class UdpCastClient;

#define MESSAGECALLBACKPARAMS const std::string &ip, int port, avos::UdpMessage &data
#define REGISTER_CALLBACK(NODE, MSG, FUNC) \
NODE->register_callback(MSG, [&](MESSAGECALLBACKPARAMS) noexcept \
{ \
    this->FUNC(ip, port, data); \
})
typedef std::function<void(MESSAGECALLBACKPARAMS)> MessageCallback;

}

#endif
