#ifndef UDPMESSAGE_H
#define UDPMESSAGE_H

#include "memory_stream/memory_stream.h"

namespace avos
{

class UdpMessage: public MemoryStream
{
public:
    UdpMessage(const UdpMessage &rhs);
    UdpMessage(const std::string &command);
    // UdpMessage(const std::string &command, long size);
    UdpMessage(const char *buffer, long size);

    const std::string &command() const;
    const std::string &domain() const;
    int identifier() const;

    void reset() override;
    bool invalid();
    bool check(int identifier);

    UdpMessage create_recv(const std::string &command);

private:
    void reset(int identifier);

private:
    bool m_is_read;
    bool m_invalid;
    int m_version;
    int m_identifier;
    std::string m_command;
    std::string m_domain;
};

}

#endif
