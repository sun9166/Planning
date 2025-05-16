#ifndef MEMORYSTREAM_H
#define MEMORYSTREAM_H

#include "stream_consts.h"
#include <string>

namespace avos
{
class MemoryStream
{
private:
    MemoryStream(char **buffer, stream::FileIndexType size, stream::FileIndexType length);

public:
    MemoryStream(char **buffer, stream::FileIndexType length);
    MemoryStream(stream::FileIndexType length);
    MemoryStream(const char *buffer, stream::FileIndexType length);
    MemoryStream(const MemoryStream &rhs);
    virtual ~MemoryStream();

    const char *buffer() const;
    stream::FileIndexType length() const;
    stream::FileIndexType size() const;
    stream::FileIndexType pos() const;

    virtual void reset();
    bool seek(stream::FileIndexType pos);

    void write(const void *value, unsigned int size);
    void read(void *value, unsigned int size);
    void write_char(char value);
    char read_char();
    void write_uint(unsigned int value);
    unsigned int read_uint();
    void write_int(int value);
    int read_int();
    void write_ulong(unsigned long value);
    unsigned long read_ulong();
    void write_long(long value);
    long read_long();
    void write_double(double value);
    double read_double();
    void write_string(const std::string &value);
    std::string read_string();

private:
    stream::FileIndexType m_pos;
    stream::FileIndexType m_size;
    stream::FileIndexType m_length;
    char **m_buffer;
    char *m_self_buffer;
    char *m_str;
};

}

#endif
