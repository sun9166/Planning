#ifndef __ZMQ_PROCESS_H__
#define __ZMQ_PROCESS_H__

#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "remapping.hpp"
#include "zmq.h"
#include "zmq_ipandport.h"



namespace acu {
namespace zmq {

class BaseClass
{
public:
    BaseClass() {}
    virtual ~BaseClass() {}
};


union t_CommunicationDataType
{
    struct SAvosSubOpt
    {
        unsigned int seq_size_ : 10;
        unsigned int callback_mode_ : 3;
        unsigned int msg_mode_ : 4;
        unsigned int zmq_ipport_fromfile : 1;
        unsigned int recover_data_ : 14;
    } sub_opt;
    int seq_size = 0;
};

class CCommonTopicMsg {
 public:
    CCommonTopicMsg() {}
    ~CCommonTopicMsg() {}
    struct STopicMsgHeader {
        int seq;
        long genstamp;
        char message_type[256];
        int message_size;
    } topic_header;

    char message_content[0];  //不能在结构体中使用指针变量,
                              // char
                              // message_content[0]这种写法
};

class CCommunicationInfo
{
public:
    CCommunicationInfo() {}

public:
    std::string src_topic;
    std::string dst_topic;
};



class CZmqProcCommunication {
 public:
    CZmqProcCommunication();
    ~CZmqProcCommunication();

 public:
    void GetInfo();

    bool isPortOnline();
    std::shared_ptr<CZmqIpAndProt> m_ip_port;
    int initIpandPort(const std::string &_topic, bool create, GetZmqIpandPort_type &func);

    GetZmqIpandPort_type lambda;
    int InitReader(const std::string &_topic, int _buffer_len = 10);

    int InitService(const std::string &_topic);

    int InitClient(const std::string &_topic);

    int InitWriter(const std::string &_topic);
    std::vector<uint8_t> gdata;
    int GetData(CCommonTopicMsg **msg_data, int &all_size, int flag);

    bool GetServiceData(CCommonTopicMsg **msg_data, int &all_size, int flag = ZMQ_DONTWAIT);

    bool GetNewestData(CCommonTopicMsg **msg_data, int &all_size);
    bool SetData(const void *data, int msg_len, int flag = 0);

    bool req_connected = false;
    bool rep_connected = false;

 private:
    zmq_msg_t *pack_frame_msg_bak;
    void *context;
    void *socket = NULL;
    void *socket_monitor = NULL;
    CTopicRemap topic_remap;
    char addr[1024];
    std::thread m_client_thread;
    bool bgetipportbyfile = false;

 private:
};

}
}


#endif
