#ifndef _SHMPROCCOMMUNICATION_H_
 #define _SHMPROCCOMMUNICATION_H_

 #include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <google/protobuf/message_lite.h>


 #include "shared_memory.h"
 #include "sempore.h"

 #define POINT_HEADER_CHECK_CODE 0x20221223

 union UAcuCommunication64bitDataType
{
    struct SAvosPubOpt
    {
        unsigned long long seq_size_ : 10;
        unsigned long long msg_max_size_ : 32;
        unsigned long long msg_mode_ : 7;
        unsigned long long islend : 1;
        unsigned long long recover_data_ : 14;
    } pub_opt;
    unsigned long long opt = 0;
};

enum EMsgMode
{
    eMsgDefaultMode,
    eMsgProtoMode,
    eMsgByteProtoMode,
    eMsgOriginalBytesMode
};
 
namespace acu {
namespace shm {

using namespace acu::common;
using namespace std;

class BaseClass
{
public:
    BaseClass() {}
    virtual ~BaseClass() {}
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

typedef std::shared_ptr<acu::shm::BaseClass> BaseClassPtr;

class ShmProcCommunication  {
 public:
    ShmProcCommunication();
    ~ShmProcCommunication();
    int InitReader(const std::string &_topic);
    int InitWriter(const std::string &_topic);
    bool InitDataBuffer(CCommonTopicMsg **data);
    bool GetData(CCommonTopicMsg **data);
    bool SetData(const CCommonTopicMsg *data, std::shared_ptr<::google::protobuf::MessageLite> msg, int msg_len);
   

 private:
    ShmAreaHeader *GetIndexHeader(const int &index);
    int GetNextReadIndex(AreaLocation &loc, pShmAreaHeader &next_p);
    int GetNextWriteIndex(AreaLocation &loc, pShmAreaHeader &area);
    void FillInnerLoc(AreaLocation &lowest, AreaLocation &highest);
    int ExeLinuxCmd(const std::string &cmd, std::vector<std::string> &resvec);
    void ClearSemaphore();

 public:
    int msg_data_size = 0;
 private:
    bool m_running = true;
    ShmHeader *header_buffer_pointer = nullptr;
    char *data_buffer_pointer = nullptr;
    std::string topic, data_topic, msg_data_topic;
    int shmid_header_;
    int shmid_data_;
    int pub_seq;
    int data_seq;

    AreaLocation read_loc;

    int buffer_len;
    bool is_data_empty;

    vector<int> msg_data_size_list;

    int whitch_one_sub = -1;

    int all_sub_num = 0;
    int read_sempore_id = -1;
    vector<int> werite_sempore_id_list;
    bool bfirstsetdata = true;
    unsigned long long pub_opt = 0;

};

}
}


#endif