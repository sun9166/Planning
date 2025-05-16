#include "Acu_zmq.h"

#include "commom.h"

#include <functional>
#include <thread>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

namespace acu {
namespace zmq {


Acu_zmq::Acu_zmq() : m_read_size(1024)
{
	std::cout << "Acu_zmq" << std::endl;
}
Acu_zmq::~Acu_zmq(){}

int Acu_zmq::Init()
{
    return 0;
}

void Acu_zmq::DoRegisterSubscriber(const std::string &topic, int seq_size,std::function<void(CCommonTopicMsg *data, int size)>& call_back)  
{
    int seq_size_ = seq_size;
    std::cout << "DoRegisterSubscriber:" << topic << std::endl;
    Callbacksub = call_back;
    if(communication_info_list[topic].src_topic=="")
    {
        communication_info_list[topic].src_topic = topic;
    }

    CCommunicationInfo communication_info = communication_info_list[topic];
    int ret = zmq_communication.InitReader(communication_info.src_topic, seq_size);
    std::thread subthread = std::thread ([&]() noexcept {
    while (true) {
            static auto msg_ptr = new CCommonTopicMsg[1];
            //std::cout << "Address of the allocated memory: " << msg_ptr << std::endl;
            int data_get = zmq_communication.GetData(&msg_ptr, m_read_size, 0);
            if (data_get) {
                //call_back(msg_ptr,data_get);
                Callbacksub(msg_ptr,data_get);
            }
            else
            {
            } 
            
            
        }
    });
    subthread.detach();
}

void Acu_zmq::DoRegisterPublisher(const std::string &topic) 
{
    std::cout << "DoRegisterPublisher  :" << topic << std::endl;
    if(communication_info_list[topic].src_topic=="")
    {
        communication_info_list[topic].src_topic = topic;
    }
    CCommunicationInfo communication_info = communication_info_list[topic];
    
    zmq_communication.InitWriter(communication_info.src_topic);
    CCommonTopicMsg *msg_ptr = (CCommonTopicMsg *)malloc(1024);
    write_msg_map[topic] = std::pair<int, CCommonTopicMsg *>(1024, msg_ptr);
}

void Acu_zmq::DoPublish(const std::string &topic,std::shared_ptr<COriginalBytes> input)  
{
	std::string message_type = topic;
    auto it = write_msg_map[topic];
    int max_msg_size = it.first;
    CCommonTopicMsg *write_msg = it.second;
    int msg_size = input->data_size();

    if (msg_size + sizeof(CCommonTopicMsg::STopicMsgHeader) > max_msg_size) {
        free(write_msg);
        max_msg_size = 1.5 * msg_size;
        write_msg = (CCommonTopicMsg *)malloc(max_msg_size + sizeof(CCommonTopicMsg::STopicMsgHeader));
        write_msg_map[topic] = std::pair<int, CCommonTopicMsg *>(max_msg_size, write_msg);
    }

    memset(write_msg->topic_header.message_type, '\0', 256);

    write_msg->topic_header.seq = 0;
    write_msg->topic_header.genstamp = 0;
    memcpy(write_msg->topic_header.message_type, topic.c_str(), message_type.size());

    memcpy(write_msg->message_content, input->data(), input->data_size());

    write_msg->topic_header.message_size = input->data_size();


    if (!zmq_communication.SetData(
            (void *)write_msg, write_msg->topic_header.message_size + sizeof(write_msg->topic_header))) {
        std::cout << "zmq publish failure  " << topic << std::endl;
    }
}

void Acu_zmq::DoRegisterServiceServer(const std::string &topic,std::function<void(CCommonTopicMsg *req,CCommonTopicMsg *rep)>& call_back)  {
    std::cout << "DoRegisterServiceServer  :" << topic << std::endl;
    Callbackreq = call_back;
    if(communication_info_list[topic].src_topic=="")
    {
        communication_info_list[topic].src_topic = topic;
    }
    CCommunicationInfo communication_info = communication_info_list[topic];
    zmq_communication.InitService(communication_info.src_topic);
    std::thread subthread1 = std::thread ([&]() noexcept {
    while (true) {
        static auto msg_ptr = new CCommonTopicMsg[1];
        static auto rep = new CCommonTopicMsg[1];
        int data_get = zmq_communication.GetData(&msg_ptr, m_read_size, 0);
        if (data_get>0) {
                //call_back(msg_ptr,rep);
                Callbackreq(msg_ptr,rep);
                zmq_communication.SetData((void *)rep, rep->topic_header.message_size + sizeof(rep->topic_header));
            }
            else
            {

            }
    }
       
    });
    subthread1.detach();

   }

void Acu_zmq::DoRegisterSeverClient(const std::string &topic)  {
    std::cout << "DoRegisterSeverClient  :" << topic << std::endl;
    if(communication_info_list[topic].src_topic=="")
    {
        communication_info_list[topic].src_topic = topic;
    }
    CCommunicationInfo communication_info = communication_info_list[topic];
    zmq_communication.InitClient(communication_info.src_topic);
}

//void Acu_zmq::DoService_req_transmit(const std::string &topic,int seq,long stamp,std::shared_ptr<COriginalBytes> input)  
int Acu_zmq::DoService_req_transmit(const std::string &topic,int seq,long stamp,std::shared_ptr<COriginalBytes> input,std::shared_ptr<COriginalBytes> output)
{
	std::string message_type = topic;
    auto it = write_msg_map[topic];
    int max_msg_size = it.first;
    CCommonTopicMsg *write_msg = it.second;
    int msg_size = input->data_size();
    if (msg_size + sizeof(CCommonTopicMsg::STopicMsgHeader) > max_msg_size) {
        free(write_msg);
        max_msg_size = 1.5 * msg_size;
        write_msg = (CCommonTopicMsg *)malloc(max_msg_size + sizeof(CCommonTopicMsg::STopicMsgHeader));
        write_msg_map[topic] = std::pair<int, CCommonTopicMsg *>(max_msg_size, write_msg);
    }

    memset(write_msg->topic_header.message_type, '\0', 256);

    write_msg->topic_header.seq = seq;
    write_msg->topic_header.genstamp = stamp;
    memcpy(write_msg->topic_header.message_type, topic.c_str(), message_type.size());

    memcpy(write_msg->message_content, input->data(), input->data_size());
    write_msg->topic_header.message_size = input->data_size();

    if (!zmq_communication.SetData(
            (void *)write_msg, write_msg->topic_header.message_size + sizeof(write_msg->topic_header))) {
        std::cout << "zmq Req failure  " << topic << std::endl;
    }

    int data_get = 0;
    int timer_cout = 0;
    static auto msg_ptr = new CCommonTopicMsg[1];
    data_get = zmq_communication.GetData(&msg_ptr, m_read_size, 0);
    if (data_get) {
        //std::cout << "DoRegisterSeverClient recieved data  :" << topic << ", data length : "<< data_get <<std::endl;
        memcpy(output->data(),msg_ptr->message_content,msg_ptr->topic_header.message_size);
    } 
    else
    {
        if(errno == EAGAIN) 
        {
            printf("rx rep data timeout \n");
            return REP_NODATA;
        }
    }
    
    if(data_get>0)
    {
        return REQ_OK;
    }
    else if(data_get==0)
    {
        return REP_NODATA;
    }
    else
    {
        return REQ_ERROR;
    }
}



}
}


