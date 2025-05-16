#ifndef _ACU_ZMQ_H_
   #define _ACU_ZMQ_H_
 
#include "zmq.hpp" 
#include "zmq_process_communication.h"

#include "commom.h"
#include <functional>
#include <iostream>
#include <mutex> 
#include <string>

#define REQ_OK     0     
#define REP_NODATA 1
#define REQ_ERROR  2


namespace acu {
namespace zmq {

typedef std::shared_ptr<acu::zmq::BaseClass> BaseClassPtr;

using namespace std;

class Acu_zmq
{
    public:
    Acu_zmq();
	~Acu_zmq();
	int Init();
    void DoRegisterSubscriber(const std::string &topic, int seq_size,std::function<void(CCommonTopicMsg *data, int size)>& call_back);
    void DoRegisterPublisher(const std::string &topic);
    void DoPublish(const std::string &topic,std::shared_ptr<COriginalBytes> input);  

    void DoRegisterServiceServer(const std::string &topic,std::function<void(CCommonTopicMsg *req,CCommonTopicMsg *rep)>& call_back);
    void DoRegisterSeverClient(const std::string &topic);
    int DoService_req_transmit(const std::string &topic,int seq,long stamp,std::shared_ptr<COriginalBytes> input,std::shared_ptr<COriginalBytes> output);
    acu::zmq::CZmqProcCommunication zmq_communication;
    
    private:
    CCommonTopicMsg *m_msg_ptr;
    std::map<std::string, CCommunicationInfo> communication_info_list; //only zmq
    std::map<string, std::pair<int, CCommonTopicMsg *>> write_msg_map;
    int m_read_size;
    std::function<void(CCommonTopicMsg *data, int size)> Callbacksub;
    std::function<void(CCommonTopicMsg *req,CCommonTopicMsg *rep)> Callbackreq;
    std::function<void(CCommonTopicMsg *data)> Callbackrep;
	 
};



}
}



#endif
