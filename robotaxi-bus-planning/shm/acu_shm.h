#ifndef _ACU_SHM_
 #define _ACU_SHM_

 #include "ShmProcCommunication.h"

namespace acu {
namespace shm {

class Acu_shm
{
    public :
     Acu_shm();
     ~Acu_shm();
     int DoRegisterSubscriber(const std::string &topic, int seq_size,std::function<void(CCommonTopicMsg *data)>& call_back);
     int DoRegisterPublisher(const std::string &topic);
     void DoPublish(long time,const std::string &topic,std::shared_ptr<::google::protobuf::MessageLite> msg);

    public :
      ShmProcCommunication Shm_comm;

    private:
     std::map<std::string, unsigned int> topic_seq_map; 
     std::map<string, CCommonTopicMsg *> read_msg_map;
     std::map<string, CCommonTopicMsg *> read_msg_map_all;
     std::map<string, std::pair<int, CCommonTopicMsg *>> write_msg_map;
     std::map<string, int> topic_que_size;

     std::function<void(CCommonTopicMsg *data)> Callbacksub_shm;
};



}
}






#endif
