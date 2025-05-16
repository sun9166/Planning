#include "acu_shm.h"



namespace acu {
namespace shm {

Acu_shm::Acu_shm()
{
    std::cout << "Acu_shm" << std::endl;
}
Acu_shm::~Acu_shm()
{

}

int Acu_shm:: DoRegisterPublisher(const std::string &topic)  
{
        int ret;
        std::cout<<"shm DoRegisterPublisher "<<topic<<std::endl;
        ret = Shm_comm.InitWriter(topic);
        topic_seq_map[topic] = 0;
        CCommonTopicMsg *msg_ptr = (CCommonTopicMsg *)malloc(sizeof(CCommonTopicMsg::STopicMsgHeader));
        write_msg_map[topic] = std::pair<int, CCommonTopicMsg *>(sizeof(CCommonTopicMsg::STopicMsgHeader), msg_ptr);
        return ret;
       
}

void Acu_shm:: DoPublish(long time,const std::string &topic,std::shared_ptr<::google::protobuf::MessageLite> msg)
{
        std::string message_type = topic;
        auto it = write_msg_map[topic];
        int max_msg_size = it.first;
        CCommonTopicMsg *write_msg = it.second;
        
        int msg_size = msg->ByteSize();

        memset(write_msg, '\0', sizeof(CCommonTopicMsg::STopicMsgHeader));
        write_msg->topic_header.seq = topic_seq_map[topic]++;
        write_msg->topic_header.genstamp = time; 

        memcpy(write_msg->topic_header.message_type, message_type.c_str(), message_type.size());
        
        write_msg->topic_header.message_size = msg->ByteSize();

        Shm_comm.SetData(write_msg, msg, write_msg->topic_header.message_size + sizeof(write_msg->topic_header));
        
}

int Acu_shm:: DoRegisterSubscriber(const std::string &topic, int seq_size,std::function<void(CCommonTopicMsg *data)>& call_back)
{
        int ret;
        std::cout<<"shm DoRegisterSubscriber "<<topic<<std::endl;
        Callbacksub_shm = call_back;
        ret = Shm_comm.InitReader(topic); 
        topic_que_size[topic] = seq_size;
        static CCommonTopicMsg *msg_ptr = NULL;
        read_msg_map[topic] = msg_ptr;
        read_msg_map_all[topic] = NULL;
        std::thread subthread = std::thread ([&]() noexcept {
        while (true) {
            //static auto msg_ptr = new CCommonTopicMsg[1];
            bool bGet = Shm_comm.GetData(&msg_ptr);
            if (bGet) {
                Callbacksub_shm(msg_ptr);
            }
            else
            {
            } 
            usleep(10000);
        }
    });
    subthread.detach();
    return ret;
}



}
}
