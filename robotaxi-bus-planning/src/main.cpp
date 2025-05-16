//#include "Demo.h"
#include "commom.h"
#include <json/json.h> 
 
//#define MAPENGINE 
#define PLANNING 
//#define PREDICTION 
//#define LOCPERSIM
#if defined(MAPENGINE)
    #include "mapengine_node.h"    //1
#elif defined(PLANNING)
    #include "planning_node.h"   //2
#elif defined(PREDICTION)
    #include "prediction_node.h" //3
#else 
    #include "loc_per_sim_node.h"
#endif

const int moudel_name = 1; 

string node_name;
t_topic_dds t_dds_sub[NUM_MAX];
t_topic_dds t_dds_pub[NUM_MAX];
t_topic_zmq t_zmq_rep[NUM_MAX];
t_topic_zmq t_zmq_req[NUM_MAX];
t_topic_zmq t_zmq_pub[NUM_MAX];
t_topic_zmq t_zmq_sub[NUM_MAX];
t_topic_shm t_shm_sub[NUM_MAX];
t_topic_shm t_shm_pub[NUM_MAX];

int topic_memory_num = 0;
int dds_sub_num = 0;
int dds_pub_num = 0;
int zmq_rep_num = 0;
int zmq_req_num = 0;
int zmq_pub_num = 0;
int zmq_sub_num = 0;
int shm_sub_num = 0;
int shm_pub_num = 0;



int main()
{
    string json_path ="/map/work/config/prediction_node.json"; 
#if defined(MAPENGINE)
    json_path ="/map/work/config/mapengine_node.json";    //1
#elif defined(PLANNING)
    json_path ="/map/work/config/planning_node.json";  //2
#elif defined(PREDICTION)
    json_path ="/map/work/config/prediction_node.json"; //3
#else
    json_path =""; 
#endif
    // if (moudel_name == 1) {
    //    json_path ="/map/work/config/mapengine_node.json";
    // }else if(moudel_name == 2) {
    //     json_path ="/map/work/config/planning_node.json";
    // } else if(moudel_name == 3) {
    //     json_path ="/map/work/config/predicition_node.json";
    // }

    std::ifstream file(json_path,ios::binary); 
         //     
    Json::Reader reader; 
    Json::Value root;
    if(!file.is_open()) {
        std::cout << "json_path: "<<json_path << std::endl;
        std::cerr << "Failed to open file" << std::endl;
        return 1;
    }
    if(!reader.parse(file, root))
    {
        printf("JSon reader file fail \n");
        return  -1;
    }
    node_name = root["node_name"].asString();
    printf("%s\n",node_name.c_str());
     const Json::Value& topic_dds_ = root["topic_dds"];
     int size = topic_dds_.size();
     
     for(int i =0;i<size;i++)
     {
         if(topic_dds_[i]["fuction"]=="pub")
         {
             t_dds_pub[dds_pub_num].type = topic_dds_[i]["type"].asString();
             t_dds_pub[dds_pub_num].topic_name = topic_dds_[i]["topic"].asString();
             t_dds_pub[dds_pub_num].topic_fuction = topic_dds_[i]["fuction"].asString();
             t_dds_pub[dds_pub_num].topic_period = topic_dds_[i]["period"].asInt();
             t_dds_pub[dds_pub_num].topic_netdev = topic_dds_[i]["net_dev"].asString();
             dds_pub_num++;
         }
         else
         {
             t_dds_sub[dds_sub_num].type = topic_dds_[i]["type"].asString();
             t_dds_sub[dds_sub_num].topic_name = topic_dds_[i]["topic"].asString();
             t_dds_sub[dds_sub_num].topic_fuction = topic_dds_[i]["fuction"].asString();
             t_dds_sub[dds_sub_num].topic_period = topic_dds_[i]["period"].asInt();
             t_dds_sub[dds_sub_num].topic_netdev = topic_dds_[i]["net_dev"].asString();
             dds_sub_num++;
         }
         std::cout << topic_dds_[i]["type"] << std::endl;
         std::cout << topic_dds_[i]["topic"] << std::endl;
         std::cout << topic_dds_[i]["fuction"] << std::endl;
         std::cout << topic_dds_[i]["period"] << std::endl;
         std::cout << topic_dds_[i]["net_dev"] << std::endl;
         
     }
    
    const Json::Value& topic_zmq_ = root["topic_zmq"];
    size = topic_zmq_.size();

     for(int i =0;i<size;i++)
     {
         if(topic_zmq_[i]["fuction"]=="Rep")
         {
             t_zmq_rep[zmq_rep_num].type = topic_zmq_[i]["type"].asString();
             t_zmq_rep[zmq_rep_num].topic_name = topic_zmq_[i]["topic"].asString();
             t_zmq_rep[zmq_rep_num].topic_fuction = topic_zmq_[i]["fuction"].asString();
             t_zmq_rep[zmq_rep_num].topic_period = topic_zmq_[i]["period"].asInt();
             t_zmq_rep[zmq_rep_num].topic_target = topic_zmq_[i]["target"].asString();
             t_zmq_rep[zmq_rep_num].topic_port = topic_zmq_[i]["port"].asInt();
             zmq_rep_num++;
         }
         else if(topic_zmq_[i]["fuction"]=="Req")
         {
             t_zmq_req[zmq_req_num].type = topic_zmq_[i]["type"].asString();
             t_zmq_req[zmq_req_num].topic_name = topic_zmq_[i]["topic"].asString();
             t_zmq_req[zmq_req_num].topic_fuction = topic_zmq_[i]["fuction"].asString();
             t_zmq_req[zmq_req_num].topic_period = topic_zmq_[i]["period"].asInt();
             t_zmq_req[zmq_req_num].topic_target = topic_zmq_[i]["target"].asString();
             t_zmq_req[zmq_req_num].topic_port = topic_zmq_[i]["port"].asInt();
             zmq_req_num++;
         }
         else if(topic_zmq_[i]["fuction"]=="Pub")
         {
             t_zmq_pub[zmq_pub_num].type = topic_zmq_[i]["type"].asString();
             t_zmq_pub[zmq_pub_num].topic_name = topic_zmq_[i]["topic"].asString();
             t_zmq_pub[zmq_pub_num].topic_fuction = topic_zmq_[i]["fuction"].asString();
             t_zmq_pub[zmq_pub_num].topic_period = topic_zmq_[i]["period"].asInt();
             t_zmq_pub[zmq_pub_num].topic_target = topic_zmq_[i]["target"].asString();
             t_zmq_pub[zmq_pub_num].topic_port = topic_zmq_[i]["port"].asInt();
             zmq_pub_num++;
         }
         else if(topic_zmq_[i]["fuction"]=="Sub")
         {
             t_zmq_sub[zmq_sub_num].type = topic_zmq_[i]["type"].asString();
             t_zmq_sub[zmq_sub_num].topic_name = topic_zmq_[i]["topic"].asString();
             t_zmq_sub[zmq_sub_num].topic_fuction = topic_zmq_[i]["fuction"].asString();
             t_zmq_sub[zmq_sub_num].topic_period = topic_zmq_[i]["period"].asInt();
             t_zmq_sub[zmq_sub_num].topic_target = topic_zmq_[i]["target"].asString();
             t_zmq_sub[zmq_sub_num].topic_port = topic_zmq_[i]["port"].asInt();
             zmq_sub_num++;
         }
         else
         {

         }
         std::cout << topic_zmq_[i]["type"] << std::endl;
         std::cout << topic_zmq_[i]["topic"] << std::endl;
         std::cout << topic_zmq_[i]["fuction"] << std::endl;
         std::cout << topic_zmq_[i]["period"] << std::endl;
         std::cout << topic_zmq_[i]["target"] << std::endl;
         std::cout << topic_zmq_[i]["port"] << std::endl;
     }

     const Json::Value& topic_shm_ = root["topic_shm"];
     size = topic_shm_.size();
     
     for(int i =0;i<size;i++)
     {
         if(topic_shm_[i]["fuction"]=="pub")
         {
             t_shm_pub[shm_pub_num].type = topic_shm_[i]["type"].asString();
             t_shm_pub[shm_pub_num].topic_name = topic_shm_[i]["topic"].asString();
             t_shm_pub[shm_pub_num].topic_fuction = topic_shm_[i]["fuction"].asString();
             t_shm_pub[shm_pub_num].topic_period = topic_shm_[i]["period"].asInt();
             shm_pub_num++;
         }
         else
         {
             t_shm_sub[shm_sub_num].type = topic_shm_[i]["type"].asString();
             t_shm_sub[shm_sub_num].topic_name = topic_shm_[i]["topic"].asString();
             t_shm_sub[shm_sub_num].topic_fuction = topic_shm_[i]["fuction"].asString();
             t_shm_sub[shm_sub_num].topic_period = topic_shm_[i]["period"].asInt();
             shm_sub_num++;
         }
         std::cout << topic_shm_[i]["type"] << std::endl;
         std::cout << topic_shm_[i]["topic"] << std::endl;
         std::cout << topic_shm_[i]["fuction"] << std::endl;
         std::cout << topic_shm_[i]["period"] << std::endl;
         
     }
#if defined(MAPENGINE)
    acu::mapengine::Mapengine  mapengine_node_;
    mapengine_node_.Init();
    mapengine_node_.Start();     //1
#elif defined(PLANNING)
    acu::planning::Planning planning_node_;
    planning_node_.Init();
    planning_node_.Start();
#elif defined(PREDICTION)
    acu::prediction::Prediction prediction_node_;
    prediction_node_.Init();
    prediction_node_.Start();
#else 
    acu::locpersim::LocPerSim loc_perc_node_;
    loc_perc_node_.Init();
    loc_perc_node_.Start();
#endif


}