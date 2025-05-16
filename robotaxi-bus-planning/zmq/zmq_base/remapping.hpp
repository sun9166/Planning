#ifndef __REMAPPING_HPP__
#define __REMAPPING_HPP__
#include "avos_udp/util.h"
#include <json/json.h> 
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory.h>
#include <memory>
#include <string>
#include <vector>

using namespace std;

namespace acu {
namespace zmq {
class CTopicRemap {
 public:
    CTopicRemap() {
        init_topic_name_remap();
    }

    string getRemapTopic(const std::string &topic) {
        string topic_name;
        if (intopic_name_remap.count(topic) == 0) {
            topic_name = topic;
        } else {
            topic_name = intopic_name_remap[topic];
        }
        return topic_name;
    }

    int DeserializationFromFile(const std::string &file_path) {
        std::ifstream ifs(file_path);
        if (ifs.is_open() == false) {
            std::cout << "NodeConfig DeserializationToFile "
                         "open file failed"
                      << std::endl;
            return -1;
        }
        Json::Reader reader;
        Json::Value root;
        if (!reader.parse(ifs, root, false)) {
            std::cout << "NodeConfig DeserializationToFile "
                         "parse file failed"
                      << std::endl;
            ifs.close();
            return -1;
        }
        auto node_ip = root["node_ip"];
        for (int i = 0; i < node_ip.size(); i++) {
            m_node_ip[node_ip[i]["node"].asString()] = node_ip[i]["ip"].asString();
        }
        auto topic_remaps = root["topic_remaps"];
        for (int i = 0; i < topic_remaps.size(); i++) {
            std::string ip = topic_remaps[i]["to"].asString();
            for (auto it : m_node_ip) {
                avos::common::Util::replace_all(ip, it.first, it.second);
            }
            intopic_name_remap[avos::common::Util::addBackslash(topic_remaps[i]["from"].asString())] = ip;
        }
        ifs.close();
        return 0;
    }
    void init_topic_name_remap() {
        
        string json_file_path = "/work/config";
        json_file_path+= "/in_topic_rename.json";
        DeserializationFromFile(json_file_path);
    }

 private:
    std::map<std::string, std::string> intopic_name_remap;
    std::map<std::string, std::string> m_node_ip;
};
}  // namespace zmq
}  // namespace acu
#endif