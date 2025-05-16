#include "Acu_dds.h"
#include "commom.h"

#include <functional>
#include <thread>
#include <memory>
#include <string>

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <mutex>
#include <thread>
#include <string>


#include "Acu_dds.h"

namespace acu {
namespace dds {

Acu_dds::Acu_dds()
{
	std::cout << "Acu_dds" << std::endl;
}
Acu_dds::~Acu_dds(){}


int Acu_dds::Init()
{
    return 0;
}

bool Acu_dds::Acu_dds_InitSubscribe(string topicname, std::function<void(void* data, uintmax_t size)>& call_back)
{
    bool ret = calmcar::dds::InitSubscribe(topicname, 2, call_back);
    if(ret!=false)
    {
        printf("topic : %s Init Subscribe successful ! \n",topicname.c_str());
    }
    return ret;
}

bool Acu_dds::Acu_dds_InitPublish(string topicname)
{
    bool ret = calmcar::dds::InitPublish(topicname, 20);
    if(ret!=false)
    {
        printf("topic : %s Init Publish successful ! \n",topicname.c_str());
    }
    return ret;

}

bool Acu_dds::Acu_dds_Publish(string topicname, calmcar::dds::SerializedData&& data)
{
    bool ret = calmcar::dds::Publish(topicname, std::move(data));   
    if(ret!=false)
    {
        //printf("topic : %s send successful ! \n",topicname.c_str());
    }
    return ret;
}



}
}