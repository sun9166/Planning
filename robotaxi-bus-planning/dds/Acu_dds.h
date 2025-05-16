#ifndef _ACH_DDS_H_
  #define _ACH_DDS_H_

#include <dds/dds.h>
#include <mutex> 
#include <string>



namespace acu {
namespace dds {
using namespace std;

class Acu_dds
{
    public:
    Acu_dds();
		~Acu_dds();
		int Init();
    bool Acu_dds_InitSubscribe(string topicname, std::function<void(void* data, uintmax_t size)>& call_back);
    bool Acu_dds_InitPublish(string topicname);
    bool Acu_dds_Publish(string topicname, calmcar::dds::SerializedData&& data);

    private:
    
	 
};



}
}


#endif