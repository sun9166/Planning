#ifndef __COMMUNICATION_BASE_H__
#define __COMMUNICATION_BASE_H__

namespace acu {
namespace common {
class CommunicationBase
{
public:
	CommunicationBase() {
		recive_count_ = 0;
	}
	virtual ~CommunicationBase() {

	}


	virtual int Init() = 0;
	virtual int SpinOnce() = 0;
	virtual int Publish() = 0;

	int recive_count_;
};


}
}//end common


















#endif