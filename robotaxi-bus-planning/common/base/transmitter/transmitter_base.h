#ifndef __COMMON_BASE_TRANSMITTER_BASE_H__
#define __COMMON_BASE_TRANSMITTER_BASE_H__


#include  <memory.h>
namespace acu {
namespace base {



class TransmitterBase
{
public:
	TransmitterBase() {

	}
	virtual ~TransmitterBase() {

	}
	virtual bool InitConnection() = 0;
	virtual bool ReConnection() {
		return true;
	};

	virtual int CheckConnection(int time_out) {
		return 0;
	}
	virtual int RecieveData(unsigned char *data, int max_len) = 0;
	virtual int SendData(const unsigned char *data, int data_len) = 0;
	virtual void CloseConnection() = 0;

};



}
}


#endif


