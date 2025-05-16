#ifndef __SOCKET_TRANSMITTER_H__
#define __SOCKET_TRANSMITTER_H__

// linux header


// usr header
#include "transmitter_base.h"
#include "socket_transmitter/socket_header.h"

namespace acu {
namespace base {



class SocketClientTransmitter: public TransmitterBase
{
public:
	SocketClientTransmitter() {
		socket_fd_ = -1;
	}
	~SocketClientTransmitter() {

	}

	void SetConfig(const SocketConfig &config) {
		config_ = config;
	}
	bool InitConnection() override;
	int RecieveData(unsigned char *data, int max_len) override;
	int SendData(const unsigned char *data, int data_len) override;
	void CloseConnection() override;

	int CheckConnection(int time_out) override;

private:
	SocketConfig config_;
	int socket_fd_;

};














}
}

#endif
