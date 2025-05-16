#ifndef __SOCKET_SERVER_TRANSMITTER_H__
#define __SOCKET_SERVER_TRANSMITTER_H__




// linux header
#include <sys/types.h>
#include <sys/socket.h>
#include <string>

// usr header
#include "transmitter_base.h"
#include "socket_transmitter/socket_header.h"

namespace acu {
namespace base {



class SocketServerTransmitter: public TransmitterBase
{
public:
	SocketServerTransmitter() {
		socket_fd_ = -1;
	}
	~SocketServerTransmitter() {

	}

	void SetConfig(const SocketConfig &config) {
		config_ = config;
	}
	bool InitConnection() override;
	int RecieveData(unsigned char *data, int max_len) override;
	int SendData(const unsigned char *data, int data_len) override;
	void CloseConnection() override;

	int CheckConnection(int time_out) override;
	void WaitForConnection();

private:
	SocketConfig config_;
	int socket_fd_;
	int connection_fd_;
	struct sockaddr_in server_addr_;

};













}
}









#endif

