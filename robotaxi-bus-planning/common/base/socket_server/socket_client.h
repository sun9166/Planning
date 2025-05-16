#ifndef __ACU_SOCKET_CLIENT_H__
#define __ACU_SOCKET_CLIENT_H__
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <sys/file.h>
#include <fcntl.h>
#include <string>
#include "common_header/global_define.h"
#include "socket_protocol_header.h"

#define BUFFER_SIZE 4096
namespace acu {
namespace common {

class SocketClient
{
public:
	SocketClient(int chache_size = BUFFER_SIZE) {
		send_buffer = new char[chache_size];
		buffer_size = chache_size;
	}
	~SocketClient() {
		delete [] send_buffer;
	}



	int InitSocket(const std::string &server_ip, const int &port) {
		server_ip_ = server_ip;
		port_ = port;
		socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
		if (socket_fd_ < 0) {
			std::cout << "get socket error" << std::endl;
			return -1;
		}
		int sockflag = 1;
		if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &sockflag, sizeof(int)) < 0) {
			std::cout << BASE_RED << "setsockopt error" << BASE_RESET << std::endl;
		}

		struct sockaddr_in server_addr;
		memset(&server_addr, 0, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(port_);
		server_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());

		// connect
		int err = connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
		if (err != 0) {
			std::cout << BASE_RED << server_ip_ << ":" << port_ << " connect failed" << BASE_RESET << std::endl;
			return -1;
		}
		std::cout << BASE_GREEN << server_ip_ << ":" << port_ << " connect success" << BASE_RESET << std::endl;
		return 0;
	}


	int SendData(const char *data, int data_len) {
		memcpy(send_buffer, (char *)&const_header, sizeof(ProtocolHeader));
		memcpy(&send_buffer[sizeof(ProtocolHeader)], data, data_len);
		memcpy(&send_buffer[sizeof(ProtocolHeader) + data_len], (char *)&const_tail, sizeof(ProtocolTail));


		int ret = send(socket_fd_, send_buffer, data_len + sizeof(ProtocolHeader) + sizeof(ProtocolTail), 0);
		if (ret < 0) {
			std::cout << BASE_RED << "target " << server_ip_ << ":" << port_ << " maybe closed" << BASE_RESET << std::endl;
			return 0;
		}
		return ret - sizeof(ProtocolHeader) - sizeof(ProtocolTail);
	}

	int RecieveData(char * recv_data, int max_len) {
		int len = max_len > buffer_size ? buffer_size : max_len;
		int ret = recv(socket_fd_, send_buffer, len, 0);
		if (ret == 0) {
			std::cout << BASE_RED << "target " << server_ip_ << ":" << port_ << " maybe closed" << BASE_RESET << std::endl;
			Close();
			return -2;
		}
		if (ret <= sizeof(ProtocolHeader) + sizeof(ProtocolTail)) {
			std::cout << "connot recieve enough data  " << std::endl;
			return -1;
		}

		ProtocolHeader *pheader = (ProtocolHeader *)send_buffer;
		if (pheader->IsHeaderProtocol() == false  ) {
			std::cout << "connot meet header protocol" << std::endl;
			return -1;
		}

		ProtocolTail *ptail = (ProtocolTail *)&send_buffer[ret - sizeof(ProtocolTail)];
		if (ptail->IsTailProtocol() == false) {
			std::cout << "connot meet tail protocol" << std::endl;
			return -1;
		}
		memcpy(recv_data, &send_buffer[sizeof(ProtocolHeader)], ret - sizeof(ProtocolHeader) - sizeof(ProtocolTail));
		return ret - sizeof(ProtocolHeader) - sizeof(ProtocolTail);
	}

	void Close() {
		close(socket_fd_);
	}

private:
	int buffer_size;
	ProtocolHeader const_header;
	ProtocolTail const_tail;
	char * send_buffer;
	int socket_fd_;
	std::string server_ip_;
	int port_;

};

}
}


#endif
