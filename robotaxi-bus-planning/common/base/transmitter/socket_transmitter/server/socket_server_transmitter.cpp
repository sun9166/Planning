#include "socket_server_transmitter.h"
#include <iostream>

namespace acu {
namespace base {

std::string GetIp(int ip) {
	std::string res = "";
	res += (std::to_string(ip & 0xff) + ".");
	res += (std::to_string(ip >> 8 & 0xff) + ".");
	res += (std::to_string(ip >> 16 & 0xff) + ".");
	res += (std::to_string(ip >> 24 & 0xff) );
	return res;
}


bool SocketServerTransmitter::InitConnection() {
	if (config_.port < 0) {
		std::cout << BASE_RED << "config error " << BASE_RESET << std::endl;
		config_.print();
		return false;
	}

	if (config_.type == eSocketTransmitterType::UDP) {
		socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
	} else if (config_.type == eSocketTransmitterType::TCP) {
		socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
	} else {
		std::cout << BASE_RED << "config error" << BASE_RESET << std::endl;
		config_.print();
		return false;
	}
	if (socket_fd_ < 0) {
		std::cout << BASE_RED << "create socket error" << BASE_RESET << std::endl;
		return false;
	}
	int sockflag = 1;
	if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &sockflag, sizeof(int)) < 0) {
		std::cout << BASE_RED << "setsockopt error" << BASE_RESET << std::endl;
	}
	// connect
	memset(&server_addr_, 0, sizeof(server_addr_));
	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(config_.port);
	server_addr_.sin_addr.s_addr = INADDR_ANY;

	if (config_.type == eSocketTransmitterType::UDP) return true;

	int err = bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
	if (err < 0) {
		std::cout << BASE_RED << config_.target_ip << ":" << config_.port << " connect failed" << BASE_RESET << std::endl;
		return false;
	}
	std::cout << BASE_GREEN << "bind port success " << config_.port << BASE_RESET << std::endl;


	// if (fcntl(socket_fd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
	// 	std::cout << BASE_RED << "set non-block error" << ":" << config_.port  << BASE_RESET << std::endl;
	// 	close(socket_fd_);
	// 	return false;
	// }

	std::cout << ", waiting for the connection..." << std::endl;
	err = listen(socket_fd_, 10);
	if (err < 0) {
		std::cout << BASE_RED << config_.target_ip << ":" << config_.port << " listen failed" << BASE_RESET << std::endl;
	}

	struct sockaddr_in client_addr;
	int size = sizeof(client_addr);
	connection_fd_ = accept(socket_fd_, (struct sockaddr*)&client_addr, (socklen_t*)&size);
	if (connection_fd_ < 0) {
		std::cout << BASE_RED << "accept socket error: " << strerror(errno) << " errno: " << errno << BASE_RESET << std::endl;
		return 0;
	}
	std::cout << "get client:ip,port:" << GetIp((int)client_addr.sin_addr.s_addr) << "," << (client_addr.sin_port) << std::endl;
	return true;
}



int SocketServerTransmitter::RecieveData(unsigned char *data, int max_len) {
	int ret = 0;
	if (config_.type == eSocketTransmitterType::UDP) {
		int addr_len = sizeof(struct sockaddr_in);
		ret = recvfrom(socket_fd_, data, max_len, 0,
		               (struct sockaddr *)&server_addr_ , (socklen_t*)&addr_len);
	} else if (config_.type == eSocketTransmitterType::TCP) {
		ret = recv(connection_fd_, data, max_len, 0);
	}
	if (ret < 0) {
		std::cout << BASE_RED << "rev data error " << BASE_RESET << std::endl;
		return -1;
	}
	return ret;

	// int ret = recv(connection_fd_, data, max_len, 0);
	// if (ret == 0) {
	// 	std::cout << BASE_RED << "rev is zerror " << BASE_RESET << std::endl;
	// 	return 0;
	// }
	// return ret;
}
int SocketServerTransmitter::SendData(const unsigned char *data, int data_len) {
	int ret = 0;
	if (config_.type == eSocketTransmitterType::UDP) {
		ret = sendto(socket_fd_, data, data_len, 0, (struct sockaddr *)&server_addr_, sizeof(server_addr_));
	} else if (config_.type == eSocketTransmitterType::TCP) {
		ret = send(connection_fd_, data, data_len, 0);
	}
	if (ret < 0) {
		std::cout << BASE_RED << "send data error " << BASE_RESET << std::endl;
		return -1;
	}
	return ret;

}
void SocketServerTransmitter::CloseConnection() {
	close(socket_fd_);
}


int SocketServerTransmitter::CheckConnection(int time_out) {
	struct pollfd fds[1];
	fds[0].fd = socket_fd_;
	fds[0].events = POLLIN;
	int res = poll(fds, 1, time_out);
	if (res < 0) {
		std::cout << BASE_RED << "poll error : "  << config_.port << " error code is " << res << BASE_RESET << std::endl;
		return -1;
	}

	if (res == 0) {  // poll() timeout?
		std::cout << BASE_YELLOW << "poll is time_out : "  << config_.port << BASE_RESET << std::endl;
		return -2;
	}

	if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
	        (fds[0].revents & POLLNVAL)) {
		std::cout << BASE_RED << "device is error : "  << config_.port  << BASE_RESET << std::endl;
		return -3;
	}
	return 0;
}








}
}


