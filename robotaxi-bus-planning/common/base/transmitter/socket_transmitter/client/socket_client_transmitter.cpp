#include "socket_client_transmitter.h"
#include <iostream>

namespace acu {
namespace base {

bool SocketClientTransmitter::InitConnection() {
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
	struct sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(config_.port);
	server_addr.sin_addr.s_addr = INADDR_ANY; //inet_addr(config_.target_ip.c_str());



	// if (fcntl(socket_fd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
	// 	std::cout << BASE_RED << "set non-block error " <<  config_.port << BASE_RESET << std::endl;
	// 	close(socket_fd_);
	// 	return false;
	// }

	if (config_.type == eSocketTransmitterType::UDP) {

		int ret = bind(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
		if (ret < 0)
		{
			perror("bind fail:");
			close(socket_fd_);
			return false;
		}
		printf("recv ready!!!\n");
		return true;
	}



	// connect
	int err = connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
	if (err != 0) {
		std::cout << BASE_RED << config_.target_ip << ":" << config_.port << " connect failed" << BASE_RESET << std::endl;
		return false;
	}
	std::cout << BASE_GREEN << config_.target_ip << ":" << config_.port << " connect success" << BASE_RESET << std::endl;
	return true;
}

int SocketClientTransmitter::RecieveData(unsigned char *data, int max_len) {
	int ret = recv(socket_fd_, data, max_len, 0);
	if (ret == 0) {
		std::cout << BASE_RED << "target " << config_.target_ip << ":" << config_.port << " maybe closed" << BASE_RESET << std::endl;
		return 0;
	}
	// struct sockaddr_in  src_addr = {0};
	// int len = sizeof(src_addr);
	// int ret = recvfrom(socket_fd_, data, max_len, 0, (struct sockaddr *)&src_addr, (socklen_t*)&len);
	return ret;
}


int SocketClientTransmitter::CheckConnection(int time_out) {
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

int SocketClientTransmitter::SendData(const unsigned char *data, int data_len) {
	int ret = send(socket_fd_, data, data_len, 0);
	if (ret < 0) {
		std::cout << BASE_RED << "target " << config_.target_ip << ":" << config_.port << " maybe closed" << BASE_RESET << std::endl;
		return 0;
	}
	return ret;
}


void SocketClientTransmitter::CloseConnection() {
	close(socket_fd_);
}


}
}




