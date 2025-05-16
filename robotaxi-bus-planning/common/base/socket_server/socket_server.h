#ifndef __SOCKET_SERVER_H__
#define __SOCKET_SERVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <string>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include "common_header/global_define.h"
#include "base/thread_pool/include/thread_pool.h"
#include "base/macros.h"
#include "socket_protocol_header.h"

#define SOCKET_BUFFER_SIZE 4096
#define MAX_CLIENT_NUM 10


namespace acu {
namespace common {



typedef std::function < void (char *recv_data, int recv_len, pchar &send_data, int& send_len)> pDataHandlerFunction;



inline void DefaultDataHandlerFunction(char *recv_data, int recv_len, pchar &send_data, int& send_len) {
	send_len = 0;
	send_data = nullptr;
	std::cout << "DefaultDataHandlerFunction" << std::endl;
}

class SocketServer
{
public:
	SocketServer(int chache_size = SOCKET_BUFFER_SIZE) {
		server_port = -1;
		sockfd = -1;
		new_fd = -1;
		client_counter = 0;
		func = DefaultDataHandlerFunction;
		recv_buffer = new char[chache_size];
		temp_buffer = new char[SOCKET_BUFFER_SIZE];
		chache_size_ = chache_size;
	}
	~SocketServer() {
		delete [] recv_buffer;
		delete [] temp_buffer;
	}

	bool InitServer(const  int& _server_port) {
		server_port = _server_port;
		my_addr.sin_family = AF_INET; //IPV4
		my_addr.sin_port = htons(server_port); //绑定端口9999，并将其转化为网络字节
		my_addr.sin_addr.s_addr = INADDR_ANY; //指定接收IP为任意（如果指定IP，则只接收指定IP）
		bzero(&(my_addr.sin_zero), 0); //最后位置赋0，补位置
//设置socked
		if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
		{
			perror("socket error!\n");
			return false;
		}
		int sockflag = 1;
		if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &sockflag, sizeof(int)) < 0) {
			perror("setsockopt error!\n");
			return false;
		}
//绑定socked
		if ((bind(sockfd, (struct sockaddr*)&my_addr, sizeof(my_addr))) < 0)
		{
			perror("bind error!\n");
			return false;
		}
//开启监听,可连接客户端最大为10个
		if (listen(sockfd, MAX_CLIENT_NUM) == -1)
		{
			perror("listen error!\n");
			return false;
		}

		return true;
	}

	void SetCallbackFunc(pDataHandlerFunction _func) {
		func = _func;
	}

	int WaitConnection() {
		if (server_port <= 0) {
			perror("server is uninit!\n");
			return -1;
		}
		auto thread_pool = ThreadPool::Instance();
		while (1)
		{
			int sin_size = sizeof(struct sockaddr_in);
//等待客户端连接，连接后their_addr接收客户端的IP等信息，没有客户端连接，则一直等待
			if ((new_fd = accept(sockfd,  (struct sockaddr*)(&their_addr), (socklen_t*)&sin_size)) < 0)
			{
				perror("accept error!\n");
				return 0;
			}
//连接成功后，连接客户端数+1
			client_counter++;
//开启进程运行客户端
			int thread_handle = thread_pool->getIdleThreadHandle(1, MAX_CLIENT_NUM);
			const auto work = [&]() {
				ListenData(new_fd, client_counter);
				return -255;
			};
			if (thread_pool->AddThreadWork(thread_handle, work) == -1) {
				std::cout << "thread_handle is not empty:" << thread_handle << std::endl;
			}

		}
		close(sockfd);
		printf("server-------closed.\n");
		return 0;
	}

private:
	void ListenData(int fd, int id)
	{
		int recv_data_index = 0;
		while (1) //一直处于监听客户端信息状态,知道客户端断开或客户端发出退出信息
		{
			memset(temp_buffer, 0, SOCKET_BUFFER_SIZE);
			int len = recv(fd, temp_buffer, SOCKET_BUFFER_SIZE, 0);
			// std::cout << "len:" << len << std::endl;
			if (len > 0) //如果接受到数据，则判断是否为退出信息
			{
				// printf("ID_%d:%d\n", id, len); //输出第N 个用户，输出的信息
				ProtocolHeader *pheader = (ProtocolHeader *)temp_buffer;
				if (pheader->IsHeaderProtocol() == false && recv_data_index == 0) {
					std::cout << "is not header protocol" << std::endl;
					continue;
				}

				memcpy(&recv_buffer[recv_data_index], temp_buffer, len);
				recv_data_index += len;

				ProtocolTail * ptail = (ProtocolTail *)&recv_buffer[recv_data_index - sizeof(ProtocolTail)];
				if (ptail->IsTailProtocol() == false) {
					std::cout << "is not tail protocol" << std::endl;
					continue;
				}
				recv_data_index = 0;

				int send_len = 0;
				char *send_data = nullptr;

				func(&recv_buffer[sizeof(ProtocolHeader)], len - sizeof(ProtocolHeader) - sizeof(ProtocolTail), send_data, send_len);
				if (send_len == 0 || send_data == nullptr) continue;
				ProtocolHeader temp;
				send_buffer = new char[send_len + sizeof(ProtocolHeader) + sizeof(ProtocolTail)];
				memcpy(send_buffer, (char *)&const_header, sizeof(ProtocolHeader));
				memcpy(&send_buffer[sizeof(ProtocolHeader)], send_data, send_len);
				memcpy(&send_buffer[sizeof(ProtocolHeader) + send_len], (char *)&const_tail, sizeof(ProtocolTail));

				send(fd, send_buffer, send_len + sizeof(ProtocolHeader) + sizeof(ProtocolTail), 0);
				delete [] send_buffer;
				send_buffer = nullptr;
			}
			else //接收数据小于0，或等于0 则退出
			{
				printf("clinet %d close!\n", id);
				break;
			}
		}
		close(fd); //关闭此客户端的连接的socket
	}
private:
	int server_port;
	int sockfd, new_fd;
	pDataHandlerFunction func;
	int chache_size_;

	struct sockaddr_in my_addr; //本地连接socked
	struct sockaddr_in their_addr; //客户端连接socked
	char *recv_buffer;
	char *temp_buffer;
	char *send_buffer;
	ProtocolHeader const_header;
	ProtocolTail const_tail;
	int client_counter;  //记录连接客户端数目，可以使用数组，结构体等数据类型记录客户端信息（IP，端口等）
//记录本地信息

};
}
}












#endif
