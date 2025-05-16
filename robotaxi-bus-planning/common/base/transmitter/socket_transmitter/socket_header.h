#ifndef __SOCKET_HEADER_H__
#define __SOCKET_HEADER_H__
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
namespace acu {
namespace base {

#define BASE_RESET   "\033[0m"
#define BASE_BLACK   "\033[30m"      /* Black */
#define BASE_RED     "\033[31m"      /* Red */
#define BASE_GREEN   "\033[32m"      /* Green */
#define BASE_YELLOW  "\033[33m"      /* Yellow */
#define BASE_BLUE    "\033[34m"      /* Blue */
#define BASE_MAGENTA "\033[35m"      /* Magenta */
#define BASE_CYAN    "\033[36m"      /* Cyan */
#define BASE_WHITE   "\033[37m"      /* White */
#define BASE_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BASE_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BASE_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BASE_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BASE_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BASE_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BASE_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BASE_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
enum class eSocketTransmitterType {UDP = 0, TCP};

typedef struct SocketConfig
{
	eSocketTransmitterType type;
	std::string target_ip;
	int port;
	SocketConfig() {
		type = eSocketTransmitterType::UDP;
		target_ip = "";
		port = -1;
	}
	void print() {
		std::cout << "socket config:" << std::endl;
		std::cout << "ip:" << target_ip << std::endl;
		std::cout << "port:" << port << std::endl;
	}
} SocketConfig;







}
}



#endif
