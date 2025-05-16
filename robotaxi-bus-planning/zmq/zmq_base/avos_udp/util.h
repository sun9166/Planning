#ifndef AVOSUTILS_UTIL_H_
#define AVOSUTILS_UTIL_H_

#include <string>
#include <string.h>
#include <vector>
#include <sys/time.h>
#include <unistd.h>
#include <uuid/uuid.h>
#include <bits/stdc++.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <regex>

#include <net/if.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

using std::abs;
namespace avos {
namespace common {

class Util
{
public:
    static int ExeLinuxCmd(const std::string &cmd, std::vector<std::string> &resvec) {
        resvec.clear();
        FILE *pp = popen(cmd.c_str(), "r"); //建立管道
        if (!pp) {
            return -1;
        }
        char tmp[1024]; //设置一个合适的长度，以存储每一行输出
        while (fgets(tmp, sizeof(tmp), pp) != NULL) {
            if (tmp[strlen(tmp) - 1] == '\n') {
                tmp[strlen(tmp) - 1] = '\0'; //去除换行符
            }
            resvec.push_back(tmp);
        }
        pclose(pp); //关闭管道
        return resvec.size();
    }

    static inline std::string generate_uuid()
    {
        uuid_t uuid;
        char str[48];

        uuid_generate(uuid);
        uuid_unparse(uuid, str);
        return str;
    }

    static inline int rand_number(int num)
    {
        srand(time(NULL));
        return rand() % num + 1;
    }

    static inline int rand_number()
    {
        return rand_number(10000);
    }

    static inline std::string rand_string(const std::string &prefix, int num)
    {
        int n = rand_number(num);
        return prefix + "_" + std::to_string(n);
    }

    static inline std::string rand_string(const std::string &prefix)
    {
        return rand_string(prefix, 10000);
    }

    static inline std::string rand_node_name(const std::string &name)
    {
        return rand_string("avos_" + name);
    }

    static long TimeNowMSec()
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (tv.tv_sec * 1000 + tv.tv_usec / 1000.0);
    }

    static std::string GetTimeStr(const std::string &format = "%Y%m%d%H%M%S")
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        tzset();
        struct tm *ptm = localtime(&(tv.tv_sec));
        char time_string[40];
        strftime(time_string, sizeof(time_string), format.c_str(), ptm);
        return std::string(time_string);
    }

    static int replace_all(string &str, const string &pattern, const string &newpat)
    {
        int count = 0;
        const size_t nsize = newpat.size();
        const size_t psize = pattern.size();

        for (size_t pos = str.find(pattern, 0); pos != string::npos; pos = str.find(pattern, pos + nsize))
        {
            str.replace(pos, psize, newpat);
            count++;
        }
        return count;
    }

    static std::vector<std::string> split_string(const std::string &str, char a)
    {
        std::vector<std::string> str_vec;
        std::string::size_type pos1, pos2;
        pos2 = str.find(a);
        pos1 = 0;
        while (std::string::npos != pos2)
        {
            str_vec.push_back(str.substr(pos1, pos2 - pos1));
            pos1 = pos2 + 1;
            pos2 = str.find(a, pos1);
        }
        str_vec.push_back(str.substr(pos1));
        return str_vec;
    }

    static bool file_exists(const std::string &file_name)
    {
        return access(file_name.c_str(), 0) == 0;
    }

    static string addBackslash(const string in_str)
    {
        string out_str = "";
        if (in_str.size() != 0 && in_str.c_str()[0] != '/')
        {
            out_str = "/" + in_str;
        }
        else
        {
            out_str = in_str;
        }
        return out_str;
    }


    static std::string GetEnv(const char *key)
    {
        char *env = getenv(key);
        if(env)
        {
            return env;
        }
        return "";
    }


    static std::string GetEnv(const std::string &key)
    {
        return Util::GetEnv(key.c_str());
    }

    static std::string get_domain()
    {
        return Util::GetEnv("ACU_DOMAIN");  
    }

    static std::string getboardip()
    {
        return Util::GetEnv("BOARD_IP");
    }


    static std::string extractIPAddress(const std::string &input)
    {
        std::regex regexPattern("inet addr:(\\d+\\.\\d+\\.\\d+\\.\\d+)");
        std::smatch match;

        if (std::regex_search(input, match, regexPattern))
        {
            return match.str(1);
        }

        return "";
    }
    static std::string getCurrentIP()
    {
        char buffer[1024];
        std::string ip = getboardip();
        if(ip != "")
        {
            return ip;
        }

        FILE *fp = popen("hostname -I", "r");
        if (fp != nullptr)
        {

            if (fgets(buffer, sizeof(buffer), fp) != nullptr)
            {
                std::cout << "buffer = " << std::string(buffer) << std::endl;
                ip = strtok(buffer, " ");
            }
            else
            {
                std::cout << "cmd hostname -I is error in this system" << std::endl;
            }
            pclose(fp);
        }

        if (ip == "")
        {
            std::string board_ip = getboardip();
            std::cout << "BOARD_IP  address: " << board_ip << std::endl;
            fp = popen("ifconfig | grep \"inet addr\"", "r");
            if (fp != nullptr)
            {

                if (fgets(buffer, sizeof(buffer), fp) != nullptr)
                {
                    std::cout << "buffer = " << std::string(buffer) << std::endl;
                    ip = extractIPAddress(std::string(buffer));
                    if (!ip.empty())
                    {
                        std::cout << "IP address: " << ip << std::endl;
                        if (board_ip != "" && ip != board_ip)
                        {
                            std::cout << "\033[31m注意环境变量BOARD_IP配置的ip与ifconfig命令返回的ip不一致:\033[0m"
                                      << "ip:" << ip << ",board_ip:" << board_ip << std::endl;
                            ip = board_ip;
                        }
                    }
                    else
                    {
                        std::cout << "No IP address found" << std::endl;
                        if (board_ip != "")
                            ip = board_ip;
                    }
                }
                else
                {
                    std::cout << "cmd hostname -I is error in this system" << std::endl;
                }
                pclose(fp);
            }
        }

        return ip;
    }

    static std::string getAvailablePort()
    {
        std::string port = "";

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock != -1)
        {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = 0;

            if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0)
            {
                sockaddr_in boundAddr{};
                socklen_t boundAddrLen = sizeof(boundAddr);
                if (getsockname(sock, reinterpret_cast<sockaddr *>(&boundAddr), &boundAddrLen) == 0)
                {
                    port = std::to_string(ntohs(boundAddr.sin_port));
                }
            }

            close(sock);
        }

        return port;
    }

    static std::string ReplaceEnvironmentVariables(const std::string &str);
};

}
}


#endif
