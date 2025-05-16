#ifndef __ACU_COMMON_UTIL_H__
#define __Acu_COMMON_UTIL_H__

#include <string>
#include <string.h>
#include <vector>


namespace acu {
namespace common {

class Util
{
public:
	Util() {

	}
	~Util() {

	}


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
};


}
}


#endif
