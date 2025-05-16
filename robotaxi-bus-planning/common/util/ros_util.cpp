#include "common/util/ros_util.h"
#include "common/base/log/include/log.h"
#include <unistd.h>

namespace  acu {
namespace common {
namespace util {
bool ExeLinuxCmdBlock(const std::string cmd) {
	pid_t status = system(cmd.c_str());
	if (-1 == status)
	{
		return false;
	}
	if (WIFEXITED(status))
	{
		if (0 == WEXITSTATUS(status))
		{
			return true;
		}
		AERROR << "ExeLinuxCmdBlock " << cmd << "error returned " << WEXITSTATUS(status);
		return false;
	}
	return true;
}

int IsFileExist(const std::string &file_path) {
	if (0 == access(file_path.c_str(), F_OK))
	{
		return 0;
	}
	return -1;
}


int GetFreeDiskSize(const std::string &file_path) {
	std::vector<std::string> res;
	std::string cmd = "df " + file_path + " | awk '{if (NR>1) {print $4}}'";
	myexec(cmd.c_str(), res);
	if (res.size() != 1) return -1;
	std::stringstream ss;
	ss << res[0];
	int freedisk = 0;
	ss >> freedisk;
	return freedisk;
}

int GetDirectSize(const std::string &file_path) {
	std::vector<std::string> res;
	std::string cmd = "du -k --max-depth=1 " + file_path + " | awk '{print $1}'";

	if (ExeLinuxCmdBlock(cmd) == false) return -1;
	myexec(cmd.c_str(), res);

	int direct_size = 0;
	for (int i = 0; i < res.size(); i++) {
		std::stringstream ss;
		int file_size = 0;
		ss << res[i];
		ss >> file_size;
		direct_size += file_size;
	}
	return direct_size;
}

std::string GetSystemTime() {
	std::vector<std::string> resvec;
	std::string cmd = "date +'%Y%m%d%H%M%S'";
	myexec(cmd.c_str(), resvec);
	if (resvec.size() == 0) return "";
	return resvec[0];
}

void StayDirSize(const std::string &dir_name, int max_size) {
	std::vector<std::string> resvec;
	std::string cmd = "ls -rt " + dir_name;
	myexec(cmd.c_str(), resvec);
	if (resvec.size() == 0) return;
	// for (int i = 0; i < resvec.size(); i++) {
	// 	std::cout << "resvec:" << resvec[i] << std::endl;
	// }
	int dir_size = GetDirectSize(dir_name);
	int index = 0;
	while (dir_size > max_size) {
		cmd = "rm -f " + dir_name + "/" + resvec[index];
		if (++index >= resvec.size()) {
			AWARN << "this dir all delete ? " <<  dir_name;
			break;
		}
		ExeLinuxCmdBlock(cmd);
		dir_size = GetDirectSize(dir_name);
	}
}

//execute shell command
//执行一个shell命令，输出结果逐行存储在resvec中，并返回行数
int myexec(const char *cmd, std::vector<std::string> &resvec) {
	resvec.clear();
	FILE *pp = popen(cmd, "r"); //建立管道
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


bool getPackagePath(std::string package_name, std::string &res) {
	std::vector<std::string> resvec;
	std::string cmd = "find " + package_name;
	myexec(cmd.c_str(), resvec);
	if (resvec.size() == 0)
		return false;
	res = resvec[0];
	return true;
}


bool GetSrcPath(std::string &res) {
	std::string proto_path;
	//if (getPackagePath("proto", proto_path ) == false) {
	//	return false;
	//}
	proto_path = "/map/work/";
	res = proto_path + "";
	return true;
}


bool LookupService(const std::string &node_name, const std::string& service, std::string &uri) {
	
	return true;
}

bool lookupService(const std::string& service, std::string&uri) {
	return true;
}


}  // namespace util
}  // namespace common
}  // namespace  acu
