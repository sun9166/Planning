#ifndef ZMQ_FILEOPT_H_
#define ZMQ_FILEOPT_H_
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace std;
namespace acu {
namespace zmq {
class CZmqFileOpt {
 public:
    CZmqFileOpt() {}

    static void appendToFile(const std::string &filepath, const std::string &line) {
        int file = open(filepath.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (file == -1) {
            std::cerr << "Failed to open file." << std::endl;
            return;
        }

        struct flock lock;
        lock.l_type = F_WRLCK;  // 写锁
        lock.l_whence = SEEK_SET;
        lock.l_start = 0;
        lock.l_len = 0;

        if (fcntl(file, F_SETLKW, &lock) == -1) {
            std::cerr << "Failed to acquire lock." << std::endl;
            close(file);
            return;
        }

        std::ofstream outfile(filepath,
                              std::ios_base::app);  // 打开文件进行追加写入
        if (!outfile) {
            std::cerr << "Failed to open file for writing." << std::endl;
            close(file);
            return;
        }

        outfile << line << std::endl;  // 追加写入新行

        lock.l_type = F_UNLCK;  // 解锁
        if (fcntl(file, F_SETLK, &lock) == -1) {
            std::cerr << "Failed to release lock." << std::endl;
        }

        close(file);
    }

    static std::string getEndpointFromFile(const std::string &filepath, const std::string &key) {
        std::string line;
        std::ifstream file(filepath);

        std::string retstr = "";

        if (file.is_open()) {
            while (getline(file, line)) {
                if (line.substr(0, key.length()) == key) {
                    retstr = line.substr(key.length());
                    break;
                }
            }
            file.close();
        }
        return retstr;
    }

    static bool isFileLocked(const std::string &filename) {
        int fd = open(filename.c_str(), O_RDONLY);

        if (fd == -1) {
            // 文件无法打开，可能被其他进程锁定
            return true;
        }

        flock lock;
        lock.l_type = F_RDLCK;  // 设置共享读取锁
        lock.l_whence = SEEK_SET;
        lock.l_start = 0;
        lock.l_len = 0;

        if (fcntl(fd, F_GETLK, &lock) == -1) {
            close(fd);
            return true;
        }

        close(fd);

        // 是否被锁定
        return lock.l_type != F_UNLCK;
    }

    static int removeFile(const std::string &filename) {
        while (CZmqFileOpt::isFileLocked(filename)) {
            sleep(1);
        }

        // 删除文件
        if (std::remove(filename.c_str()) != 0) {
            std::cerr << "Failed to delete file." << std::endl;
            return 1;
        }

        return 0;
    }

 private:
};
}  // namespace zmq
}  // namespace acu
#endif
