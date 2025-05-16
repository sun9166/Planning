#ifndef __ACU_LOCK_LOG_H__
#define __ACU_LOCK_LOG_H__
#include "mutex"
#include <fstream>
#include <iostream>
#include "log.h"



class LockLog
{
private:
    std::mutex lock_;
    std::ofstream file_stream_;
    std::string file_path_;
    int status;
public:
    LockLog(const std::string& file_path) {

        file_path_ = file_path;
        // if (file_stream_.is_open()) return;
        file_stream_.open(file_path_.c_str());
        status = 1;
    }
    int write(const std::string& log) {

        std::unique_lock<std::mutex> lock(lock_);
        file_stream_ << log << '\n';
        file_stream_.flush();
        return 0;
    }
};

#endif
