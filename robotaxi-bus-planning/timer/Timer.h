#ifndef _TIMER_H_
 #define _TIMER_H_

#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>

namespace acu {
namespace Timer {

class Timer {
public:
    Timer(int interval, std::function<void()> callback)
        : interval_(interval), interval_tmp(interval),callback_(callback), running_(false) {}
 
    void start() {
        running_ = true;
        std::thread thread([this]() {
            auto now = std::chrono::system_clock::now();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
            while (running_) {
                //std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
                now = std::chrono::system_clock::now();
                milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
                t_pint1 = milliseconds.count();
                callback_();
                now = std::chrono::system_clock::now();
                milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
                t_pint2 = milliseconds.count();
                if(t_pint2>t_pint1)
                {
                    if((t_pint2-t_pint1)<(interval_tmp-1))
                    {
                        interval_ = interval_tmp - (t_pint2-t_pint1)-1;
                        usleep(interval_*1000);
                    }
                    else
                    {
                        printf("timrer task CallBack is overload \n");
                    }
                }
                else
                {
                    usleep(interval_tmp*1000);
                }
            }
        });
        thread.detach();
    }
 
    void stop() {
        running_ = false;
    }
 
private:
    int interval_;
    int interval_tmp;
    int t_pint1;
    int t_pint2;
    std::function<void()> callback_;
    std::atomic<bool> running_;
};


}
}
 

#endif