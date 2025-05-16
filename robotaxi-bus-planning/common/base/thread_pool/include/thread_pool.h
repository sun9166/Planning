/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/04/01    1.0.0        build
 *****************************************************************************/
#ifndef __THREAD_POOL_H__
#define __THREAD_POOL_H__

#include <setjmp.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "common/base/macros.h"
#include "cpu_select.h"

#define CHECK_CT_VALID(thread_id)                                    \
  auto check_valid_CT = ThreadPool::Instance();                      \
  if (check_valid_CT->IsNeedStopWork(thread_id) == true) {           \
    longjmp(check_valid_CT->thread_content_vec_[thread_id]->env, 1); \
  }

enum class eThreadStatus { IDLE = 0, BUSY, WAITING, STOPPED };

using WorkItem = std::function<int()>;

class ThreadContent {
 public:
  ThreadContent() {
    is_work_alive_.store(true);
    stop_thread_.store(false);
    m_ready.store(false);
  }
  ~ThreadContent() {}

  ThreadContent(const ThreadContent &) = delete;
  ThreadContent &operator=(const ThreadContent &) = delete;

  int AddWork(const WorkItem &work_item, int core_num = -1) {
    if (status_.load() != eThreadStatus::IDLE) return -1;
    status_.store(eThreadStatus::BUSY);
    std::unique_lock<std::mutex> locker(mutex_);
    work_item_ = std::move(work_item);
    m_ready.store(true);
  
    is_work_alive_.store(true);
    cv_.notify_one();
    return 0;
  }

  void Start() {
    status_.store(eThreadStatus::IDLE);
    while (!stop_thread_.load()) {
      std::unique_lock<std::mutex> locker(mutex_);
      cv_.wait(locker, [&]() ->bool { return m_ready.load(); });
      m_ready.store(false);
      int re = -1;
      int jmp_value = setjmp(env);
      if (jmp_value == 0) {
        // std::cout << "compute" << std::endl;
        re = work_item_();
      }
      // std::cout << "compute over" << std::endl;
      work_return_val_.store(re);
      if (work_return_val_ == -255) {
        status_.store(eThreadStatus::IDLE);
      } else {
        status_.store(eThreadStatus::WAITING);
      }
    }
  }

  void InformThreadStop() { is_work_alive_.store(false); }

  void ResetStatus() {
    status_.store(eThreadStatus::IDLE);
    work_return_val_.store(-1);
  }

  void Stop() {
    status_.store(eThreadStatus::STOPPED);
    stop_thread_.store(true);
    cv_.notify_one();
    usleep(1000);
  }

 private:
  WorkItem work_item_;
  std::atomic<bool> stop_thread_;
  std::mutex mutex_;
  std::condition_variable cv_;

 public:
  jmp_buf env;
  std::atomic<bool> m_ready;
  std::atomic<bool> is_work_alive_;
  std::atomic<eThreadStatus> status_;
  std::atomic<int> work_return_val_;
};

class ThreadPool {
 public:
  bool Init(int thread_num) {
    thread_pool_.resize(thread_num);
    thread_content_vec_.resize(thread_num);
    for (int i = 0; i < thread_num; i++) {
      std::cout << "i " << i << std::endl;
      thread_content_vec_[i] = std::make_shared<ThreadContent>();
      const auto com_loop = [&] { thread_content_vec_[i]->Start(); };
      thread_pool_[i] = std::thread(com_loop);

      usleep(100000);
      thread_pool_[i].detach();
      // thread_pool_[i] = std::thread(thread_content_vec_[i].Start());
    }
    sleep(1);
    return true;
  }

  bool Delete() {
    // thread_pool_.clear();
    thread_content_vec_.clear();
    return true;
  }

  int getIdleThreadHandle(int start_index, int end_index) {
    if (start_index >= end_index || start_index >= thread_content_vec_.size()) {
      return getIdleThreadHandle();
    }
    if (thread_content_vec_.size() == 0) return -1;
    for (int i = start_index; i < end_index && i < thread_content_vec_.size();
         i++) {
      if (thread_content_vec_[i]->status_.load() == eThreadStatus::IDLE) {
        return i;
      }
    }
    return -1;
  }

  int getIdleThreadHandle(int index = -1) {
    if (thread_content_vec_.size() == 0) return -1;
    if (index < 0) {
      for (int i = 0; i < thread_content_vec_.size(); i++) {
        if (thread_content_vec_[i]->status_.load() == eThreadStatus::IDLE) {
          return i;
        }
      }
      return -1;
    }
    if (index >= thread_content_vec_.size()) return -1;
    if (thread_content_vec_[index]->status_.load() != eThreadStatus::IDLE)
      return -1;
    return index;
  }

  int ResetThreadStatus(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return -1;
    }
    thread_content_vec_[thread_id]->ResetStatus();
    return 0;
  }
  int AddThreadWork(int thread_id, const WorkItem &work_item,
                    int core_num = -1) {
    if (thread_id >= thread_content_vec_.size()) return -1;
    return thread_content_vec_[thread_id]->AddWork(work_item, core_num);
  }

  int GetThreadCpu(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) return -1;
  }

  eThreadStatus CheckThreadState(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return eThreadStatus::STOPPED;
    }
    return thread_content_vec_[thread_id]->status_.load();
  }

  int GetWorkReturnValue(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return -1;
    }
    return thread_content_vec_[thread_id]->work_return_val_.load();
  }

  int StopWork(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return -1;
    }
    std::cout << "set stopping id " << thread_id << std::endl;
    thread_content_vec_[thread_id]->InformThreadStop();
    if (thread_content_vec_[thread_id]->status_.load() == eThreadStatus::BUSY) {
      return -1;
    }
    if (thread_content_vec_[thread_id]->status_.load() ==
        eThreadStatus::WAITING) {
      thread_content_vec_[thread_id]->status_.store(eThreadStatus::IDLE);
      return 0;
    }
    return -1;
  }

  int StopThread(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return -1;
    }
    StopWork(thread_id);
    thread_content_vec_[thread_id]->Stop();
    if (thread_content_vec_[thread_id]->status_.load() !=
        eThreadStatus::STOPPED) {
      return -1;
    }
    return 0;
  }

  bool IsNeedStopWork(int thread_id) {
    if (thread_id >= thread_content_vec_.size()) {
      return false;
    }

    if (thread_content_vec_[thread_id]->is_work_alive_.load() == false) {
      // std::cout << "need stop " << thread_id << std::endl;
      return true;
    }
    // std::cout << "not need stop" << thread_id << std::endl;
    return false;
  }

 private:
  ThreadPool() {
    thread_pool_.reserve(20);
    thread_content_vec_.reserve(20);
  }

 private:
  std::vector<std::thread> thread_pool_;

 public:
  std::vector<std::shared_ptr<ThreadContent>> thread_content_vec_;

  BASE_DECLARE_SINGLETON(ThreadPool)
};

#endif  // __THREAD_POOL_H__
