#ifndef __ACU_COMMON_LOCK_QUEUE_H__
#define __ACU_COMMON_LOCK_QUEUE_H__

#include <mutex>
#include <atomic>

#include <condition_variable>
#include <functional>
#include <queue>

template <typename T>
class LockQueue
{
public:
  inline LockQueue()
  {
  }

  inline T front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.front();
  }

  inline T back()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.back();
  }



  inline void push(const T& value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
  }

  inline void pop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty())
    {
      queue_.pop();
    }
  }

  inline T popFront()
  {
    T value;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!queue_.empty())
    {
      value = std::move(queue_.front());
      queue_.pop();
    }
    return value;
  }

  inline void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lock(mutex_);
    swap(empty, queue_);
  }

  inline size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

public:
  std::queue<T> queue_;

private:
  mutable std::mutex mutex_;
};


#endif
