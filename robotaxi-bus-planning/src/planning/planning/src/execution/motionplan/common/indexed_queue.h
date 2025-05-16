/**
 * @file:
 **/

#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "common/util/map_util.h"

namespace acu {
namespace planning {

template <typename I, typename T>
class IndexedQueue {
 public:
  // Get infinite capacity with 0.
  explicit IndexedQueue(size_t capacity) : capacity_(capacity) {}

  const T *Find(const I id) const {
    auto *result = acu::common::util::FindOrNull(map_, id);
    return result ? result->get() : nullptr;
  }

  const T *Latest() const {
    if (queue_.empty()) {
      return nullptr;
    }
    return Find(queue_.back().first);
  }

  bool Add(const I id, std::unique_ptr<T> ptr) {
    if (Find(id)) {
      return false;
    }

    if (capacity_ > 0 && queue_.size() == capacity_) {
      map_.erase(queue_.front().first);
      queue_.pop();
    }
    queue_.push(std::make_pair(id, ptr.get()));
    map_[id] = std::move(ptr);
    return true;
  }

  void Clear() {
    // capacity_ = 0;
    while (!queue_.empty()) {
      queue_.pop();
    }
    map_.clear();
  }

 public:
  size_t capacity_ = 0;
  std::queue<std::pair<I, const T *>> queue_;
  std::unordered_map<I, std::unique_ptr<T>> map_;
};

}  // namespace planning
}  // namespace acu
