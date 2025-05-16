#ifndef CALMCAR_DDS_INTERNAL_DATA_H
#define CALMCAR_DDS_INTERNAL_DATA_H

#include "common.h"
#include "sync_message.h"
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>

namespace calmcar {
namespace dds {
#define STARTING_MODULE_KEY "starting_module"
#define ENDING_MODULE_KEY "ending_module"

class sub_func {
 public:
  explicit sub_func(const std::function<void(void*, uintmax_t)>& f)
      : func_(f) {}

  sub_func() {}

  sub_func(const sub_func& x) : sub_handle_(x.sub_handle_), func_(x.func_) {}

  sub_func& operator=(const sub_func& x) {
    sub_handle_ = x.sub_handle_;
    func_ = x.func_;
    return *this;
  }

  ~sub_func() {}

  void* sub_handle_{nullptr};
  std::function<void(void*, uintmax_t)> func_{};

  std::function<void(const std::shared_ptr<BinraryData>& msg)> internal_func() {
    return [this](const std::shared_ptr<BinraryData>& msg) {
      this->func_(static_cast<void*>(&msg->data()[0]), msg->size());
    };
  }
};  // sub_func

struct internal_data {
  internal_data(const internal_data&) = delete;
  internal_data& operator=(const internal_data&) = delete;
  internal_data() = default;
  std::mutex mutex{};
  std::thread thread_running{};
  std::thread thread_discovery{};
  std::string local_module{};
  std::string local_config{};
  SyncMessage* local_msg{};

  // Store local memory data
  std::map<std::string, std::vector<uint8_t>> cache_data{};

  // Used for discovery peer modules
  eprosima::fastrtps::Publisher* discovery_pub{};
  eprosima::fastrtps::Subscriber* discovery_sub{};

  // Store the remote sync message for send and recv
  std::map<std::string, SyncMessage*> remote_module{};

  // The remote pub map
  std::map<std::string, void*> data_pub{};
  // The reomte sub call back func map
  std::map<std::string, sub_func*> func{};

  std::atomic_bool exit = ATOMIC_VAR_INIT(false);

  // global param for pub and sub  participant,reduce thread nums.
  eprosima::fastrtps::Participant* participant_{nullptr};
  ConfigDataPubSubType* config_type_{nullptr};
  CacheDataPubSubType* cache_type_{nullptr};

  ~internal_data() {
    if (remote_module.size()) {
      ConfigData init_data;
      init_data.key(ENDING_MODULE_KEY);
      init_data.value(local_module);
      discovery_pub->write(&init_data);
      exit.store(true, std::memory_order_release);
    }
  }
};

inline internal_data& _s_data() {
  static internal_data data{};
  return data;
}

namespace impl {
bool SetLocalConfig(
    const std::vector<boost::variant<size_t, std::string>>& keys,
    const Any& value);

std::vector<Any> GetLocalConfig(
    const std::vector<boost::variant<size_t, std::string>>& keys);

bool SetLocalCache(const std::string& type, const void* value,
                   const uint64_t& size);

uint64_t GetLocalCache(const std::string& type, void* data,
                       const uint64_t& size);

bool GetLocalCache(const std::string& type, CacheData& value);

std::vector<std::string> GetLocalConfigObjects(
    const std::vector<boost::variant<size_t, std::string>>& keys);
}  // namespace impl
}  // namespace dds
}  // namespace calmcar

#endif  // CALMCAR_DDS_INTERNAL_DATA_H
