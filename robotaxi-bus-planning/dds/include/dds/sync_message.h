#ifndef CALMCAR_DDS_SYNC_MESSAGE_H
#define CALMCAR_DDS_SYNC_MESSAGE_H

#include "cache_data.h"
#include "cache_dataPubSubTypes.h"
#include "common.h"
#include "config_data.h"
#include "config_dataPubSubTypes.h"

namespace calmcar {
namespace dds {
#define SYNC_OK "OK"
#define SYNC_FAILED "FAILED"
#define TIME_OUT_SECOND 5

#define SYNC_REQUEST_KEY "SyncRequestKey_"
#define SYNC_RESPONSE_KEY "SyncResponseKey_"

enum method { MESSAGE_SET = 1, MESSAGE_GET, MESSAGE_GET_ARRAY_OBJECTS };

class SyncMessage {
 public:
  explicit SyncMessage(const uint32_t&);
  explicit SyncMessage(const std::string&);
  explicit SyncMessage(const eprosima::fastrtps::Duration_t&);

  SyncMessage() = default;
  SyncMessage(const SyncMessage&);
  SyncMessage& operator=(const SyncMessage&);
  ~SyncMessage() = default;

  const uint32_t& domain_id();
  void domain_id(const uint32_t&);

  void module(const std::string& module);
  const std::string& module();

  bool config_data(const std::string& config_file, const std::string& module);
  std::shared_ptr<JsonData>& config_data(const std::string& module);
  void save_config(const std::string& config_file, const std::shared_ptr<JsonData>& data);

  static eprosima::fastrtps::Participant* CreateParticipant(
      const std::string& name);
  static eprosima::fastrtps::Publisher* CreatePublisher(
      eprosima::fastrtps::Participant* participant, const std::string& topic,
      const std::string& data_type);
  static eprosima::fastrtps::Subscriber* CreateSubscriber(
      eprosima::fastrtps::Participant* participant, const std::string& topic,
      const std::string& data_type);

  bool Init(const std::string& module);

  // ConfigData used for string data
  bool SendTo(SyncMessage* peer_module, ConfigData& data);
  std::vector<Any> RecvFrom(SyncMessage* peer_module);

  // CacheData used for memory data
  bool SendTo(SyncMessage* peer_module, CacheData& data);
  bool RecvFrom(SyncMessage* peer_module, CacheData& data);

  void DealConfigMessage(SyncMessage* peer_module);
  void DealCacheMessage(SyncMessage* peer_module);

  // ConfigData used
  eprosima::fastrtps::Publisher* config_request_publisher{nullptr};
  eprosima::fastrtps::Subscriber* config_request_subscriber{nullptr};
  eprosima::fastrtps::Publisher* config_response_publisher{nullptr};
  eprosima::fastrtps::Subscriber* config_response_subscriber{nullptr};

  // BinraryData used
  eprosima::fastrtps::Publisher* cache_request_publisher{nullptr};
  eprosima::fastrtps::Subscriber* cache_request_subscriber{nullptr};
  eprosima::fastrtps::Publisher* cache_response_publisher{nullptr};
  eprosima::fastrtps::Subscriber* cache_response_subscriber{nullptr};

 private:
  uint32_t domain_id_{1};
  std::string module_{};

  eprosima::fastrtps::Duration_t time_out_{TIME_OUT_SECOND, 0};
  std::string request_key_{SYNC_REQUEST_KEY};
  std::string response_key_{SYNC_RESPONSE_KEY};

  std::map<std::string, std::shared_ptr<JsonData>> config_data_{};
};  // SyncMessage

Any SetValue(const Any::Type& type, const std::string& str);

std::string FormatKeys(
    const std::vector<boost::variant<size_t, std::string>>& keys);

std::string FormatKeys(
    const std::vector<boost::variant<size_t, std::string>>::const_iterator
        key_begin,
    const std::vector<boost::variant<size_t, std::string>>::const_iterator
        key_end);

std::string FormatValue(const Any& value);

std::string FormatValue(const std::vector<Any>& value);

std::string FormatValue(const std::vector<std::string>& value);

std::vector<Any> ParseValues(const std::string& str);

std::vector<boost::variant<size_t, std::string>> ParseString(std::string& str,
                                                             Any& value);
}  // namespace dds
}  // namespace calmcar

#endif  // CALMCAR_DDS_SYNC_MESSAGE_H
