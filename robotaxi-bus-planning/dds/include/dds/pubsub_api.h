#ifndef CALMCAR_DDS_PUBSUB_API_H
#define CALMCAR_DDS_PUBSUB_API_H

#include "binrary_data.h"
#include "common.h"
#include "config_data.h"

namespace calmcar {
namespace dds {
class PubSubAPI {
 public:
  PubSubAPI();
  ~PubSubAPI();

  static void* InitPublish(const std::string& type, const int& history_depth,
                           const bool changed_topic = false);
  static void* InitSubscribe(
      const std::string& type, const int& history_depth,
      const std::function<void(const std::shared_ptr<BinraryData>& msg)>& func,
      const bool changed_topic = false);

  static void Publish(void* pub, BinraryData& data);

  static void DeletePubData(void*&);
  static void DeleteSubData(void*&);
};  // PubSubAPI
}  // namespace dds
}  // namespace calmcar

#endif  // CALMCAR_DDS_PUBSUB_API_H
