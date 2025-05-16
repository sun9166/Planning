#ifndef CALMCAR_DDS_JSON_DATA_H
#define CALMCAR_DDS_JSON_DATA_H

#include <boost/variant.hpp>
#include <nlohmann/json.hpp>

#include "any.h"

using namespace nlohmann;

namespace calmcar {
namespace dds {
#define ARRAY_KEYS_STRING "_is_array_item_"

struct JsonData {
 public:
  JsonData() = default;
  ~JsonData() = default;

  size_t depth_ = 0;
  std::map<std::string, std::shared_ptr<JsonData>> all_items_{};
  std::map<std::string, std::shared_ptr<JsonData>> next_item_{};
  std::string key_path_{};
  json value_{};
  bool is_array_ = false;
};

void LoopParse(std::shared_ptr<JsonData>& parent, json& iter, size_t depth);
bool ParseJson(const std::string& config, std::shared_ptr<JsonData>& root_data);
void PrintData(std::shared_ptr<JsonData>& data);
void SaveJsonDataToFile(const std::string& config_file,
                        const std::shared_ptr<JsonData>& data);
std::string FormatArraryItemKey(size_t v);
bool IsArraryKey(const std::string& v);
std::vector<std::string> GetItemKeys(std::shared_ptr<JsonData>& data,
                                     const std::string& key);
}  // namespace dds
}  // namespace calmcar

#endif