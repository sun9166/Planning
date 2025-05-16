#ifndef CALMCAR_DDS_MESSAGE_H
#define CALMCAR_DDS_MESSAGE_H

#include "binrary_data.h"
#include "binrary_dataPubSubTypes.h"
#include "cache_data.h"
#include "cache_dataPubSubTypes.h"
#include "common.h"
#include "config_data.h"
#include "config_dataPubSubTypes.h"
#include "internal_data.h"
#include "pubsub_api.h"
#include "sync_message.h"

namespace calmcar {
namespace dds {
internal_data& _s_data();

/* ================ Declaration ================ */
struct CALMCAR_DDS_EXPORT SerializedData {
 public:
  // Allocate memory buffer with given size, maybe uninitialized
  explicit SerializedData(uintmax_t size);
  ~SerializedData();
  // Move constructor, retrieve ownership from other
  SerializedData(SerializedData&& other);
  // Move assignment, retrieve ownership from other
  SerializedData& operator=(SerializedData&& other);

  // Size of underlying buffer
  uintmax_t size() const;
  // Ptr of underlying buffer, fill the serialized plain data into it
  void* buffer() const;

  void* get_data() const;

 private:
  struct PrivateData;
  std::unique_ptr<PrivateData> private_ptr;
  // Reserved for internal usage
  explicit SerializedData(PrivateData* d_ptr);
  // Copy is disabled, because the underlying data belongs to framework itself
  SerializedData(const SerializedData&) = delete;
  SerializedData& operator=(const SerializedData&) = delete;
};

/**
 * @brief Initialize the module
 *
 * @param module the module`s name
 * @param config_file the module`s config file name
 * @return
 */
CALMCAR_DDS_EXPORT bool InitModule(const std::string& module,
                                   const std::string& config_file);

/**
 * @brief Release the module
 *
 * @param module the module`s name
 * @return
 */
CALMCAR_DDS_EXPORT void ReleaseModule(const std::string& module);

/**
 * @brief wait for the remote module is initialize
 *
 * @param remote_module the remote module name
 * @param time_out the timeout for waiting, second
 * @return bool true: OK, false: Not init
 */
CALMCAR_DDS_EXPORT bool WaitForRemoteModule(const std::string& remote_module,
                                            const uint64_t& time_out_second);
/**
 * @brief Initialize Publisher for module with data type
 *
 * @param type the data type for this publish
 * @param history_depth the history depth data
 * @param changed_topic if change rclcpp topic name set true, or set false
 * @return
 */
CALMCAR_DDS_EXPORT bool InitPublish(const std::string& type,
                                    const int& history_depth,
                                    const bool changed_topic = true);

/**
 * @brief Initialize Subscriber for module
 *
 * @param type the data type for this subscribe
 * @param history_depth the history depth data
 * @param call_back the func for deal with the sub data
 * @param changed_topic if change rclcpp topic name set true, or set false
 * @return
 */
CALMCAR_DDS_EXPORT bool InitSubscribe(
    const std::string& type, const int& history_depth,
    const std::function<void(void* data, uintmax_t size)>& call_back,
    const bool changed_topic = true);

/**
 * @brief Publish serialized data to key type.
 * @deprecated Slower than SerializedData version, has an extra memcpy.
 * @param type the publish data type
 * @param data data ptr, must be plain data, e.g. serialized
 * @param size data size
 * @return
 */
CALMCAR_DDS_EXPORT bool Publish(const std::string& type, const void* data,
                                uintmax_t size);

/**
 * @brief Publish serialized data to key.\n
 *  The data will be sent directly into backend without any copy.
 * @example
 * ```cpp
 * SerializedData data(size);
 * pbFrame.SerializeToArray(data.buffer(), data.size());
 * Publish("node_name", std::move(data));
 * ```
 * @param type the publish data type
 * @param data Serialized data, must filled with plain data,
 *    use std::move to transfer ownership to framework.
 * @return
 */
CALMCAR_DDS_EXPORT bool Publish(const std::string& type, SerializedData&& data);

/**
 * @brief Set value to config node
 * @example
 * ```cpp
 * // Modify existing node
 * // Before: "general": { "array": [ 1, 2, 3 ] }
 * SetConfig(module, { "general", "array"}, 3.14, 1);
 * // After: "general": { "array": [ 1, 3.14, 3 ] }
 * //                                   ^
 *
 * // Add new node
 * // Before: "general": { "array": [ 1, 2, 3 ] }
 * SetConfig(module, { "general", "name" }, "test");
 * // After: "general": { "array": [ 1, 2, 3 ], "name": "test" }
 * //                                           ^
 * ```
 * @tparam T value type, can be boolean, numeric and string.
 * @param module  Target module name, could be local or remote module,
 *                **MUST** be globally unique.
 * @param keys    Json keys from root to specified value node.\n
 *                  - size_t: Index for json array.\n
 *                  - string: Key for json object.\n
 *                If parent node type not match key, the node will be converted
 *                into new type, all data will be **wiped out**.
 * @param value   Node value to be set.\n
 *                Old value will be **overwritten regardless** of own type.
 * @return true   For value is set successfully.
 * @return false  For module not found or timeout(remote module).
 */
template <typename T>
bool SetConfig(const std::string& module,
               const std::vector<boost::variant<size_t, std::string>>& keys,
               T&& value);

/**
 * @brief Get value from config node
 * @example
 * ```cpp
 * // json : "general": { "array": [ 1, 2, 3 ], "name": "test" }
 * GetConfig<std:string>(module, { "general", "name" });
 * // result : { "test" }
 * GetConfig<int32_t>(module, { "general", "array"}, 1);
 * // result : { 2 }
 * GetConfig<int32_t>(module, { "general", "array" });
 * // result : { 1, 2, 3 }
 * ```
 * @tparam T value type, can be boolean, numeric and string.
 * @param module  Target module name, could be local or remote module,
 *                **MUST** be globally unique.
 * @param keys    Json keys from root to specified value node.\n
 *                  - size_t: Index for json array.\n
 *                  - string: Key for json object.\n.
 * @return std::vector<T> the result values.
 */
template <typename T>
std::vector<T> GetConfig(
    const std::string& module,
    const std::vector<boost::variant<size_t, std::string>>& keys);

/**
 * @brief Get the object from config node
 * @example
 * ```cpp
 * // json : "general": { "array": [ 1, 2, 3 ], "name": "test" }
 * GetConfigObjects<std:string>(module, { "general" });
 * // result : { "array", "name" }
 * ```
 * @tparam T value type, can be boolean, numeric and string.
 * @param module  Target module name, could be local or remote module,
 *                **MUST** be globally unique.
 * @param keys    Json keys from root to specified value node.\n
 *                  - size_t: Index for json array.\n
 *                  - string: Key for json object.\n.
 * @return std::vector<std::string> the result values
 */
std::vector<std::string> GetConfigObjects(
    const std::string& module,
    const std::vector<boost::variant<size_t, std::string>>& keys);

/**
 * @brief Set the Cache object
 *
 * @param module  Target module name, could be local or remote module,
 *                **MUST** be globally unique.
 * @param type    The Cache data type
 * @param value   The data value
 * @param size    The data size
 * @return
 */
CALMCAR_DDS_EXPORT bool SetCache(const std::string& module,
                                 const std::string& type, const void* value,
                                 const uint64_t& size);

/**
 * @brief Get the Cache object
 *
 * @param module  Target module name, could be local or remote module,
 *                **MUST** be globally unique.
 * @param type    The Cache data type
 * @param void*   (out)The Cache data
 * @return uint64_t  The data size, 0 is no data value.
 */
CALMCAR_DDS_EXPORT uint64_t GetCache(const std::string& module,
                                     const std::string& type, void* data,
                                     const uint64_t& cache_size);
/* ================ Declaration ================ */

/* ================ Definition ================ */
namespace impl {
CALMCAR_DDS_EXPORT bool SetConfig(
    const std::string& module,
    const std::vector<boost::variant<size_t, std::string>>& keys,
    const Any& value);

CALMCAR_DDS_EXPORT std::vector<Any> GetConfig(
    const std::string& module,
    const std::vector<boost::variant<size_t, std::string>>& keys);
}  // namespace impl

template <typename T>
bool SetConfig(const std::string& module,
               const std::vector<boost::variant<size_t, std::string>>& keys,
               T&& value) {
  return impl::SetConfig(module, keys, Any(std::forward<T>(value)));
}

template <typename T>
std::vector<T> GetConfig(
    const std::string& module,
    const std::vector<boost::variant<size_t, std::string>>& keys) {
  std::vector<Any> res = impl::GetConfig(module, keys);
  if (!res.empty()) {
    std::vector<T> ret{};
    for (auto& tmp : res) {
      ret.push_back(tmp.value<T>());
    }
    return ret;
  } else {
    return std::vector<T>();
  }
}

}  // namespace dds
}  // namespace calmcar

#endif  // CALMCAR_DDS_MESSAGE_H
