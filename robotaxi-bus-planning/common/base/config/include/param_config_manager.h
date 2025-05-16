#ifndef __VEHICLE_CONFIG_MANAGER_H__
#define __VEHICLE_CONFIG_MANAGER_H__

#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <limits.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include "config_schema.pb.h"

class ModelConfig;

using namespace acu::common::config;
typedef std::map<std::string, ModelConfig*> ModelConfigMap;
typedef ModelConfigMap::iterator ModelConfigMapIterator;
typedef ModelConfigMap::const_iterator ModelConfigMapConstIterator;

class ParamConfigManager {
 public:
  void Reset() { inited_ = false; }
  ParamConfigManager();
  ~ParamConfigManager(){};

  int GetConfigPath(std::string& res);
  int GetMapPath(std::string& _map_path);
  int GetCarList(std::vector<std::string>& car_list);
  int GetCurrentCarname(std::string& car_name);
  int SetCurrentCarname(std::string car_name);

  bool Init(const std::string& adapter_config_filename);
  bool ModuleConfigLoad(const std::string& adapter_config_filename);
  bool GetModelConfig(const std::string& top_model_name, ModelConfig** model_config);
  bool SetModelConfig(const std::string& model_name, ModelConfig** model_config);

  bool SaveConfig(const std::string& adapter_config_filename);

 private:
  int LoadGlobalPath();
  bool GetFileContent(const std::string& path, std::string* content);
  bool InitVehicleParam(const std::string& vehicle_config_filename);

 private:
  ModelConfigMap model_config_map_;
  bool inited_;
  std::string map_path_, config_path_, compute_center_type_;
  std::string current_car_name_;
};

class ModelConfig {
 public:
  ModelConfig() {}
  ~ModelConfig() {}

  bool Reset(const acu::common::config::ModelConfigProto& proto);
  bool SaveToProto(acu::common::config::ModelConfigProto& proto);

  std::string name() const { return name_; }

  bool GetValue(const std::string& name, int* value) const {
    return GetValueFromMap<int>(name, integer_param_map_, value);
  }

  bool GetValue(const std::string& name, std::string* value) const {
    return GetValueFromMap<std::string>(name, string_param_map_, value);
  }

  bool GetValue(const std::string& name, double* value) const {
    return GetValueFromMap<double>(name, double_param_map_, value);
  }

  bool GetValue(const std::string& name, float* value) const {
    return GetValueFromMap<float>(name, float_param_map_, value);
  }

  bool GetValue(const std::string& name, bool* value) const {
    return GetValueFromMap<bool>(name, bool_param_map_, value);
  }

  bool GetValue(const std::string& name, std::vector<int>* values) const {
    return GetValueFromMap<std::vector<int>>(name, array_integer_param_map_, values);
  }

  bool GetValue(const std::string& name, std::vector<double>* values) const {
    return GetValueFromMap<std::vector<double>>(name, array_double_param_map_, values);
  }

  bool GetValue(const std::string& name, std::vector<float>* values) const {
    return GetValueFromMap<std::vector<float>>(name, array_float_param_map_, values);
  }

  bool GetValue(const std::string& name, std::vector<std::string>* values) const {
    return GetValueFromMap<std::vector<std::string>>(name, array_string_param_map_, values);
  }

  bool GetValue(const std::string& name, std::vector<bool>* values) const {
    return GetValueFromMap<std::vector<bool>>(name, array_bool_param_map_, values);
  }

  void AddValue(const std::string& name, int value) { integer_param_map_[name] = value; }

  void AddValue(const std::string& name, std::string value) { string_param_map_[name] = value; }

  void AddValue(const std::string& name, double value) { double_param_map_[name] = value; }

  void AddValue(const std::string& name, float value) { float_param_map_[name] = value; }

  void AddValue(const std::string& name, bool value) { bool_param_map_[name] = value; }

  void AddValue(const std::string& name, std::vector<int> values) {
    array_integer_param_map_[name] = values;
  }

  void AddValue(const std::string& name, std::vector<double> values) {
    array_double_param_map_[name] = values;
  }

  void AddValue(const std::string& name, std::vector<float> values) {
    array_float_param_map_[name] = values;
  }

  void AddValue(const std::string& name, std::vector<std::string> values) {
    array_string_param_map_[name] = values;
  }

  void AddValue(const std::string& name, std::vector<bool> values) {
    array_bool_param_map_[name] = values;
  }

  bool ChangeValue(const std::string& name, int value) {
    return ChangeValueFromMap<int>(name, integer_param_map_, value);
  }

  bool ChangeValue(const std::string& name, std::string value) {
    return ChangeValueFromMap<std::string>(name, string_param_map_, value);
  }

  bool ChangeValue(const std::string& name, double value) {
    return ChangeValueFromMap<double>(name, double_param_map_, value);
  }

  bool ChangeValue(const std::string& name, float value) {
    return ChangeValueFromMap<float>(name, float_param_map_, value);
  }

  bool ChangeValue(const std::string& name, bool value) {
    return ChangeValueFromMap<bool>(name, bool_param_map_, value);
  }

  bool ChangeValue(const std::string& name, std::vector<int> values) {
    return ChangeValueFromMap<std::vector<int>>(name, array_integer_param_map_, values);
  }

  bool ChangeValue(const std::string& name, std::vector<double> values) {
    return ChangeValueFromMap<std::vector<double>>(name, array_double_param_map_, values);
  }

  bool ChangeValue(const std::string& name, std::vector<float> values) {
    return ChangeValueFromMap<std::vector<float>>(name, array_float_param_map_, values);
  }

  bool ChangeValue(const std::string& name, std::vector<std::string> values) {
    return ChangeValueFromMap<std::vector<std::string>>(name, array_string_param_map_, values);
  }

  bool ChangeValue(const std::string& name, std::vector<bool> values) {
    return ChangeValueFromMap<std::vector<bool>>(name, array_bool_param_map_, values);
  }

 private:
  template <typename T>
  bool GetValueFromMap(const std::string& name, const std::map<std::string, T>& container,
                       T* value) const;

  template <typename T>
  bool ChangeValueFromMap(const std::string& name, std::map<std::string, T>& container,
                          const T& value);

  template <typename T>
  void RepeatedToVector(const google::protobuf::RepeatedField<T>& repeated_values,
                        std::vector<T>* vec_values);

  std::string name_;
  std::string version_;

  typedef std::map<std::string, int> IntegerParamMap;
  typedef std::map<std::string, std::string> StringParamMap;
  typedef std::map<std::string, double> DoubleParamMap;
  typedef std::map<std::string, float> FloatParamMap;
  typedef std::map<std::string, bool> BoolParamMap;
  typedef std::map<std::string, std::vector<int>> ArrayIntegerParamMap;
  typedef std::map<std::string, std::vector<std::string>> ArrayStringParamMap;
  typedef std::map<std::string, std::vector<double>> ArrayDoubleParamMap;
  typedef std::map<std::string, std::vector<float>> ArrayFloatParamMap;
  typedef std::map<std::string, std::vector<bool>> ArrayBoolParamMap;

  IntegerParamMap integer_param_map_;
  StringParamMap string_param_map_;
  DoubleParamMap double_param_map_;
  FloatParamMap float_param_map_;
  BoolParamMap bool_param_map_;
  ArrayIntegerParamMap array_integer_param_map_;
  ArrayStringParamMap array_string_param_map_;
  ArrayDoubleParamMap array_double_param_map_;
  ArrayFloatParamMap array_float_param_map_;
  ArrayBoolParamMap array_bool_param_map_;
};

template <typename T>
bool ModelConfig::GetValueFromMap(const std::string& name,
                                  const std::map<std::string, T>& container, T* value) const {
  typename std::map<std::string, T>::const_iterator citer = container.find(name);

  if (citer == container.end()) {
    return false;
  }

  *value = citer->second;
  return true;
}

template <typename T>
void ModelConfig::RepeatedToVector(const google::protobuf::RepeatedField<T>& repeated_values,
                                   std::vector<T>* vec_list) {
  vec_list->reserve(repeated_values.size());
  vec_list->clear();
  for (T value : repeated_values) {
    vec_list->push_back(value);
  }
}

template <typename T>
bool ModelConfig::ChangeValueFromMap(const std::string& name, std::map<std::string, T>& container,
                                     const T& value) {
  auto it = container.find(name);
  if (it == container.end()) {
    return false;
  }

  it->second = value;
  return true;
}

// template <typename T>
// void ModelConfig::VectorToRepeated( const std::vector<T> &vec_list,
//   google::protobuf::RepeatedField<T> * repeated_values) {
//   for (T value : vec_list) {
//     repeated_values->add_values(value);
//   }
// }

#endif
