/******************************************************************************
* Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: config
* FileName: vehicle_config_manager.h
*
* Description: declare the header and  auto generate for translate proto into struct

*
* History:
* xiayang         2018/04/27    1.0.0    build this module.
******************************************************************************/

#include "common/base/config/include/param_config_manager.h"
#include <fstream>
#include <iostream>
#include <string>

#include "google/protobuf/text_format.h"
using namespace std;

using namespace acu::common::config;
using google::protobuf::TextFormat;

using acu::common::config::ModelConfigFileListProto;
using acu::common::config::ModelConfigProto;
using acu::common::config::MultiModelConfigProto;

ParamConfigManager::ParamConfigManager() {
  inited_ = false;
  map_path_ = "";
  config_path_ = "";
  current_car_name_ = "";
}

int ParamConfigManager::LoadGlobalPath() {
  std::string proto_file;
  /*FILE* pp = popen("rospack find proto", "r");  // 建立管道
  if (!pp) {
    return -1;
  }
  char tmp[1024];  // 设置一个合适的长度，以存储每一行输出
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0';  // 去除换行符
    }
    proto_file = tmp;
  }
  pclose(pp);  // 关闭管道

  Reset();*/      //huangchuo
  //std::string src_path = proto_file + "/../../";   //huangchuo
  std::string src_path = "/map/work/";
  std::string top_config_file = src_path + "config/car_select.conf";
  if (ModuleConfigLoad(top_config_file) == false) {
    std::cout << "cannot load config file " << top_config_file << std::endl;
    return -1;
  }
  ModelConfig* pmodel_config = new ModelConfig();
  GetModelConfig("car_select", &pmodel_config);
  std::string config_path;
  if (pmodel_config->GetValue("config_path", &config_path) == false) {
    std::cout << "cannot get param config_path" << std::endl;
    return -1;
  }
  std::string current_carname;
  if (pmodel_config->GetValue("current_carname", &current_carname) == false) {
    std::cout << "cannot get param current_carname" << std::endl;
    return -1;
  }
  current_car_name_ = current_carname;

  if (pmodel_config->GetValue("compute_center_type", &compute_center_type_) == false) {
    std::cout << "cannot get param compute_center_type_" << std::endl;
  }

  std::string map_path;
  if (pmodel_config->GetValue("map_path", &map_path) == false) {
    std::cout << "cannot get param map_path" << std::endl;
    return -1;
  }

  delete pmodel_config;

  if (config_path == ".") {
    config_path_ = src_path + "config/" + current_carname + "/";

  } else {
    config_path_ = config_path;
  }

  // std::cout << "config path " << config_path_ << std::endl;

  if (map_path == ".") {
    //map_path_ = src_path + "map_file/";     //huangchuo
    map_path_ = "/map/map/map_file/";
  } else {
    map_path_ = map_path;
  }
  // std::cout << "map_path  " << map_path_ << std::endl;

  Reset();
  return 0;
}

int ParamConfigManager::GetCarList(std::vector<std::string>& car_list) {
  car_list.clear();
  std::string proto_file;
  FILE* pp = popen("rospack find proto", "r");  // 建立管道
  if (!pp) {
    return -1;
  }
  char tmp[1024];  // 设置一个合适的长度，以存储每一行输出
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0';  // 去除换行符
    }
    proto_file = tmp;
  }
  pclose(pp);  // 关闭管道
  std::string top_config_file = proto_file + "/../../config/";

  auto dir = opendir(top_config_file.c_str());
  struct dirent* ent;
  if (dir) {
    while ((ent = readdir(dir)) != NULL) {
      if (0 == strcmp(ent->d_name, "..") || 0 == strcmp(ent->d_name, ".")) {
        continue;
      }

      auto path = std::string(top_config_file) + "/" + std::string(ent->d_name) + "/";
      if (opendir(path.c_str()) != NULL) {
        car_list.push_back(ent->d_name);
      }
    }
    closedir(dir);
  }
  return 0;
}

int ParamConfigManager::GetCurrentCarname(std::string& car_name) {
  if (current_car_name_ == "") {
    int res = LoadGlobalPath();
    if (res < 0) return res;
  }
  car_name = current_car_name_;
  return 0;
}
int ParamConfigManager::SetCurrentCarname(std::string car_name) {
  std::string proto_file;
  FILE* pp = popen("rospack find proto", "r");  // 建立管道
  if (!pp) {
    return -1;
  }
  char tmp[1024];  // 设置一个合适的长度，以存储每一行输出
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0';  // 去除换行符
    }
    proto_file = tmp;
  }
  pclose(pp);  // 关闭管道

  Reset();
  std::string src_path = proto_file + "/../../";
  std::string top_config_file = src_path + "config/car_select.conf";
  if (ModuleConfigLoad(top_config_file) == false) {
    std::cout << "cannot load config file " << top_config_file << std::endl;
    return -1;
  }
  ModelConfig* pmodel_config = new ModelConfig();
  GetModelConfig("car_select", &pmodel_config);

  if (pmodel_config->ChangeValue("current_carname", car_name) == false) {
    std::cout << "cannot set param car_name" << std::endl;
    return -1;
  }

  MultiModelConfigProto multi_model_config_proto;
  auto data = multi_model_config_proto.add_model_configs();
  pmodel_config->SaveToProto(*data);

  std::string str;
  google::protobuf::TextFormat::PrintToString(multi_model_config_proto, &str);
  std::cout << str;
  ofstream of_file(top_config_file);
  if (of_file.fail()) {
    cout << "cannot open file " << top_config_file << endl;
    exit(0);
  }
  of_file << str;
  of_file.close();

  delete pmodel_config;
  Reset();
  return 0;
}

int ParamConfigManager::GetConfigPath(std::string& _config_path) {
  if (config_path_ == "") {
    int res = LoadGlobalPath();
    if (res < 0) return res;
  }

  _config_path = config_path_;
  return 0;
}

int ParamConfigManager::GetMapPath(std::string& _map_path) {
  if (map_path_ == "") {
    int res = LoadGlobalPath();
    if (res < 0) return res;
  }

  _map_path = map_path_;
  return 0;
}

bool ParamConfigManager::GetFileContent(const string& path, string* content) {
  if (content == NULL) {
    return false;
  }

  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    cout << "failed to open file: " << path << endl;
    return false;
  }
  struct stat buf;
  if (::fstat(fd, &buf) != 0) {
    cout << "failed to lstat file: " << path << endl;
    ::close(fd);
    return false;
  }

  size_t fsize = buf.st_size;
  content->resize(fsize);
  char* data = const_cast<char*>(content->data());
  int size = 0;
  size_t has_read = 0;
  do {
    size = ::read(fd, data + has_read, fsize - has_read);
    if (size < 0) {
      cout << "failed to read file: " << path << endl;
      ::close(fd);
      return false;
    }
    has_read += size;
  } while (size > 0);
  ::close(fd);
  return true;
}

bool isFileExist(std::string file_name) {
  if ((access(file_name.c_str(), F_OK)) != -1) {
    return true;
  } else {
    return false;
  }
}

bool ParamConfigManager::ModuleConfigLoad(const std::string& adapter_config_filename) {
  // if (inited_) {
  //   std::cout << "ModuleConfigLoad is init...." << std::endl;
  //   return true;
  // }

  if (!isFileExist(adapter_config_filename)) {
    cout << "cannot open config file " << adapter_config_filename << endl;
    exit(1);
  }

  model_config_map_.clear();

  std::string content;
  if (!GetFileContent(adapter_config_filename, &content)) {
    cout << "failed to get ConfigManager config path: " << adapter_config_filename << endl;
    return false;
  }

  MultiModelConfigProto multi_model_config_proto;
  if (!TextFormat::ParseFromString(content, &multi_model_config_proto)) {
    cout << "invalid MultiModelConfigProto file: " << content << endl;
    return false;
  }

  // cout << "multi_model_config_proto.model_configs() : " <<
  // multi_model_config_proto.model_configs_size() << endl;
  for (const ModelConfigProto& model_config_proto : multi_model_config_proto.model_configs()) {
    ModelConfig* model_config = new ModelConfig();
    if (!model_config->Reset(model_config_proto)) {
      return false;
    }
    // cout << "model_config->name(): " << model_config->name() << endl;
    std::pair<ModelConfigMapIterator, bool> result =
        model_config_map_.emplace(model_config->name(), model_config);
    if (!result.second) {
      cout << "duplicate ModelConfig, name: " << model_config->name() << endl;
      return false;
    }
  }
  inited_ = true;
  return true;
}

bool ParamConfigManager::Init(const std::string& adapter_config_filename) {
  if (inited_) {
    return true;
  }

  model_config_map_.clear();

  std::cout << "get_config_file: " << adapter_config_filename << std::endl;
  if (!isFileExist(adapter_config_filename)) {
    cout << "cannot open config file " << adapter_config_filename << endl;
    exit(1);
  }

  std::string content;
  if (!GetFileContent(adapter_config_filename, &content)) {
    cout << "failed to get ConfigManager config path: " << adapter_config_filename << endl;
    return false;
  }

  ModelConfigFileListProto file_list_proto;

  if (!TextFormat::ParseFromString(content, &file_list_proto)) {
    cout << "invalid ModelConfigFileListProto file: " << adapter_config_filename << endl;
    return false;
  }

  for (const std::string& model_config_file : file_list_proto.model_config_path()) {
    std::string config_content;
    if (!GetFileContent(model_config_file, &config_content)) {
      cout << "failed to get_file_content: " << model_config_file << endl;
      return false;
    }

    MultiModelConfigProto multi_model_config_proto;

    if (!TextFormat::ParseFromString(config_content, &multi_model_config_proto)) {
      cout << "invalid MultiModelConfigProto file: " << config_content << endl;
      return false;
    }

    for (const ModelConfigProto& model_config_proto : multi_model_config_proto.model_configs()) {
      ModelConfig* model_config = new ModelConfig();
      if (!model_config->Reset(model_config_proto)) {
        cout << "cannot reset" << endl;
        return false;
      }

      // ROS_INFO_STREAM ( "load ModelConfig succ. name: " << model_config->name());

      std::pair<ModelConfigMapIterator, bool> result =
          model_config_map_.emplace(model_config->name(), model_config);
      if (!result.second) {
        cout << "duplicate ModelConfig, name: " << model_config->name() << endl;
        return false;
      }
    }
  }

  // ROS_INFO_STREAM( "finish to load ModelConfigs. num_models: "
  //                  << model_config_map_.size() );

  inited_ = true;

  return true;
}

bool ParamConfigManager::SaveConfig(const std::string& adapter_config_filename) {
  MultiModelConfigProto multi_model_config_proto;
  for (auto it : model_config_map_) {
    auto data = multi_model_config_proto.add_model_configs();
    it.second->SaveToProto(*data);
  }
  std::string str;
  google::protobuf::TextFormat::PrintToString(multi_model_config_proto, &str);
  // std::cout << str ;
  ofstream of_file(adapter_config_filename);
  if (of_file.fail()) {
    cout << "cannot open file " << adapter_config_filename << endl;
    exit(0);
  }
  of_file << str;
  of_file.close();
  return true;
}

bool ParamConfigManager::SetModelConfig(const std::string& model_name, ModelConfig** model_config) {
  if (!inited_) {
    return false;
  }

  ModelConfigMapConstIterator citer = model_config_map_.find(model_name);

  if (citer == model_config_map_.end()) {
    std::cout << "error........................." << std::endl;
    return false;
  }
  // model_config_map_[model_name]->ChangeValue("lat_period", 1000.0);
  double value;

  model_config_map_[model_name] = *model_config;

  return true;
}

bool ParamConfigManager::GetModelConfig(const std::string& model_name, ModelConfig** model_config) {
  if (!inited_) {
    std::cout << "is init ....." << std::endl;
    return false;
  }
  // ModelConfigMapConstIterator citer = model_config_map_.find(model_name);

  // if (citer == model_config_map_.end()) {
  //   return false;
  // }

  // ModelConfigMap model_config_map_ = citer->second;
  ModelConfigMapConstIterator citer = model_config_map_.find(model_name);

  if (citer == model_config_map_.end()) {
    return false;
  }

  *model_config = citer->second;
  return true;
}

bool ModelConfig::SaveToProto(ModelConfigProto& proto) {
  proto.set_name(name_);
  proto.set_version(version_);
  for (auto it = integer_param_map_.begin(); it != integer_param_map_.end(); ++it) {
    auto data = proto.add_integer_params();
    data->set_name(it->first);
    data->set_value(it->second);
  }

  for (auto it = string_param_map_.begin(); it != string_param_map_.end(); ++it) {
    auto data = proto.add_string_params();
    data->set_name(it->first);
    data->set_value(it->second);
  }

  for (auto it = double_param_map_.begin(); it != double_param_map_.end(); ++it) {
    auto data = proto.add_double_params();
    data->set_name(it->first);
    data->set_value(it->second);
    // std::cout << "name: " << it->first << " | " << "value: " << it->second << std::endl;
  }

  for (auto it = float_param_map_.begin(); it != float_param_map_.end(); ++it) {
    auto data = proto.add_float_params();
    data->set_name(it->first);
    data->set_value(it->second);
  }

  for (auto it = bool_param_map_.begin(); it != bool_param_map_.end(); ++it) {
    auto data = proto.add_bool_params();
    data->set_name(it->first);
    data->set_value(it->second);
  }

  for (auto it = array_integer_param_map_.begin(); it != array_integer_param_map_.end(); ++it) {
    auto data = proto.add_array_integer_params();
    for (auto value : it->second) {
      data->add_values(value);
    }
    data->set_name(it->first);
  }

  for (auto it = array_string_param_map_.begin(); it != array_string_param_map_.end(); ++it) {
    auto data = proto.add_array_string_params();
    for (auto value : it->second) {
      data->add_values(value);
    }
    data->set_name(it->first);
  }

  for (auto it = array_double_param_map_.begin(); it != array_double_param_map_.end(); ++it) {
    auto data = proto.add_array_double_params();
    for (auto value : it->second) {
      data->add_values(value);
    }
    data->set_name(it->first);
  }

  for (auto it = array_float_param_map_.begin(); it != array_float_param_map_.end(); ++it) {
    auto data = proto.add_array_float_params();
    for (auto value : it->second) {
      data->add_values(value);
    }
    data->set_name(it->first);
  }

  for (auto it = array_bool_param_map_.begin(); it != array_bool_param_map_.end(); ++it) {
    auto data = proto.add_array_bool_params();
    for (auto value : it->second) {
      data->add_values(value);
    }
    data->set_name(it->first);
  }
}

bool ModelConfig::Reset(const ModelConfigProto& proto) {
  name_ = proto.name();
  version_ = proto.version();

  integer_param_map_.clear();
  string_param_map_.clear();
  double_param_map_.clear();
  float_param_map_.clear();
  bool_param_map_.clear();
  array_integer_param_map_.clear();
  array_string_param_map_.clear();
  array_double_param_map_.clear();
  array_float_param_map_.clear();
  array_bool_param_map_.clear();

  for (const KeyValueInt& pair : proto.integer_params()) {
    integer_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueString& pair : proto.string_params()) {
    string_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueDouble& pair : proto.double_params()) {
    double_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueFloat& pair : proto.float_params()) {
    float_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueBool& pair : proto.bool_params()) {
    bool_param_map_.emplace(pair.name(), pair.value());
  }

  for (const KeyValueArrayInt& pair : proto.array_integer_params()) {
    std::vector<int> values;
    RepeatedToVector(pair.values(), &values);
    array_integer_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayString& pair : proto.array_string_params()) {
    std::vector<std::string> values;
    values.reserve(pair.values_size());
    for (const std::string& value : pair.values()) {
      values.push_back(value);
    }
    array_string_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayDouble& pair : proto.array_double_params()) {
    std::vector<double> values;
    RepeatedToVector(pair.values(), &values);
    array_double_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayFloat& pair : proto.array_float_params()) {
    std::vector<float> values;
    RepeatedToVector(pair.values(), &values);
    array_float_param_map_.emplace(pair.name(), values);
  }

  for (const KeyValueArrayBool& pair : proto.array_bool_params()) {
    std::vector<bool> values;
    RepeatedToVector(pair.values(), &values);
    array_bool_param_map_.emplace(pair.name(), values);
  }

  // cout << "reset ModelConfig. model_name: " << name_
  //      << " integer_param_map's size: " << integer_param_map_.size()
  //      << " string_param_map's size: " << string_param_map_.size()
  //      << " double_param_map's size: " << double_param_map_.size()
  //      << " float_param_map's size: " << float_param_map_.size()
  //      << " bool_param_map's size: " << bool_param_map_.size()
  //      << " array_integer_param_map's size: "
  //      << array_integer_param_map_.size()
  //      << " array_string_param_map's size: " << array_string_param_map_.size()
  //      << " array_double_param_map's size: " << array_double_param_map_.size()
  //      << " array_float_param_map's size: " << array_float_param_map_.size()
  //      << " array_bool_param_map's size: " << array_bool_param_map_.size() << endl;

  return true;
}
