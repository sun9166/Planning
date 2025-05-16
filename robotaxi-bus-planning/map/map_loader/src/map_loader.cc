#include "common/util/file.h"
#include "common/util/string_util.h"
#include "common/util/string_tokenizer.h"
// #include "config_schema.pb.h"
#include "map/map_loader/include/map_loader.h"
#include "tinyxml2.h"

namespace acu {
namespace map {

std::shared_ptr<LinkMap> MapLoader::link_map_ = nullptr;
std::mutex MapLoader::link_map_mutex_;
std::shared_ptr<BaseMap> MapLoader::base_map_ = nullptr;
std::mutex MapLoader::base_map_mutex_;
std::shared_ptr<Mapinfo> MapLoader::mapinfo_ = nullptr;
std::mutex MapLoader::mapinfo_mutex_;
std::shared_ptr<VectorMap> MapLoader::vector_map_ = nullptr;
std::mutex MapLoader::vector_map_mutex_;
std::shared_ptr<AppMap> MapLoader::app_map_ = nullptr;
std::mutex MapLoader::app_map_mutex_;

bool isFileExist(const std::string file_name) {
  if (file_name == "")
    return false;
  if (access(file_name.c_str(), F_OK) == 0)
    return true;
  return false;
}

std::shared_ptr<VectorMap> MapLoader::CreateVectorMap() {
  Mapinfo* mapinfo = GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();
  std::string xml_file;

  if (!mpheader.VectormapEnabled) {
    return nullptr;
  }

  std::shared_ptr<VectorMap> vectormap(new VectorMap());

  xml_file = mapinfo->GetMapPath() + mpheader.vector_map_dir + 
            mpheader.vector_map_file + "_hdmap.bin";
  AINFO << "SSSSSSS: " << xml_file;
  if (vectormap->LoadMapFromFile(xml_file) != 0) {
    std::cout<<"Failed to load VectorMap " << xml_file<<std::endl;
    return nullptr;
  }

  xml_file = mapinfo->GetMapPath() + mpheader.vector_map_dir + 
            mpheader.vector_map_file + "_roadnet.bin";
  AINFO << "SSSSSSS: " << xml_file;
  if (vectormap->LoadRoadnetFromFile(xml_file) != 0) {
    std::cout<<"Failed to load roadnet " << xml_file<<std::endl;
    return nullptr;
  }
  if (vectormap->LoadCarSelectConf() != 0) {
    std::cout<<"Failed to load car_select.conf "<<std::endl;
    return nullptr;
  }
  printf("Load VectorMap and roadnet success: ");
  return vectormap;
}

VectorMap* MapLoader::GetVectorMapPtr() {
  if (vector_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(vector_map_mutex_);
    if (vector_map_ == nullptr) {  // Double check.
      vector_map_ = CreateVectorMap();
      if (vector_map_ == nullptr) {
        return nullptr;
      }
    }
  }
  return vector_map_.get();
}

HDMapImpl *MapLoader::GetHDMapImplPtr() {
  if (vector_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(vector_map_mutex_);
    if (vector_map_ == nullptr) {  // Double check.
      vector_map_ = CreateVectorMap();
      if (vector_map_ == nullptr) {
        return nullptr;
      }
    }
  }
  return vector_map_->GetHDMapImplPtr();
}

std::shared_ptr<LinkMap> MapLoader::CreateLinkMap() {
  if (link_map_ != nullptr) {
    return link_map_;
  }

  std::lock_guard<std::mutex> lock(link_map_mutex_);

  if (link_map_ != nullptr) { // Double check.
    return link_map_;
  }

  Mapinfo* mapinfo = GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();

  std::string xml_file = mapinfo->GetMapPath() + mpheader.link_map_dir + 
      mpheader.link_map_file;

  std::shared_ptr<LinkMap> linkmap(new LinkMap());

  if (!linkmap->LoadMapFromFile(xml_file, mapinfo->GetMapPath() + mpheader.wxb_map_dir)) {
    std::cout<<"Failed to load linkMap " << xml_file<<std::endl;
    return nullptr;
  }

  link_map_ = linkmap;

  return link_map_;
}

acu::map::LinkMap* MapLoader::GetLinkmapPtr() {
  Mapinfo* mapinfo = GetMapinfoPtr();
  if (mapinfo == nullptr) {
    return nullptr;
  }

  auto mpheader = mapinfo->GetMapParamHeader();

  link_map_ = CreateLinkMap();
  if (link_map_ == nullptr) {
    return nullptr;
  }

  return link_map_.get();
}

BaseMap *MapLoader::GetBasemapPtr() {
  Mapinfo* mapinfo = GetMapinfoPtr();
  if (mapinfo == nullptr) {
    return nullptr;
  }

  if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      auto mpheader = mapinfo->GetMapParamHeader();
      std::string basemap_dir = mapinfo->GetMapPath() + mpheader.basemap_dir;
      base_map_ = std::shared_ptr<BaseMap>(new BaseMap(basemap_dir, mpheader));
      if (base_map_->CreatSharedMemory() < 0) base_map_ = nullptr;
    }
  }
  return base_map_.get();
}

std::shared_ptr<AppMap> MapLoader::CreateAppMap() {
  if (app_map_ != nullptr) {
    return app_map_;
  }

  std::lock_guard<std::mutex> lock(app_map_mutex_);

  if (app_map_ != nullptr) { // Double check.
    return app_map_;
  }

  Mapinfo* mapinfo = GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();

  std::string xml_file = mapinfo->GetMapPath() + mpheader.app_map_dir + 
      mpheader.app_map_file;

  std::shared_ptr<AppMap> appmap(new AppMap());

  if (!appmap->LoadMapFromFile(xml_file)) {
    // ROS_ERROR_STREAM("Failed to load app map " << xml_file);
    return nullptr;
  }

  app_map_ = appmap;

  return app_map_;
}

acu::map::AppMap *MapLoader::GetAppmapPtr() {
  Mapinfo* mapinfo = GetMapinfoPtr();
  if (mapinfo == nullptr) {
    return nullptr;
  }

  auto mpheader = mapinfo->GetMapParamHeader();

  app_map_ = CreateAppMap();
  if (app_map_ == nullptr) {
    return nullptr;
  }

  return app_map_.get();
}

Mapinfo* MapLoader::GetMapinfoPtr() {
  if (mapinfo_ == nullptr) {
    std::lock_guard<std::mutex> lock(mapinfo_mutex_);
      std::shared_ptr<Mapinfo> mapinfo(new Mapinfo());
      mapinfo_ = mapinfo;
      mapinfo_->LoadMapInfo();
  }
  return mapinfo_.get();
}

}  // namespace map
}  // namespace acu
