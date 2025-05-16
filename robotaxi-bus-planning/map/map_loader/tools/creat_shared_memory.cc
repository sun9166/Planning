#include "map/map_loader/include/map_loader.h"
#include "common/base/log/include/acu_node.h"

using namespace acu::map;

int main(int argc, char **argv) {
	bool Enable_GLOG_Screen = true;
	acu::common::AcuNode::Init("create_shm", Enable_GLOG_Screen);

  BaseMap *basemap_ = MapLoader::GetBasemapPtr();
  auto r = basemap_->CreatSharedMemory();
  return 0;
}