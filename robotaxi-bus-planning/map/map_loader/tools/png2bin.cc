//#include <sys/io.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "map/map_loader/include/map_loader.h"
#include "common/base/log/include/acu_node.h"

using namespace std;
using namespace cv;
using namespace acu::map;

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

int main(int argc, char **argv) {
	bool Enable_GLOG_Screen = true;
	acu::common::AcuNode::Init("png2bin", Enable_GLOG_Screen);

  ofstream outfile;
  vector<String> files;

	auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();

  std::string dir_path = mapinfo->GetMapPath() + mpheader.basemap_dir;

  AINFO << "work dir: " << dir_path;

  if(!PathExists(dir_path)){
    AERROR << "dir is not exist : " << dir_path;
    return -1;
  }
  dir_path += "/*.image";
  glob(dir_path, files, false);
  for (int i = 0; i < files.size(); i++) {
    auto file = (std::string)files[i];
    file.pop_back();
    file.pop_back();
    file.pop_back();
    file.pop_back();
    file.pop_back();
    file += "bin";
    if (access(file.c_str(), 0) == 0) continue;
    Mat Iface = imread(files[i].c_str(), CV_LOAD_IMAGE_UNCHANGED);
    if (Iface.empty()) {
      AERROR << "failed to imread : " << files[i];
      continue;
    }
    std::cout << files[i] << std::endl;
    outfile.open(file, ios::binary);
    if (!outfile.is_open()) {
      AERROR << "failed to open : " << file;
      continue;
    }
    for (int r = 0; r < Iface.rows; r++)
      outfile.write(reinterpret_cast<const char *>(Iface.ptr(r)),
                    Iface.cols * Iface.elemSize());
    if (outfile.bad()) {
      AERROR << "failed to write : " << file;
      continue;
    }
    outfile.close();
  }
  return 0;
}
