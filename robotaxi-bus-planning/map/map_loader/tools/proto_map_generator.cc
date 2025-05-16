/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: proto_map_generator.cc
*
* Description: generate proto map from xml

*
* History:
* lbh         2018/05/15    1.0.0    build this module.
******************************************************************************/
#include <glob.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include "common/util/file.h"
#include "map.pb.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h> 
#include "map/map_loader/include/map_loader.h"
#include "common/base/log/include/acu_node.h"
#include "map/vectormap/src/hdmap/adapter/opendrive_adapter.h"

using namespace std;

int ExistFile(string file)
{
  return access(file.c_str(), F_OK);
}

int main(int argc, char **argv) {
	bool Enable_GLOG_Screen = true;
	acu::common::AcuNode::Init("generator", Enable_GLOG_Screen);

	auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
  auto mpheader = mapinfo->GetMapParamHeader();
  if (!mpheader.VectormapEnabled) {
    return -1;
  }

  std::string path = mapinfo->GetMapPath() + mpheader.vector_map_dir;
  std::string file = path + mpheader.vector_map_file;

  acu::hdmap::Map pb_map;
  acu::hdmap::Roadnet pb_roadnet;

  if (0 != ExistFile(file + ".xml")) {
    AINFO << "No vector map found " << path << ".xml";
    return 0;
  }

  if ((ExistFile(file + "_hdmap.bin") != 0) ||
      (ExistFile(file + "_hdmap.txt") != 0) ||
      (ExistFile(file + "_roadnet.bin") != 0) ||
      (ExistFile(file + "_roadnet.txt") != 0)) {
    AINFO << "begin to load: " << file << ".xml";
    ACU_CHECK(acu::hdmap::adapter::OpendriveAdapter::LoadData(
                  file + ".xml", &pb_map, &pb_roadnet));
  } else {
    AINFO << "no xmls need to converse under " << path;
    return 0;
  }
  std::string output_file = file + "_hdmap.bin";
  if (ExistFile(output_file) == 0) {
    AERROR << output_file << " exists.";
    return 0;
  } else {
    ACU_CHECK(acu::common::util::SetProtoToBinaryFile(pb_map, output_file));
    AINFO << "creat: " << output_file;
  }

  output_file = file + "_roadnet.bin";
  if (ExistFile(output_file) == 0) {
    AERROR << output_file << " exists.";
    return 0;
  } else {
    ACU_CHECK(
      acu::common::util::SetProtoToBinaryFile(pb_roadnet, output_file));
    AINFO << "creat: " << output_file;
  }

  output_file = file + "_hdmap.txt";
  if (ExistFile(output_file) == 0) {
    AERROR << output_file << " exists.";
    return 0;
  } else {
    ACU_CHECK(
      acu::common::util::SetProtoToASCIIFile(pb_map, output_file));
    AINFO << "creat: " << output_file;
  }

  output_file = file + "_roadnet.txt";
  if (ExistFile(output_file) == 0) {
    AERROR << output_file << " exists.";
    return 0;
  } else {
    ACU_CHECK(
      acu::common::util::SetProtoToASCIIFile(pb_roadnet, output_file));
    AINFO << "creat: " << output_file;
  }

  AINFO << "load map success";
  return 0;
}
