/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName: png_manager.cc
*
* Description: find the cell rgba

*
* History:
* lbh         2018/05/22    1.0.0    build this module.
******************************************************************************/
#include "png_parser.h"

namespace acu {
namespace map {

PNGParser::PNGParser(std::string protocol) { CreatMapCell(protocol); }

eCellInfo PNGParser::GetObject(const uint bgra) const {
  auto iter = map_cell_.find(bgra);
  if (iter != map_cell_.end())
    return iter->second;

  return eCellInfo::UNKNOWN;
}

uint PNGParser::UintTurn(uint in) {
  uint out;
  ((unsigned char *)&out)[0] = ((unsigned char *)&in)[2];
  ((unsigned char *)&out)[1] = ((unsigned char *)&in)[1];
  ((unsigned char *)&out)[2] = ((unsigned char *)&in)[0];
  ((unsigned char *)&out)[3] = ((unsigned char *)&in)[3];
  return out;
}
void PNGParser::CreatMapCell(std::string protocol) {
  map_cell_.clear();
  if (protocol == "IDP") {
    for (int i = 0; i < array_size(IDP); i++) {
      map_cell_.emplace(UintTurn(*((uint *)(*(IDP + i)))),
                        static_cast<eCellInfo>(*(*(IDP + i) + 4)));
    }
  }
  if (protocol == "JD") {
    for (int i = 0; i < array_size(JD); i++) {
      map_cell_.emplace(UintTurn(*((uint *)(*(JD + i)))),
                        static_cast<eCellInfo>(*(*(JD + i) + 4)));
    }
  }
}

}  // namespace map
}  // namespace acu