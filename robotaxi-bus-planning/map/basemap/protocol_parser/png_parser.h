/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName: png_parser.h
*
* Description: png parser

*
* History:
* lbh         2018/05/22    1.0.0    build this module.
******************************************************************************/

#ifndef BASEMAP_PNG_PARSER_H_
#define BASEMAP_PNG_PARSER_H_
#include <iostream>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include "map/basemap/protocol_parser/basemap_protocol.h"

namespace acu {
namespace map {

template <class T, size_t N>
constexpr size_t array_size(T (&)[N]) {
  return N;
}

class PNGParser {
 public:
  PNGParser(std::string protocol="IDP");
  eCellInfo GetObject(const uint bgra) const;

 private:
  void CreatMapCell(std::string protocol);
  uint UintTurn(uint in);
  std::unordered_map<uint, eCellInfo> map_cell_;
};
}  // namespace map
}  // namespace acu
#endif