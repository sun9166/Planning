/******************************************************************************
* Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: common
* FileName: geoheader.h
*
* Description: declare types we usually use
*
* History:
* Feng younan          2018/08/20          v1.0.0        Build this model.
******************************************************************************/
#ifndef __COMMON_GEOMETRY_GEOHEADER_H__
#define __COMMON_GEOMETRY_GEOHEADER_H__

#include "site.h"
#include "linebase.h"

#include <map>
#include <set>
#include <list>
#include <unordered_map>
#include <vector>

namespace geometry {

using SiteVec = std::vector<Site>;

using LineVec = std::vector<LineBase>;

using SiteSet = std::set<Site, Site::compare>;

using SiteList = std::list<Site>;

template <typename T>
using KeySiteMap = std::map<T, Site>;

template <typename T>
using SiteValueMap = std::map<Site, T, Site::compare>;

template <typename T>
using SiteValueUnorderedMap =
    std::unordered_map<Site, T, Site::hash_key, Site::hash_equal>;

template <typename T>
using SiteValueMultiMap = std::multimap<Site, T, Site::compare>;

}  // geometry
#endif  // __COMMON_GEOMETRY_GEOHEADER_H__