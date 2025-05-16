
#ifndef COMMON_TOOLBOX_GEOMETRY_INCLUDE_GEOHEADER_H__
#define COMMON_TOOLBOX_GEOMETRY_INCLUDE_GEOHEADER_H__

#include "common/toolbox/geometry/include/site.h"
#include "common/toolbox/geometry/include/linebase.h"

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