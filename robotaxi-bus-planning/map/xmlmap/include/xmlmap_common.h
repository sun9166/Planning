/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLMAP_COMMON_H_
#define ACU1_XMLMAP_COMMON_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "xmlstruct_common.h"
#include "xmlstruct_mapinfo.h"

namespace acu {
namespace xmlmap {

class SegmentInfo;
class RoadRegionInfo;
class FunctionRegionInfo;
class FunctionPointInfo;
class JobInfo;
class LocationInfo;
class EdgesInfo;
class BoundaryLineInfo;

class XmlMap;

using SegmentInfoConstPtr = std::shared_ptr<const SegmentInfo>;
using RoadRegionInfoConstPtr = std::shared_ptr<const RoadRegionInfo>;
using FunctionRegionInfoConstPtr = std::shared_ptr<const FunctionRegionInfo>;
using FunctionPointInfoConstPtr = std::shared_ptr<const FunctionPointInfo>;
using JobInfoConstPtr = std::shared_ptr<const JobInfo>;
using LocationInfoConstPtr = std::shared_ptr<const LocationInfo>;
using EdgesInfoConstPtr = std::shared_ptr<const EdgesInfo>;
using BoundaryLineInfoConstPtr = std::shared_ptr<const BoundaryLineInfo>;

class SegmentInfo {
 public:
  explicit SegmentInfo(const Segment &segment) : segment_(segment) {}
  const std::string &id() const { return segment_.id; }
  const std::string &type() const { return segment_.type; }
  const std::string &contact_boundary_id() const { return segment_.contact_boundary_id; }
  const double &length() const { return segment_.geometry.length; }
  const Segment &segment() const { return segment_; }

 private:
  Segment segment_;
};

class BoundaryLineInfo {
 public:
  explicit BoundaryLineInfo(const BoundaryLine &boundaryline) : boundaryline_(boundaryline) {}
  const std::string &id() const { return boundaryline_.boundary_line_id; }
  const std::string &type() const { return boundaryline_.boundary_line_type; }
  const int &order() const { return boundaryline_.boundary_line_order; }
  const Geometry &geometry() const { return boundaryline_.geometry; }
  const BoundaryLine &boundaryline() const { return boundaryline_; }

 private:
  BoundaryLine boundaryline_;
};

class RoadRegionInfo{
  public:
  explicit RoadRegionInfo(const RoadRegion &road_region) : road_region_(road_region) {}
  const std::string &id() const { return road_region_.id; }
  //const double &area() const { return road_region_.outline.area; }
  const RoadRegion &roadRegion() const { return road_region_; }

 private:
  RoadRegion road_region_;
};

class FunctionRegionInfo {
 public:
  explicit FunctionRegionInfo(const FunctionRegion &function_region)
      : function_region_(function_region) {}
  const std::string &id() const { return function_region_.id; }
  const std::string &name() const { return function_region_.name; }
  const FunctionRegion &functionRegion() const { return function_region_; }

 private:
  FunctionRegion function_region_;
};

class FunctionPointInfo {
 public:
  explicit FunctionPointInfo(const FunctionPoint &function_point)
      : function_point_(function_point) {}
  const std::string &id() const { return function_point_.id; }
  const std::string &name() const { return function_point_.name; }
  const FunctionPoint &functionPoint() const { return function_point_; }

 private:
  FunctionPoint function_point_;
};

class JobInfo {
 public:
  explicit JobInfo(const Job &job) : job_(job) {}
  const std::string &id() const { return job_.id; }
  const Job &job() const { return job_; }

 private:
  Job job_;
};

class LocationInfo {
 public:
  explicit LocationInfo(const Location &location) : location_(location) {}
  const std::string &id() const { return location_.id; }
  const Location &location() const { return location_; }

 private:
  Location location_;
};

class EdgesInfo {
 public:
  explicit EdgesInfo(const Edges &edges) : edges_(edges) {}
  const std::string &id() const { return edges_.id; }
  const Edges &edges() const { return edges_; }

 private:
  Edges edges_;
};
}  // namespace hdmap
}  // namespace acu

#endif
