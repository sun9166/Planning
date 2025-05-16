/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLSTRUCT_COMMON_H_
#define ACU1_XMLSTRUCT_COMMON_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace acu {
namespace xmlmap {
// common struct
struct Point {
  double x;
  double y;
  double z;
};
typedef std::vector<Point> PointSet;
// header
struct Origin {
  int zone;
  int multpcd_enable;
  Point center_point;
};
struct Boundary {
  double xg;
  double yg;
  double length;
  double width;
};
struct Header {
  std::string name;
  std::string version;
  std::string date;
  Origin origin;
  Boundary boundary;
};
// roads
typedef std::vector<double> AngleSet;
struct Geometry {
  double length;
  PointSet pointset;
  AngleSet angleset;
};

struct Segment {
  std::string id;
  std::string type;
  std::string contact_boundary_id;
  Geometry geometry;
};
typedef std::vector<Segment> Roads;

struct Outline {
  double area;
  PointSet pointset;
};
typedef std::vector<Outline> Outlines;

struct RoadRegion{
  std::string id;
  Outline outline;
};
typedef std::vector<RoadRegion> Surfaces;

struct Curb  {
  std::string id;
  int sequence;
  Geometry geometry;
};
typedef std::vector<Curb> Curbs;

struct FunctionRegion {
  std::string id;
  std::string name;
  std::string type;
  Outlines outlines;
  Curbs curbs;
};
typedef std::vector<FunctionRegion> Regions;

// boundaries
struct BoundaryLine {
  std::string boundary_line_id;
  std::string boundary_line_type;
  int boundary_line_order;
  Geometry geometry;
};
typedef std::vector<BoundaryLine> BoundaryLines;
typedef std::vector<BoundaryLines> InteriorOutlines;

struct RegionBoundary {
  BoundaryLines exterior_outline;
  InteriorOutlines interior_outlines;
};
typedef std::vector<RegionBoundary> Boundaries;

// points
struct Position {
  double roll;
  double pitch;
  double yaw;
  Point center_point;
};
struct FunctionPoint {
  std::string id;
  std::string name;
  std::string type;
  //int type;
  Position position;
};
typedef std::vector<FunctionPoint> Points;

// jobs
struct ConnectSegment {
  std::string id;
  int order;
};
typedef std::vector<ConnectSegment> ConnectSegments;

struct Job {
  std::string id;
  std::string from;
  std::string to;
  ConnectSegments connect_segments;
};
typedef std::vector<Job> Jobs;

// pathpoint
struct Location {
  std::string id;
  Point center_point;
};
typedef std::vector<Location> PathPoint;

// topology
typedef std::vector<std::string> ConnectIds;
struct Edges {
  std::string id;
  ConnectIds connect_ids;
};
typedef std::vector<Edges> Topology;

// vectormap
struct Vectormap {
  Header header;
  Roads roads;
  Surfaces surfaces;
  Regions regions;
  Boundaries boundaries;
  Points points;
  Jobs jobs;
  PathPoint pathpoint;
  Topology topology;
};
}
}
#endif
