/*
idriverplus
create 20181122 lbh
=========================================================================*/
#include "xmlmap.h"
#include "xml_parser/xmlmap_adapter.h"
#include "../vectormap/include/geotool.h"

namespace acu {
namespace xmlmap {

XmlVectormap::~XmlVectormap() {
  if (vectormap_ != NULL) delete vectormap_;
}

int XmlVectormap::LoadMapFromFile(const std::string& map_filename) {
  vectormap_ = new Vectormap;
  if (adapter::XmlMapAdapter::LoadData(map_filename, vectormap_) == -1)
    return -1;
  return LoadMapFromStruct(vectormap_);
}

int XmlVectormap::LoadMapFromStruct(Vectormap* vectormap) {
  if (vectormap != vectormap_) {  // avoid an unnecessary copy
    Clear();
    vectormap_ = vectormap;
  }
  for (const auto& seg : vectormap_->roads) {
    roads_table_[seg.id].reset(new SegmentInfo(seg));
  }
  for (const auto& rr : vectormap_->surfaces) {
    surfaces_table_[rr.id].reset(new RoadRegionInfo(rr));
  }
  for (const auto& fr : vectormap_->regions) {
    regions_table_[fr.id].reset(new FunctionRegionInfo(fr));
    function_region_table_[fr.name] = fr.id;
  }
  for (const auto& fp : vectormap_->points) {
    points_table_[fp.id].reset(new FunctionPointInfo(fp));
    function_point_table_[fp.name] = fp.id;
  }
  for (const auto& job : vectormap_->jobs) {
    std::string name = job.from + job.to;
    segment_table_[name] = job.connect_segments;
    jobs_table_[job.id].reset(new JobInfo(job));
  }
  for (const auto& boundary : vectormap_->boundaries) {
    for (const auto& ebl : boundary.exterior_outline) {
      exterior_outline_table_[ebl.boundary_line_id].reset(new BoundaryLineInfo(ebl));
     }
    for (const auto& ibls : boundary.interior_outlines) {
       for (const auto& ibl : ibls) {
         interior_outline_table_[ibl.boundary_line_id].reset(new BoundaryLineInfo(ibl));
      }
     }
  }
  for (const auto& fp : vectormap_->pathpoint) {
    path_point_table_[fp.id].reset(new LocationInfo(fp));
  }
  for (const auto& fp : vectormap_->topology) {
    topology_table_[fp.id].reset(new EdgesInfo(fp));
  }

  header_ = vectormap_->header;
  return 0;
}

int XmlVectormap::GetFunctionRegionIdByName(std::string name,
    std::string& id) const {
  auto it = function_region_table_.find(name);
  if (it != function_region_table_.end()) {
    id = it->second;
    return 0;
  } else
    return -1;
}
int XmlVectormap::GetFunctionPointIdByName(std::string name,
    std::string& id) const {
  auto it = function_point_table_.find(name);
  if (it != function_point_table_.end()) {
    id = it->second;
    return 0;
  } else
    return -1;
}
int XmlVectormap::GetSegmentIdByJob(std::string from, std::string to,
                                    ConnectSegments& id) const {
  std::string name = from + to;
  auto it = segment_table_.find(name);
  if (it != segment_table_.end()) {
    id = it->second;
    return 0;
  } else
    return -1;
}
int XmlVectormap::GetSegmentLengthByFRName(std::string name,
    double& length) const {
  std::string id;
  ConnectSegments connect_segments;
  if (GetFunctionRegionIdByName(name, id) == -1) return -1;
  if (GetSegmentIdByJob(id, id, connect_segments) == -1) return -1;
  for (auto i = connect_segments.begin(); i != connect_segments.end(); ++i) {
     auto sp = GetSegmentById((*i).id);
     if (sp == nullptr) return -1;
     length += sp->length();
  }
  return 0;
}

int XmlVectormap::GetSegmentOrderBySegmentId(const std::string& id, const std::string& job_id,
    int& order) const {
  auto re = GetJobById(job_id);
  Job job = re->job();
  for (int i = 0; i < job.connect_segments.size(); ++i) {
    if (id == job.connect_segments.at(i).id) {
      order = job.connect_segments.at(i).order;
      return 0;
    }
 }
 return -1;
}

int XmlVectormap::GetBoundaryLineOrderByBoundaryLineId(const std::string& id, int& order) const {
    auto it = exterior_outline_table_.find(id);
    auto it1 = interior_outline_table_.find(id);
    if (it != exterior_outline_table_.end()) {
       order = it->second->order();
       return 0;
    } else if (it1 != interior_outline_table_.end()) {
       order = it1->second->order();
       return 0;
    } else if (exterior_outline_table_.size() == 0 && interior_outline_table_.size() == 0) {
       return 1;
    }
   return -1;
}

int XmlVectormap::GetBoundaryLineTypeByBoundaryLineId(const std::string& id, std::string& type) const {
    auto it = exterior_outline_table_.find(id);
    auto it1 = interior_outline_table_.find(id);
    if (it != exterior_outline_table_.end()) {
       type = it->second->type();
       return 0;
    } else if (it1 != interior_outline_table_.end()) {
       type = it1->second->type();
       return 0;
    } else if (exterior_outline_table_.size() == 0 && interior_outline_table_.size() == 0) {
       return 1;
    }
   return -1;
}

FunctionType XmlVectormap::GetFunctionType(const std::string& id) const {
  auto it = regions_table_.find(id);
  if (it != regions_table_.end()) return FREGION;
  auto it2 = points_table_.find(id);
  if (it2 != points_table_.end()) return FPOINT;
  return UNDEFINE;
}

SegmentInfoConstPtr XmlVectormap::GetSegmentById(const std::string& id) const {
  auto it = roads_table_.find(id);
  return it != roads_table_.end() ? it->second : nullptr;
}

bool XmlVectormap::GetSegmentIds(std::vector<std::string>& segment_ids)
{
  segment_ids.clear();

  for (auto node : roads_table_) {
    segment_ids.push_back(node.first);
  }

  return true;
}

std::string XmlVectormap::GetSegmentTypeBySegmentId(const std::string& id) const {
  auto it = roads_table_.find(id);
  return it != roads_table_.end() ?  it->second->type() : NULL;
}

std::string XmlVectormap::GetBoundaryLineIdBySegmentId(const std::string& id) const {
   auto it = roads_table_.find(id);
   return it != roads_table_.end() ?  it->second->contact_boundary_id() : NULL;
}

BoundaryLineInfoConstPtr XmlVectormap::GetBoundaryLineByBoundaryLineId(const std::string& id) const {
   auto it = exterior_outline_table_.find(id);
   auto it1 = interior_outline_table_.find(id);
   if (it != exterior_outline_table_.end()) {
       return it->second;
    } else if (it1 != interior_outline_table_.end()) {
       return it1->second;
    } else if (exterior_outline_table_.size() == 0 && interior_outline_table_.size() == 0) {
       return nullptr;
    } else return nullptr;
}

RoadRegionInfoConstPtr XmlVectormap::GetRoadRegionById(const std::string& id) const {
  auto it = surfaces_table_.find(id);
  return it != surfaces_table_.end() ? it->second : nullptr;
}

FunctionRegionInfoConstPtr XmlVectormap::GetFunctionRegionById(
  const std::string& id) const {
  auto it = regions_table_.find(id);
  return it != regions_table_.end() ? it->second : nullptr;
}

FunctionPointInfoConstPtr XmlVectormap::GetFunctionPointById(
  const std::string& id) const {
  auto it = points_table_.find(id);
  return it != points_table_.end() ? it->second : nullptr;
}

JobInfoConstPtr XmlVectormap::GetJobById(const std::string& id) const {
  auto it = jobs_table_.find(id);
  return it != jobs_table_.end() ? it->second : nullptr;
}

// add by xf
LocationInfoConstPtr XmlVectormap::GetLocationById(
  const std::string& id) const {
  auto it = path_point_table_.find(id);
  return it != path_point_table_.end() ? it->second : nullptr;
}

EdgesInfoConstPtr XmlVectormap::GetEdgesById(const std::string& id) const {
  auto it = topology_table_.find(id);
  return it != topology_table_.end() ? it->second : nullptr;
}

bool XmlVectormap::GetLocationIds(std::vector<std::string>& loc_ids)
{
  loc_ids.clear();

  for (auto node : path_point_table_) {
    loc_ids.push_back(node.first);
  }

  return true;
}

bool XmlVectormap::GetEdgesIds(std::vector<std::string>& edges_ids)
{
  edges_ids.clear();

  for (auto node : topology_table_) {
    edges_ids.push_back(node.first);
  }

  return true;
}

void XmlVectormap::Clear() {
  roads_table_.clear();
  surfaces_table_.clear();
  regions_table_.clear();
  points_table_.clear();
  jobs_table_.clear();
  path_point_table_.clear();
  topology_table_.clear();
  interior_outline_table_.clear();
  exterior_outline_table_.clear();
}

}
}
