/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLMAP_H_
#define ACU1_XMLMAP_H_

#include <dirent.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>


#include "xmlmap_common.h"

namespace acu {
namespace xmlmap {

enum FunctionType { UNDEFINE = -1, FPOINT, FREGION };

class XmlVectormap {
 public:
  using RoadsTable =
      std::unordered_map<std::string, std::shared_ptr<SegmentInfo>>;
  using SurfacesTable =
      std::unordered_map<std::string, std::shared_ptr<RoadRegionInfo>>;
  using RegionsTable =
      std::unordered_map<std::string, std::shared_ptr<FunctionRegionInfo>>;
  using PointsTable =
      std::unordered_map<std::string, std::shared_ptr<FunctionPointInfo>>;
  using JobsTable = std::unordered_map<std::string, std::shared_ptr<JobInfo>>;
  using PathPointTable =
      std::unordered_map<std::string, std::shared_ptr<LocationInfo>>;
  using TopologyTable =
      std::unordered_map<std::string, std::shared_ptr<EdgesInfo>>;
  using ExteriorOutlineTable = 
      std::unordered_map<std::string, std::shared_ptr<BoundaryLineInfo>>;
  using InteriorOutlineTable = 
      std::unordered_map<std::string, std::shared_ptr<BoundaryLineInfo>>;

  using FunctionRegionTable = std::unordered_map<std::string, std::string>;
  using FunctionPointTable = std::unordered_map<std::string, std::string>;
  using SegmentTable = std::unordered_map<std::string, ConnectSegments>;

 public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string& map_filename);
  /**
   * @brief get function type of id
   */
  FunctionType GetFunctionType(const std::string& id) const;
  Header GetVectormapHeader() const { return header_; }
  Boundary GetAreaBoundary() const { return header_.boundary; }
  Points* GetPointsStructPtr() const { return &vectormap_->points; }
  Regions* GetRegionsStructPtr() const { return &vectormap_->regions; }
  Vectormap* GetVectormapPtr() const { return vectormap_; }

  // @return 0 : success, -1 :failed
  int GetFunctionRegionIdByName(std::string name, std::string& id) const;
  int GetFunctionPointIdByName(std::string name, std::string& id) const;
  int GetSegmentIdByJob(std::string from, std::string to,
                        ConnectSegments& id) const;
  int GetSegmentLengthByFRName(std::string name, double& length) const;
  int GetSegmentOrderBySegmentId(const std::string& id, const std::string& job_id, int& order) const; 
  int GetBoundaryLineOrderByBoundaryLineId(const std::string& id, int& order) const; 
  int GetBoundaryLineTypeByBoundaryLineId(const std::string& id, std::string& type) const;

  std::string GetSegmentTypeBySegmentId(const std::string& id) const;
  std::string GetBoundaryLineIdBySegmentId(const std::string& id) const;

  SegmentInfoConstPtr GetSegmentById(const std::string& id) const;
  bool GetSegmentIds(std::vector<std::string>& segment_ids);
  RoadRegionInfoConstPtr GetRoadRegionById(const std::string& id) const;
  BoundaryLineInfoConstPtr GetBoundaryLineByBoundaryLineId(const std::string& id) const ;
  
  FunctionRegionInfoConstPtr GetFunctionRegionById(const std::string& id) const;
  FunctionPointInfoConstPtr GetFunctionPointById(const std::string& id) const;
  JobInfoConstPtr GetJobById(const std::string& id) const;
  LocationInfoConstPtr GetLocationById(const std::string& id) const;
  EdgesInfoConstPtr GetEdgesById(const std::string& id) const;
  bool GetLocationIds(std::vector<std::string>& loc_ids);
  bool GetEdgesIds(std::vector<std::string>& edges_ids);

  ~XmlVectormap();
  // old, will remove
  int GetSegmentIdByName(std::string name, std::string& id) const { return -1; }

 private:
  void Clear();
  int LoadMapFromStruct(Vectormap* vectormap);
  Vectormap* vectormap_;
  Header header_;
  RoadsTable roads_table_;
  SurfacesTable surfaces_table_;
  RegionsTable regions_table_;
  PointsTable points_table_;
  JobsTable jobs_table_;
  SegmentTable segment_table_;
  FunctionRegionTable function_region_table_;
  FunctionPointTable function_point_table_;
  InteriorOutlineTable interior_outline_table_;
  ExteriorOutlineTable exterior_outline_table_;

  PathPointTable path_point_table_;
  TopologyTable topology_table_;
};

}
}

#endif
