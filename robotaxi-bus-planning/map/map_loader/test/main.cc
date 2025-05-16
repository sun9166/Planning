#include "common/base/log/include/acu_node.h"
#include <string>
#include <unistd.h>
#include "map/map_loader/include/map_loader.h"
#include "map/map_loader/include/show_vectormap.h"
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include "common/base/config/include/param_config_manager.h"

using namespace std;
using namespace acu::map;
using namespace acu::hdmap;
using namespace acu::vectormap;
using acu::common::PointENU;

PointGCCS car_gccs;
LaneInfoConstPtr lane_ptr;
PointENU loc_enupoint;
double nearest_s;
double nearest_l;
const acu::vectormap::VectorMap* vcmap_;

int test_getmapinfo()
{
	Mapinfo* mapinfo = MapLoader::GetMapinfoPtr();

	MapItem map_item;
	if (!mapinfo->GetCurrentMap(map_item)) {
		AERROR << "GetAvailableMaps failed!";
		return -1;
	}
	AINFO << "GetCurrentMap: " << map_item.name << " - " << map_item.date;

	std::vector<MapItem> maps;
	if (!mapinfo->GetAvailableMaps(maps)) {
		AERROR << "GetAvailableMaps failed!";
		return -1;
	}
	for (auto map : maps) {
		AINFO << "GetAvailableMaps: " << map.name << " - " << map.date;
		if ((map.name == map_item.name) && (map.date == map_item.date)) {
			AINFO << "Got it";
		}
	}
	AINFO << mapinfo->GetWorkspace();
	AINFO << mapinfo->GetMapPath();
	AINFO << mapinfo->GetMapHeader().name;
	AINFO << mapinfo->GetMapHeader().date;
	AINFO << mapinfo->GetMapParamHeader().image_matrix_size;
	AINFO << mapinfo->GetMapParamHeader().basemap_image_size;
	AINFO << mapinfo->GetMapParamHeader().basemap_resolution_ratio;
	AINFO << mapinfo->GetMapParamHeader().image_pixel_chars;
	AINFO << mapinfo->GetMapParamHeader().basemap_filename_level;
	AINFO << mapinfo->GetMapParamHeader().basemap_dir;
	AINFO << mapinfo->GetMapParamHeader().vector_map_file;
	AINFO << mapinfo->GetMapinfoHeader().name;
	AINFO << mapinfo->GetMapinfoHeader().zone;
	AINFO << mapinfo->GetMapinfoHeader().version;
	AINFO << mapinfo->GetMapinfoHeader().type;
	AINFO << mapinfo->GetMapinfoHeader().describe;
	AINFO << mapinfo->GetMapinfoHeader().date;

	return 0;
}

int test_basemap()
{
	// if (LoadVectorFile(car_gccs) == -1) {
	// 	return -1;
	// }
	// AINFO << "car_pgccs: (" << car_gccs.xg << ", " << car_gccs.yg << ") ";

	struct timeval current_time;
	gettimeofday(&current_time, NULL);
	double time1 = current_time.tv_sec + (double)current_time.tv_usec / 1000000.0;
	BaseMap* basemap = MapLoader::GetBasemapPtr();
	PointVCS pvcs = {0, 0, 0};
	int nIn = 0;
	int nOut = 0;
	for(int i = 0; i < 10000; i++){
		if (basemap->IsInFreespace(car_gccs, pvcs)) {
			nIn++;
			// AINFO << "ego(" << car_gccs.xg << ", " << car_gccs.yg << ") is in basemap.";
		} else {
			nOut++;
			AINFO << "ego(" << car_gccs.xg << ", " << car_gccs.yg << ") is out basemap.";
		}
	}
	gettimeofday(&current_time, NULL);
	double time2 = current_time.tv_sec + (double)current_time.tv_usec / 1000000.0;
	std::cout << "nIn:" << nIn << ", nOut: "<< nOut ;
	std::cout << std::setprecision(17) << ", IsInFreespace() 10000 times, Using time t2-t1: " << (time2 - time1) * 1000 << "(ms)" << std::endl;

	return 0;
}

int test_lane(string lane_id) {
	Id Id_lane;
	Id_lane.set_id(lane_id);
	vector<Id> suc_lanes, pre_lanes;
	bool find_input_lane = false;
	if (vcmap_->GetLaneSuccessorIDs(Id_lane, suc_lanes) > 0) {
		for (auto &suc_lane : suc_lanes) {
			if (suc_lane.id() == lane_id) {
				find_input_lane = true;
				break;
			}
		}
		if (find_input_lane) {
			std::cout<<"Lane "<<lane_id<<" 's suc lanes contain self"<<std::endl;
		}
	}
	if (vcmap_->GetLanePredecessorIDs(Id_lane, pre_lanes) > 0) {
		for (auto &pre_lane : pre_lanes) {
			if (pre_lane.id() == lane_id) {
				find_input_lane = true;
				break;
			}
		}
		if (find_input_lane) {
			std::cout<<"Lane "<<lane_id<<" 's pre lanes contain self"<<std::endl;
		}
	}
	if (suc_lanes.size()) {
		vector<Id> suc_pre_lanes, all_suc_pre_lanes;
		for (auto &suc_lane : suc_lanes) {
			if (vcmap_->GetLanePredecessorIDs(suc_lane, suc_pre_lanes) > 0) {
				all_suc_pre_lanes.insert(all_suc_pre_lanes.end(), suc_pre_lanes.begin(), suc_pre_lanes.end());
			}
		}
		find_input_lane = false;
		for (auto &suc_pre_lane : all_suc_pre_lanes) {
			if (suc_pre_lane.id() == lane_id) {
				find_input_lane = true;
				break;
			}
		}
		if (!find_input_lane) {
			std::cout<<"Lane "<<lane_id<<" 's suc's pre don't contain self"<<std::endl;
		}
	}
	if (pre_lanes.size()) {
		vector<Id> pre_suc_lanes, all_pre_suc_lanes;
		for (auto &pre_lane : pre_lanes) {
			if (vcmap_->GetLaneSuccessorIDs(pre_lane, pre_suc_lanes) > 0) {
				all_pre_suc_lanes.insert(all_pre_suc_lanes.end(), pre_suc_lanes.begin(), pre_suc_lanes.end());
			}
		}
		find_input_lane = false;
		for (auto &pre_suc_lane : all_pre_suc_lanes) {
			if (pre_suc_lane.id() == lane_id) {
				find_input_lane = true;
				break;
			}
		}
		if (!find_input_lane) {
			std::cout<<"Lane "<<lane_id<<" 's pre's suc don't contain self"<<std::endl;
		}
	}
	acu::hdmap::Lane::LaneType lanetype;
	if (vcmap_->GetLaneType(Id_lane, lanetype) == 0) {
		if ((int)lanetype == 3 || (int)lanetype == 4) {
			Id left_Id, right_Id;
			if (vcmap_->GetLeftRightLaneIDs(Id_lane, left_Id, right_Id) >= 2) {
				std::cout<<"Lane "<<lane_id<<" is special but has right parallel lane"<<std::endl;
			}
		}
		if ((int)lanetype == 3 || (int)lanetype == 4) {
			for (auto &suc_lane : suc_lanes) {
				acu::hdmap::Lane::LaneType suc_lanetype;
				if (vcmap_->GetLaneType(suc_lane, suc_lanetype) == 0) {
					if ((int)suc_lanetype == 2) {
						std::cout<<"Lane "<<lane_id<<" is special but has suc driving type lane"<<std::endl;
					}
				}
			}
		}
	} else {
		std::cout<<"Lane "<<lane_id<<" can't get lane type"<<std::endl;
	}

	if (lanetype != 2) return 0;
	LaneInfoConstPtr input_lane_ptr = vcmap_->GetLaneById(lane_id);
	if (input_lane_ptr != nullptr) {
		double left = 0.0, right = 0.0;
		bool width_wrong = false;
		for (double s = 0.0; s < input_lane_ptr->total_length(); s+=0.5) {
			input_lane_ptr->GetWidth(s, &left, &right);
			if (left < 1.1) {
				width_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m left boundary < 1.1m"<<std::endl;
			}
			if (right < 1.1) {
				width_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m right boundary < 1.1m"<<std::endl;
			}
			input_lane_ptr->GetRoadWidth(s, &left, &right);
			if (left < 1.1) {
				width_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m left road boundary < 1.1m"<<std::endl;
			}
			if (right < 1.1) {
				width_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m right road boundary < 1.1m"<<std::endl;
			}
			if (width_wrong) break;
		}
		bool width_maybe_wrong = false;
		for (double s = 0.0; s < input_lane_ptr->total_length(); s+=0.5) {
			input_lane_ptr->GetWidth(s, &left, &right);
			if (left > 3.0) {
				width_maybe_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m left boundary > 3m"<<std::endl;
			} 
			if (right > 3.0) {
				width_maybe_wrong = true;
				std::cout<<"Lane "<<lane_id<<" at "<<s<<"m right boundary > 3m"<<std::endl;
			}
			if (width_maybe_wrong) break;
		}
	}
	return 0;
}

int test_road(string road_id) {
	// 当前road的pre和suc间还有其他共同道路
	printf("----------------------------------------------------------------------------");
	Id Id_road;
	Id_road.set_id(road_id);
	vector<Id> suc_roads, pre_roads;
	bool find_input_road = false;
	if (vcmap_->GetRoadSuccessorIDs(Id_road, suc_roads) == 0) {
		find_input_road = false;
		for (auto &suc_road : suc_roads) {
			if (suc_road.id() == road_id) {
				find_input_road = true;
				break;
			}
		}
		if (find_input_road) {
			std::cout<<"Road "<<road_id<<" 's suc roads contain self"<<std::endl;
		}
	}
	
	if (vcmap_->GetRoadPredecessorIDs(Id_road, pre_roads) == 0) {
		find_input_road = false;
		for (auto &pre_road : pre_roads) {
			if (pre_road.id() == road_id) {
				find_input_road = true;
				break;
			}
		}
		if (find_input_road) {
			std::cout<<"Road "<<road_id<<" 's pre roads contain self"<<std::endl;
		}
	}
	
	if (suc_roads.size()) {
		vector<Id> suc_pre_roads, all_suc_pre_roads;
		for (auto &suc_road : suc_roads) {
			if (vcmap_->GetRoadPredecessorIDs(suc_road, suc_pre_roads) == 0) {
				all_suc_pre_roads.insert(all_suc_pre_roads.end(), suc_pre_roads.begin(), suc_pre_roads.end());
			}
		}
		find_input_road = false;
		for (auto &suc_pre_road : all_suc_pre_roads) {
			if (suc_pre_road.id() == road_id) {
				find_input_road = true;
				break;
			}
		}
		if (!find_input_road) {
			std::cout<<"Road "<<road_id<<" 's suc roads contain self"<<std::endl;
		}
	}
	
	if (pre_roads.size()) {
		vector<Id> pre_suc_roads, all_pre_suc_roads;
		for (auto &pre_road : pre_roads) {
			if (vcmap_->GetRoadSuccessorIDs(pre_road, pre_suc_roads) == 0) {
				all_pre_suc_roads.insert(all_pre_suc_roads.end(), pre_suc_roads.begin(), pre_suc_roads.end());
			}
		}
		find_input_road = false;
		for (auto &pre_suc_road : all_pre_suc_roads) {
			if (pre_suc_road.id() == road_id) {
				find_input_road = true;
				break;
			}
		}
	}
	
	if (suc_roads.size() && pre_roads.size()) {
		vector<Id> suc_pre_roads, pre_suc_roads, all_suc_pre_roads, all_pre_suc_roads;
		for (auto &suc_road : suc_roads) {
			if (vcmap_->GetRoadPredecessorIDs(suc_road, suc_pre_roads) == 0) {
				all_suc_pre_roads.insert(all_suc_pre_roads.end(), suc_pre_roads.begin(), suc_pre_roads.end());
			}
		}
		for (auto &pre_road : pre_roads) {
			if (vcmap_->GetRoadSuccessorIDs(pre_road, pre_suc_roads) == 0) {
				all_pre_suc_roads.insert(all_pre_suc_roads.end(), pre_suc_roads.begin(), pre_suc_roads.end());
			}
		}
		vector<string> same_pre_suc_roads;
		for (auto &suc_pre : all_suc_pre_roads) {
			for (auto &pre_suc : all_pre_suc_roads) {
				if (suc_pre.id() == pre_suc.id() && suc_pre.id() != road_id) {
					same_pre_suc_roads.push_back(suc_pre.id());
				}
			}
		}
		for (auto &road : same_pre_suc_roads) {
			std::cout<<"Road "<<road_id<<" 's suc roads and pre roads have other road "<<road<<std::endl;
		}
	}
	
	roadLanes sections_lanes_in_road;
	if (vcmap_->GetRoadLanes(Id_road, sections_lanes_in_road) == 0 && sections_lanes_in_road.size()) {
		auto lanes_in_road = sections_lanes_in_road.begin()->second;
		vector<Id> suc_lanes, roadlanes_suc_lanes, roadlanes_suc_roads;
		for (auto &it : lanes_in_road) {
			test_lane(it.id());
			if (vcmap_->GetLaneSuccessorIDs(it, suc_lanes) > 0) {
				roadlanes_suc_lanes.insert(roadlanes_suc_lanes.end(), suc_lanes.begin(), suc_lanes.end());
			}
		}
		for (auto &roadlanes_suc_lane : roadlanes_suc_lanes) {
			Id temp_Id_road;
			if (vcmap_->GetRoad(roadlanes_suc_lane, temp_Id_road) == 0) {
				roadlanes_suc_roads.push_back(temp_Id_road);
			}
		}
		for (auto &road_suc_road : suc_roads) {
			bool find_flag = false;
			for (auto &lane_suc_road : roadlanes_suc_roads) {
				if (road_suc_road.id() == lane_suc_road.id()) {
					find_flag = true;
					break;
				}
			}
			if (!find_flag) {
				std::cout<<"Road "<<road_id<<" 's suc "<<road_suc_road.id()<<" can't be arrived suc by containing lanes"<<std::endl;
			}
		}
		vector<Id> pre_lanes, roadlanes_pre_lanes, roadlanes_pre_roads;
		for (auto &it : lanes_in_road) {
			LaneInfoConstPtr temp_ptr = vcmap_->GetLaneById(it);
			if (temp_ptr == nullptr) continue;
			if (vcmap_->GetLanePredecessorIDs(it, pre_lanes) > 0) {
				roadlanes_pre_lanes.insert(roadlanes_pre_lanes.end(), pre_lanes.begin(), pre_lanes.end());
			}
		}
		for (auto &roadlanes_pre_lane : roadlanes_pre_lanes) {
			Id temp_Id_road;
			if (vcmap_->GetRoad(roadlanes_pre_lane, temp_Id_road) == 0) {
				roadlanes_pre_roads.push_back(temp_Id_road);
			}
		}
		for (auto &road_pre_road : pre_roads) {
			bool find_flag = false;
			for (auto &lane_pre_road : roadlanes_pre_roads) {
				if (road_pre_road.id() == lane_pre_road.id()) {
					find_flag = true;
					break;
				}
			}
			if (!find_flag) {
				std::cout<<"Road "<<road_id<<" 's pre road "<<road_pre_road.id()<<" can't be arrived by lane"<<std::endl;
			}
		}
	}
	
	auto lanes_in_road = sections_lanes_in_road.begin()->second;
	if (lanes_in_road.size()) {
		bool is_in_junction = false;
		for (auto &road_lane : lanes_in_road) {
			LaneInfoConstPtr road_lane_ptr = vcmap_->GetLaneById(road_lane);
			if (road_lane_ptr == nullptr) continue;
			const std::vector<OverlapInfoConstPtr> junctions_ptr = road_lane_ptr->junctions();
			if (junctions_ptr.size() > 0) {
				is_in_junction = true;
				break;
			}
		}
		if (is_in_junction) {
			
			for (auto &road_lane : lanes_in_road) {
				LaneInfoConstPtr road_lane_ptr = vcmap_->GetLaneById(road_lane);
				if (road_lane_ptr == nullptr) continue;
				const std::vector<OverlapInfoConstPtr> junctions_ptr = road_lane_ptr->junctions();
				if (junctions_ptr.size() <= 0) {
					std::cout<<"Road "<<road_id<<" is in junction, but contains"<<road_lane.id()<<" isn't in junction"<<std::endl;
				}
			}
			
			vector<int> lane_turns;
			for (auto &road_lane : lanes_in_road) {
				LaneInfoConstPtr road_lane_ptr = vcmap_->GetLaneById(road_lane);
				if (road_lane_ptr == nullptr) continue;
				if (lane_turns.empty()) {
					lane_turns.push_back((int)road_lane_ptr->lane().turn());
				} else if ((int)road_lane_ptr->lane().turn() != lane_turns.back()) {
					std::cout<<"Road "<<road_id<<"  is in junction, but lanes turns are not same"<<std::endl;
				}
			}
			
			for (auto &road_lane : lanes_in_road) {
				LaneInfoConstPtr road_lane_ptr = vcmap_->GetLaneById(road_lane);
				if (road_lane_ptr == nullptr) continue;
				vector<string> merge_lanes;
				const vector<OverlapInfoConstPtr> crosses_ptr = road_lane_ptr->cross_lanes();
  				for (auto &cross_ptr : crosses_ptr) {
    				if (cross_ptr->overlap().object().size() != 2) continue;
    				for (auto &object : cross_ptr->overlap().object()) {
    					if (object.id().id() != road_lane_ptr->id().id() && object.lane_overlap_info().is_merge()) {
    						merge_lanes.push_back(object.id().id());
    					}
					}
				}
				bool parallel_merge_flag = false;
				for (auto &cross_lane : merge_lanes) {
					for (auto &it : lanes_in_road) {
						if (cross_lane == it.id()) {
							parallel_merge_flag = true;
							break;
						}
					}
				}
				if (parallel_merge_flag) {
					std::cout<<"Road "<<road_id<<" is in junction, but containing lanes merge themselves"<<std::endl;
				}
			}
		}
	}
	return 0;
}

int test_getvectormap()
{
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr();
	if (vcmap_ == nullptr) {
		printf("Vcmap ptr init failed.\n");
		return -1;
	}
	const RoadNodeTable* road_net_table = vcmap_->GetRoadNet();
	for (auto it = road_net_table->begin(); it != road_net_table->end(); it++) {
		string road_id = it->first;
		test_road(road_id);
	}
	return 0;
}


void Test_GetLeftRightDistanceToFreespace() {
	//point(443598.218357, 4436350.485970), angle -1.497872
	car_gccs.xg = 443598.218357;
	car_gccs.yg = 4436350.485970;
	car_gccs.angle = -1.497872;
	// loc_enupoint.set_x(car_gccs.xg);
	// loc_enupoint.set_y(car_gccs.yg);


	BaseMap* basemap = MapLoader::GetBasemapPtr();
	PointVCS pvcs = {0, 0, 0};

	acu::common::math::Vec2d tmp_point(car_gccs.xg, car_gccs.yg);
	acu::common::math::Box2d car_box(tmp_point, car_gccs.angle, 1000.0, 1000.0);

	double left = 0.0;
	double right = 0.0;
	double heading = car_gccs.angle / 180 * M_PI;
	if(basemap->GetLeftRightDistanceToFreespace(car_gccs, heading, left, right) < 0){
		std::cout << std::setprecision(10) << "xg: " << car_gccs.xg <<", yg: " << car_gccs.yg  
			<< ", angle: " << car_gccs.angle
			<< ",  left: " << left << ", right: " << right << std::endl;
	}
	else {  
		std::cout << "\tleft: " << left << ", right: " << right <<", angel:" << car_gccs.angle 
		<< std::setprecision(10) <<", xg: " << car_gccs.xg <<", yg: " << car_gccs.yg<< std::endl;
	}
}

int Test_cleararea() {
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	  return 0;
  	}
  	 string land_id = "241497552_1_-1"; //clearare
	//  string land_id = "241524149_1_-1"; // crosswalk
  	 LaneInfoConstPtr lane_ptr = vcmap_->GetLaneById(land_id);
	 
  	 const std::vector<OverlapInfoConstPtr> clear_areas_ptr = lane_ptr->clear_areas();
	//    const std::vector<OverlapInfoConstPtr> clear_areas_ptr = lane_ptr->crosswalks();
  	 std::cout<<"clear area size "<<clear_areas_ptr.size()<<std::endl;
}

int Test_GetNearestLane() {
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	//   return 0;
  	}
  	PointENU loc_enu_point;
	loc_enu_point.set_x(0.0);
	loc_enu_point.set_y(0.0);
	double s, l;
  	LaneInfoConstPtr lane_ptr;
  	if (vcmap_->GetNearestLane(loc_enu_point, lane_ptr, s, l) == 0 &&
      lane_ptr != nullptr) {
		std::cout<<"GetNearestLane is OK."<<std::endl; 
	}else {
		std::cout<<"GetNearestLane is NOT OK."<<std::endl;
	}
}

int Test_RightOfWay1() {	//
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	  	return 0;
  	}
	//N4006987E11633957		右转弯 vs 直行
	Id lane_id;	
	std::vector<Id> lane_id1;
	lane_id.set_id("288144610_1_-3");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288152807_1_-1");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288013683_1_-2");
	lane_id1.push_back(lane_id);

	std::vector<Id> lane_id2;
	lane_id.set_id("288144525_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288144493_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288013683_1_-2");
	lane_id2.push_back(lane_id);

	int exit_relation;
	int relation;
	CollisionIndex col_index1, col_index2;
	int lanePrivilege;
	vcmap_->GetLanesRelvant(lane_id1, lane_id2, exit_relation, relation,
		col_index1, col_index2, lanePrivilege);

	std::cout << "\texit_relation: " << exit_relation << ", relation: " << relation
		<< ", Privilege: " << lanePrivilege << std::endl;
	std::cout << "\tcol_index1 start: " << col_index1.start_s << 
					 ", end: "<< col_index1.end_s << 
					 ", index: "<<col_index1.index << std::endl;
	std::cout << "\tcol_index2 start: " << col_index2.start_s << 
					 ", end: "<< col_index2.end_s << 
					 ", index: "<<col_index2.index << std::endl;
}

int Test_RightOfWay2() {
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	  	return 0;
  	}
	//N4006987E11633957		直行  vs 直行	（对向车道）
	Id lane_id;
	std::vector<Id> lane_id1;
	lane_id.set_id("288013653_1_-2");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288144489_1_-2");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288144522_1_-2");
	lane_id1.push_back(lane_id);

	std::vector<Id> lane_id2;
	lane_id.set_id("288144525_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288144493_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288013683_1_-2");
	lane_id2.push_back(lane_id);

	int exit_relation;
	int relation;
	CollisionIndex col_index1, col_index2;
	int lanePrivilege;
	vcmap_->GetLanesRelvant(lane_id1, lane_id2, exit_relation, relation,
		col_index1, col_index2, lanePrivilege);

	std::cout << "\texit_relation: " << exit_relation << ", relation: " << relation
		<< ", Privilege: " << lanePrivilege << std::endl;
	std::cout << "\tcol_index1 start: " << col_index1.start_s << 
					 ", end: "<< col_index1.end_s << 
					 ", index: "<<col_index1.index << std::endl;
	std::cout << "\tcol_index2 start: " << col_index2.start_s << 
					 ", end: "<< col_index2.end_s << 
					 ", index: "<<col_index2.index << std::endl;
}

int Test_RightOfWay3() {
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	  	return 0;
  	}
	//N4006987E11633957		直行  vs 直行	（十字交叉）
	Id lane_id;
	std::vector<Id> lane_id1;
	lane_id.set_id("288144610_1_-1");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288144688_1_-1");
	lane_id1.push_back(lane_id);
	lane_id.set_id("288823649_1_-1");
	lane_id1.push_back(lane_id);

	std::vector<Id> lane_id2;
	lane_id.set_id("288144525_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288144493_1_-2");
	lane_id2.push_back(lane_id);
	lane_id.set_id("288013683_1_-2");
	lane_id2.push_back(lane_id);

	int exit_relation;
	int relation;
	CollisionIndex col_index1, col_index2;
	int lanePrivilege;
	vcmap_->GetLanesRelvant(lane_id1, lane_id2, exit_relation, relation,
		col_index1, col_index2, lanePrivilege);

	std::cout << "\texit_relation: " << exit_relation << ", relation: " << relation
		<< ", Privilege: " << lanePrivilege << std::endl;
	std::cout << "\tcol_index1 start: " << col_index1.start_s << 
					 ", end: "<< col_index1.end_s << 
					 ", index: "<<col_index1.index << std::endl;
	std::cout << "\tcol_index2 start: " << col_index2.start_s << 
					 ", end: "<< col_index2.end_s << 
					 ", index: "<<col_index2.index << std::endl;
}

int Test_RightOfWay4() {	//
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr."<<std::endl;
  	  	return 0;
  	}
	//N4006987E11633957		左转弯 vs 直行		类同1
	Id lane_id;	
	std::vector<Id> lane_id1;
	lane_id.set_id("287219422_1_-1");
	lane_id1.push_back(lane_id);
	lane_id.set_id("287220422_1_-1");
	lane_id1.push_back(lane_id);
	lane_id.set_id("287220624_1_-1");
	lane_id1.push_back(lane_id);

	std::vector<Id> lane_id2;
	lane_id.set_id("287220628_1_-1");
	lane_id2.push_back(lane_id);
	lane_id.set_id("287220913_1_-1");
	lane_id2.push_back(lane_id);
	lane_id.set_id("287220707_1_-1");
	lane_id2.push_back(lane_id);

	int exit_relation;
	int relation;
	CollisionIndex col_index1, col_index2;
	int lanePrivilege;
	vcmap_->GetLanesRelvant(lane_id2, lane_id1, exit_relation, relation,
		col_index1, col_index2, lanePrivilege);

	std::cout << "\texit_relation: " << exit_relation << ", relation: " << relation
		<< ", Privilege: " << lanePrivilege << std::endl;
	std::cout << "\tcol_index1 start: " << col_index1.start_s << 
					 ", end: "<< col_index1.end_s << 
					 ", index: "<<col_index1.index << std::endl;
	std::cout << "\tcol_index2 start: " << col_index2.start_s << 
					 ", end: "<< col_index2.end_s << 
					 ", index: "<<col_index2.index << std::endl;
}

int test_OppositeLanes() {
	vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
  	if (vcmap_ == nullptr) {
  		std::cout<<"vcmap_ is nulltpr." << std::endl;
  	  	return 0;
  	}

	Id lane_id;	
	std::vector<Id> opposite_ids;
	
	//test 01
	lane_id.set_id("606516251464699921_1_-1");
	vcmap_->GetOppositeLaneIDs(lane_id, opposite_ids);
	std::cout << "lane_id: " << lane_id.id() << std::endl;
	for(auto opp_id : opposite_ids) {
		std::cout << "\t\t" << opp_id.id() << std::endl;
	}
	opposite_ids.clear();

	//test 02
	lane_id.set_id("606516339947737109_1_-1");
	vcmap_->GetOppositeLaneIDs(lane_id, opposite_ids);
	std::cout << "lane_id: " << lane_id.id() << std::endl;
	for(auto opp_id : opposite_ids) {
		std::cout << "\t\t" << opp_id.id() << std::endl;
	}
	opposite_ids.clear();

	//test 02
	lane_id.set_id("606786583010349058_1_-1");
	vcmap_->GetOppositeLaneIDs(lane_id, opposite_ids);
	std::cout << "lane_id: " << lane_id.id() << std::endl;
	for(auto opp_id : opposite_ids) {
		std::cout << "\t\t" << opp_id.id() << std::endl;
	}
	opposite_ids.clear();
	
	return 0;
}

int main(int argc, char *argv[])
{
	std::string node_name = "map_loader_test";
	bool Enable_GLOG_Screen = true;
	acu::common::AcuNode::Init(node_name, Enable_GLOG_Screen);
	
	if (true) {
		// test_getmapinfo();
		// test_getvectormap();
		// Test_GetNearestLane();
		// Test_RightOfWay4();

		test_OppositeLanes();
	}


	return 0;
}

