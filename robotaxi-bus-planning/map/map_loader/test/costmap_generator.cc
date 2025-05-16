#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <stdio.h>
#include "map/map_loader/include/map_loader.h"
#include "map/map_loader/include/show_vectormap.h"


using namespace std;
using namespace acu::map;
using namespace acu::hdmap;
using namespace acu::vectormap;

#define MAX_COST 1000000.0

struct RoadStruct {
	string id;
	double length;
	double round_cost;
	vector<string> successors;

	//For dijkstra
	bool known;
	double distance;

	//For A*
	bool is_open;
	bool is_close;
	double g_score;
	double h_score;
	double f_score;
	string parent;
};

int road_number_ = 0;
vector<RoadStruct> road_net_;
unordered_map<string, int> road_map_;
vector<vector<double>> cost_matrix_;

bool GetRoadNet() {
	road_net_.clear();
	auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
	if (!mapinfo->IsVectorMapEnabled()) {
		printf("MapEnabled failed.");
		return false;
	}
	const VectorMap* hdmap = acu::map::MapLoader::GetVectorMapPtr();
	if (hdmap == nullptr) {
		printf("Vcmap ptr init failed.");
		return false;
	}
	const RoadNodeTable* road_net_table = hdmap->GetRoadNet();
	for (auto it = road_net_table->begin(); it != road_net_table->end(); it++) {
		RoadStruct road_node; 
		road_node.id = it->first;
		road_node.length = it->second->length();
		// Calculate the road length for temp
		roadLanes lanes;
		hdmap->GetRoadLanes(it->second->id(), lanes);
		bool is_prediction_road = true;
		for (auto &lane :lanes.at(0).second) {//道路包含车道是否全部为虚拟车道
			acu::hdmap::Lane::LaneType type;
			if (hdmap->GetLaneType(lane, type) == 0) {
				if (type == acu::hdmap::Lane::LaneType::Lane_LaneType_CITY_DRIVING ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_ISLAND ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_PARKING ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGLEFT ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGSTRAIGHT) {
					is_prediction_road = false;
					break;
				}
			}
		}
		if (is_prediction_road) {
			AWARN<<"road "<<it->second->id().id()<<" is prediction road.";
			road_node.length = MAX_COST;
			road_net_.push_back(road_node);
			road_map_[it->first] = road_net_.size() - 1;
			continue;
		}
		if (lanes.size() && lanes.at(0).second.size()) {
			LaneInfoConstPtr ptr = hdmap->GetLaneById(lanes.at(0).second.at(0));
			road_node.length = ptr->total_length();
		}
		
		for (auto &successor : it->second->GetSuccessorIds()) {//下级道路包含车道是否全部为虚拟车道
			roadLanes success_lanes;
			hdmap->GetRoadLanes(successor, success_lanes);
			for (auto &lane :success_lanes.at(0).second) {
				acu::hdmap::Lane::LaneType type;
				if (hdmap->GetLaneType(lane, type) == 0 && 
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_CITY_DRIVING ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_ISLAND ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_PARKING ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGLEFT ||
					type == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGSTRAIGHT) {
					road_node.successors.push_back(successor.id());
					break;
				}
			}
		}
		road_net_.push_back(road_node);
		road_map_[it->first] = road_net_.size() - 1;
	}
	road_number_ = road_net_.size();
	return !road_net_.empty();
}

void Dijkstra(string start_id) {
	int cur_index = road_map_[start_id];
	double min_dis = MAX_COST;
	for (int i = 0; i < road_number_; i++) {
		road_net_.at(i).known = false;
		road_net_.at(i).distance = MAX_COST;
	}
	road_net_.at(cur_index).known = true;
	road_net_.at(cur_index).distance = 0.0;
	for (int j = 0; j < road_number_; j++) {
		for (auto successor : road_net_.at(cur_index).successors) {
			int index = road_map_[successor];
			road_net_.at(index).distance = min(road_net_.at(index).distance, 
					road_net_.at(cur_index).distance + road_net_.at(index).length);
		}
		for (int jj = 0; jj < road_number_; jj++) {
			if (!road_net_.at(jj).known && road_net_.at(jj).distance < min_dis) {
				cur_index = jj;
				min_dis = road_net_.at(jj).distance;
			}
		}
		if (road_net_.at(cur_index).known) {
			break;
		}
		road_net_.at(cur_index).known = true;
		min_dis = MAX_COST;
	}
}

bool GenerateCostmap() {
	if (!GetRoadNet()) {
		printf("GenerateCostmap failed.");
		return false;
	}
	for (int i = 0; i < road_number_; i++) {
		vector<double> cost_vector;
		Dijkstra(road_net_.at(i).id);
		for (auto road : road_net_) {
			cost_vector.push_back(road.distance);
		}
		cost_matrix_.push_back(cost_vector);
	}
	for (int j = 0; j < road_number_; j++) {
		road_net_.at(j).round_cost = MAX_COST;
		for (int jj = 0; jj < road_number_; jj++) {
			if (jj == j) {
				continue;
			}
			double dis = cost_matrix_.at(j).at(jj) + cost_matrix_.at(jj).at(j);
			road_net_.at(j).round_cost = min(road_net_.at(j).round_cost, dis);
		}
	}
	return true;
}

bool ExitFile(string& name) {
	if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

int main(int argc, char *argv[]) {
	string node_name = "map_loader_test";

	bool Enable_GLOG_Screen = true;
	acu::common::AcuNode::Init(node_name, Enable_GLOG_Screen);

	if (true) {
		auto mapinfo = acu::map::MapLoader::GetMapinfoPtr();
		auto mpheader = mapinfo->GetMapParamHeader();
		if (!mpheader.VectormapEnabled) {
	
			return -1;
		}
		string file_path = mapinfo->GetMapPath() + mpheader.vector_map_dir + 
    	        		   mpheader.vector_map_file + "_costmap.txt";

    	if (ExitFile(file_path)) {

			return -1;
    	}
		if (!GenerateCostmap()) {
			printf("VectormapEnabled failed.");
			return -1;
		}
		ofstream FileWriteLog(file_path, ios::out|ios::app);
		if (FileWriteLog) {
			for (int i = 0; i < cost_matrix_.size(); i++) {
				for (int j = 0; j < cost_matrix_[i].size(); j++) {
					FileWriteLog << cost_matrix_[i][j]<< "\t";
				}
				FileWriteLog <<endl;
			}
		}
		FileWriteLog.close();
	}
	return 0;
}

