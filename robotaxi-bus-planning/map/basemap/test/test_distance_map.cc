#include "map/map_loader/include/map_loader.h"
#include "map/basemap/include/basemap.h"
#include<ctime>

// using namespace acu:map;
int main() {

    static acu::map::BaseMap* bmp = acu::map::MapLoader::GetBasemapPtr();
    // BasemapInfo basemap_info;

    // double check
    if (bmp == nullptr) {
        bmp = acu::map::MapLoader::GetBasemapPtr();
        if (bmp == nullptr) {
            std::cout << "[test_distance_map] : Get BaseMap failed." << std::endl;
            return -1;
        }
    }

    // basemap_info.is_valid = true;
    double xg = 454621.145657;
    double yg = 4441763.66725;
    float angle = -8.1;

    clock_t startTime,endTime;
    startTime = clock();//计时开始
    PointGCCS pgccs{xg, yg, angle};
    for (int i = -50; i < 50; i++) {
        for (int j = -100; j < 100; j++) {
            // double xg = (double)i;
            // double yg = (double)j;
            // int i = 0;
            // int j = 0;
            PointVCS pvcs{0.1*i, 0.1*j, 0.0};
            // std::cout << std::setprecision(10) << "xg: " << xg << ", yg: " << yg << std::endl;
            // bool foot_in_basemap = bmp->IsInFreespace(pgccs, pvcs);
            // if (!foot_in_basemap) {
            //     std::cout << "[test_distance_map] : ego is out basemap." << std::endl;
            // }
            // else
            // {
            //     std::cout << "[test_distance_map] : ego is in basemap." << std::endl;
            // }
            
            double distance = bmp->DistanceToFreeSpace(pgccs, pvcs);
            std::cout << "distance: [ " << distance << " ] to free space border. " 
                << i << ", " << j << std::endl;
        }
    }
    endTime = clock();

    std::cout << "Used time : " << (double)(endTime - startTime)/CLOCKS_PER_SEC << " s." << std::endl;


    return 0;
}