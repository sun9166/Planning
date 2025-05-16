#ifndef COMMON_TOOBOX_COLLISION_DETECT_INCLUDE_COLLISION_DETECT_H_
#define COMMON_TOOBOX_COLLISION_DETECT_INCLUDE_COLLISION_DETECT_H_
#include "common/toolbox/geometry/include/site.h"
#include "common/toolbox/geometry/include/dwa_header.h"
#include "common/base/log/include/log.h"
#include "datapool/include/cognition_typedef.h"
#include "datapool/include/common_typedef.h"
#include <locale>


using namespace acu::planning;
using namespace acu::dwa;
#define COLLISION_INNER 2
class ObjectCollisionDetect
{
public:
  ObjectCollisionDetect() {};
  ~ObjectCollisionDetect() {};

  // param:
       // direction: 0:front; 1:back; -1:default
  template <class T>
  static bool CollisionDetection(const T &point,
                                 acu::dwa::SortedTree &obs,
                                 const CarModel &car_model,
                                 const int &direction,
                                 geometry_msgs::Point32 &collision_point,
                                 int half = 0) {

    //half 0:whole car, 1:left-half car, 2:right-half car

    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly;
    collision_point.x = __DBL_MAX__;
    collision_point.y = __DBL_MAX__;

    // std::cout << "obs size::" << obs.size() << std::endl;

    tp.x = point.x;
    tp.y = point.y;
    // tp.z = point.angle;
    tp.z = point.angle / 180 * M_PI;
    // std::cout << "points::" << tp.x << "," << tp.y << "," << tp.z << "," << point.globalangle << std::endl;
    // std::cout << "car::" << car_model.front_over_hang << "," << car_model.back_over_hang << "," << car_model.half_wheel<< std::endl;
    double front_over_hang = car_model.front_over_hang;
    double back_over_hang = car_model.back_over_hang;
    if (direction == 0) {
      front_over_hang += 0.2;
    } else if(direction == 1) {
      back_over_hang += 0.2;
    } else {
      back_over_hang = back_over_hang;
      front_over_hang = front_over_hang;
    }
    if(half == 1)
    {
      std::tie(p, q) = geometry::carmodel_left(tp,
                                          poly,
                                          front_over_hang,
                                          back_over_hang,
                                          car_model.half_wheel);      
    }
    else if(half == 2)
    {
      std::tie(p, q) = geometry::carmodel_right(tp,
                                          poly,
                                          front_over_hang,
                                          back_over_hang,
                                          car_model.half_wheel);        
    }
    else
    {
      std::tie(p, q) = geometry::carmodel(tp,
                                          poly,
                                          front_over_hang,
                                          back_over_hang,
                                          car_model.half_wheel + 0.05);        
    }


    auto ha = obs.lower_bound(p.x);
    if (!obs.count(ha->first) || ha->first > q.x ) {
      return false;
    }
    for (; ha->first <= q.x && ha != obs.end(); ++ha) {
      float temp = ha->first;
      auto hi = obs[temp].lower_bound(p);
      if (!obs[temp].count(*hi) || (*hi).y > q.y ) {
        // return false;
        continue;
      }
      for (; (*hi).y <= q.y && hi != obs[ha->first].end(); ++hi) {
        if (geometry::CheckPointInPolygon<geometry_msgs::Point32>(*hi, poly)) {
          collision_point.x = hi->x;
          collision_point.y = hi->y;
          collision_point.z = hi->z;
          // AWARN << "collision_point: " << collision_point.x << "," << collision_point.y;
          return true;
        }
      }

    }
    return false;
  }


  template <class T>
  static bool CollisionDetection(const std::vector<T> &point_vec,
                                 acu::dwa::SortedTree &obs,
                                 const CarModel &car_model,
                                 geometry_msgs::Point32 &collision_point,
                                 double valid_dis = numeric_limits<double>::max()) {

    if (point_vec.size() == 0 || point_vec.size() == 1) return false;
    double dis = 0;
    auto it = point_vec.begin();
    int counter = 0;
    for (; it != point_vec.end() && std::next(it, 1) != point_vec.end(); ++it) {
      double temp = std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      dis += temp;
      counter++;

      if (dis > valid_dis) break;
      if (counter % COLLISION_INNER == 0) {
        if (ObjectCollisionDetect::CollisionDetection<T>(*it, obs, car_model, 0, collision_point)) {
          // std::cout << "collision::" << obs.size() << std::endl;
          return true;
        }
      }

    }
    return false;
  }

  template <class T>
  static bool AdapterCollisionDetection(const std::vector<T> &point_vec,
                                        acu::dwa::SortedTree &obs,
                                        const CarModel &car_model,
                                        geometry_msgs::Point32 &collision_point,
                                        double valid_dis = numeric_limits<double>::max()) {

    if (point_vec.size() == 0 || point_vec.size() == 1) return false;
    double dis = 0;
    CarModel cur_model;
    cur_model.Set(car_model.front_over_hang, car_model.back_over_hang, car_model.half_wheel);
    auto it = point_vec.begin();
    int counter = 0;
    for (; it != point_vec.end() && std::next(it, 1) != point_vec.end(); ++it) {
      double temp = std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      dis += temp;
      counter++;
      if (dis > valid_dis) break;
      float v = it->linear_velocity;
      float w = it->angular_velocity;
      double curvature = fabs(v) < 1e-2 ? 0.0 : fabs(w / v);
      double offset = 0.05;
      if (curvature < 0.0) {
        offset = 0.05;
      } else if (curvature < 0.005) {
        offset = 0.05;
      } else if (curvature < 0.02) {
        offset = 0.1;
      } else if (curvature < 0.1) {
        offset = 0.15;
      } else if (curvature < 0.2) {
        offset = 0.2;
      } else if (curvature < 0.5) {
        offset = 0.25;;
      } else if (curvature < 0.667) {
        offset = 0.35;
      } else {
        offset = 0.4;
      }
      double square_xy = it->x * it->x + it->y * it->y;
      if (square_xy < 9.0) {
        offset = 0.0;
      }
      cur_model.half_wheel = car_model.half_wheel + offset;
      if (counter % COLLISION_INNER == 0) {
        if (ObjectCollisionDetect::CollisionDetection<T>(*it, obs, cur_model, 0, collision_point)) {
          // std::cout << "collision::" << obs.size() << std::endl;
          return true;
        }
      }
    }
    return false;
  }

  template <class T, class ContainerT>
  static bool RoadCollisionDetection_dis(const ContainerT &point_vec,
                                         double& collid_dis,
                                         acu::dwa::SortedTree &obs,
                                         const CarModel &car_model,
                                         geometry_msgs::Point32 &collision_point,
                                         double& collid_height,
                                         double valid_dis = numeric_limits<double>::max(),
                                         int half = 0) {

    // AWARN << "RoadCollisionDetection " << point_vec.size();
    collid_dis = numeric_limits<double>::max();
    collid_height = 0.0;
    int count = 0;
    if (point_vec.size() == 0 || point_vec.size() == 1) return false;
    double dis = 0;
    auto it = point_vec.begin();
    int counter = 0;
    for (; it != point_vec.end() && std::next(it, 1) != point_vec.end(); ++it) {
      // AINFO << "ROAD POINT x|y  " << it->x << "|" << it->y << "|" << it->angle << "|" << it->globalangle;
      double temp = std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      dis += temp;
      count++;
      counter++;
      if (dis > valid_dis) break;
      // if (count < 25) continue;
      if (counter % COLLISION_INNER == 0) {
        int pt_direction = it->reverse ? 1 : 0;
        if (ObjectCollisionDetect::CollisionDetection<T>(*it, obs, car_model, pt_direction, collision_point, half)) {
          // AWARN << "RoadCollisionDetection out index|x|y " << count << "|" << it->x << "|" << it->y;
          // AWARN << "object collision " << count << "|" << collision_point.x << "|" << collision_point.y;
          collid_dis = dis;
          collid_height = collision_point.z; 
          collision_point.z = count;  //for index  xy
          return true;
        }
      }
    }

    // AWARN << "RoadCollisionDetection out" ;
    return false;
  }

  template <class T, class ContainerT>
  static bool RoadAdapterCollisionDetection_dis(const ContainerT &point_vec,
                                                double &collid_dis,
                                                acu::dwa::SortedTree &obs,
                                                const CarModel &car_model,
                                                geometry_msgs::Point32 &collision_point,
                                                double& collid_height,
                                                double valid_dis = numeric_limits<double>::max()) {

    // AWARN << "RoadAdapterCollisionDetection " << point_vec.size();
    collid_dis = numeric_limits<double>::max();
    collid_height = 0.0;
    int count = 0;
    if (point_vec.size() == 0 || point_vec.size() == 1) return false;
    double dis = 0;
    CarModel cur_model;
    cur_model.Set(car_model.front_over_hang, car_model.back_over_hang, car_model.half_wheel);
    auto it = point_vec.begin();
    int counter = 0;
    for (; it != point_vec.end() && std::next(it, 1) != point_vec.end(); ++it) {
      // AINFO << "ROAD POINT x|y  " << it->x << "|" << it->y << "|" << it->angle << "|" << it->globalangle;
      double temp = std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      dis += temp;
      count++;
      counter++;
      if (dis > valid_dis) break;
      // if (count < 25) continue;
      cur_model.half_wheel = car_model.half_wheel + it->velocity;
      if (counter % COLLISION_INNER == 0) {
        int pt_direction = it->reverse ? 1 : 0;
        if (ObjectCollisionDetect::CollisionDetection<T>(*it, obs, cur_model, pt_direction, collision_point)) {
          // AWARN << "RoadAdapterCollisionDetection out index|x|y " << count << "|" << it->x << "|" << it->y;
          // AWARN << "object collision " << count << "|" << collision_point.x << "|" << collision_point.y;
          collid_dis = dis;
          collid_height = collision_point.z;
          collision_point.z = count;  //for index  xy
          return true;
        }
      }
    }

    // AWARN << "RoadAdapterCollisionDetection out" ;
    return false;
  }

  static bool RoadCollisionCheck(const std::list<geometry::Site> &point_vec,
                                 acu::dwa::SortedTree &obs,
                                 const CarModel &car_model,
                                 const double &valid_dis,
                                 CollisionInfo &collision_info) {

    // AWARN << "RoadAdapterCollisionDetection " << point_vec.size();
    collision_info.Reset();
    if (point_vec.size() == 0 || point_vec.size() == 1) return false;
    int counter = 0;
    double dis = 0;
    auto it = point_vec.begin();
    geometry_msgs::Point32 collision_point;
    for (; it != point_vec.end() && std::next(it, 1) != point_vec.end(); ++it) {
      if (dis > valid_dis) break;
      dis += std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      counter++;
      int pt_direction = it->reverse ? 1 : 0;
      if (ObjectCollisionDetect::CollisionDetection<geometry::Site>(*it, obs, car_model, pt_direction, collision_point)) {
          collision_info.path_pt_info = *it;
          collision_info.path_pt_info.index = counter;
          collision_info.path_pt_info.length = dis;
          collision_info.cell_info.x = collision_point.x;
          collision_info.cell_info.y = collision_point.y;
          collision_info.cell_info.height = collision_point.z;
          return true;
        }
    }
    return false;
  }

  static CollisionInfoVec CollisionDetectionVec(const std::list<geometry::Site> &ego,
      acu::dwa::SortedTree &obspts, VehicleElem &vehiclemodel) {
    CollisionInfoVec collision_info_vec;
    if (ego.size() == 0) {
      return collision_info_vec;
    }
    geometry_msgs::Point32 collision_cell;
    geometry_msgs::Point32 no_collision;
    geometry_msgs::Point32 p, q, tp, tp0;
    geometry_msgs::Polygon poly, poly0;
    collision_cell.x = __DBL_MAX__;
    collision_cell.y = __DBL_MAX__;
    collision_cell.z = __DBL_MAX__; // used for motion to record collision point index at path.
    tp0.x = ego.front().x;
    tp0.y = ego.front().y;
    tp0.z = ego.front().angle * M_PI / 180.0;
    no_collision = collision_cell;
    double test_dis = 0.0;
    geometry::carmodel(tp0, poly0, vehiclemodel.frontoverhang,
                       vehiclemodel.backoverhang,
                       vehiclemodel.halfwheeltrack);
    int i = 0;
    int counter = 0;
    for (auto it = ego.begin(); it != ego.end() &&
         std::next(it, 1) != ego.end(); ++it) {
      ++i;
      counter++;
      // size_t i = 0; i + 1 < ego.size(); i += 4) {
      tp.x = it->x;
      tp.y = it->y;
      tp.z = it->angle * M_PI / 180.0;
      double point_dis = std::hypot(tp.x - std::next(it, 1)->x,
                                    tp.y - std::next(it, 1)->y);
      test_dis += point_dis;
      if (test_dis > 20.0) break;

      if (counter % COLLISION_INNER == 0) {
        continue;
      }
      std::tie(p, q) = geometry::carmodel(tp, poly, vehiclemodel.frontoverhang,
                                          vehiclemodel.backoverhang,
                                          vehiclemodel.halfwheeltrack);
      auto ha = obspts.lower_bound(p.x);

      if (!obspts.count(ha->first) || ha->first > q.x) {
        // continue;
      } else {
        for (; ha->first <= q.x && ha != obspts.end(); ++ha) {
          auto hi = obspts[ha->first].lower_bound(p);
          if (!obspts[ha->first].count(*hi) || (*hi).y > q.y) {
            // continue;
          } else {
            for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi) {
              if (geometry::CheckPointInPolygon<geometry_msgs::Point32>(*hi, poly)) {
                if ( geometry::CheckPointInPolygon<geometry_msgs::Point32>(*hi, poly0)) continue;
                collision_cell.x = hi->x;
                collision_cell.y = hi->y;
                collision_cell.z = static_cast<float>(i);// used for motion to record collision point index in path.
                // ROS_WARN_STREAM("[dwaplanner] collide id "<<collision_cell.z
                //   <<"\tcollide cell.x "<<collision_cell.x<<"\tcollide cell.y "<<collision_cell.y);
                collision_info_vec.emplace_back(make_tuple(1, collision_cell));
                return collision_info_vec;
              }
            }
          }
        }
      }
    }
    return collision_info_vec;
  }

};

#endif  // COMMON_TOOBOX_COLLISION_DETECT_INCLUDE_COLLISION_DETECT_H_