#ifndef COMMON_TOOLBOX_GEOMETRY_INCLUDE_DWA_HEADER_H_
#define COMMON_TOOLBOX_GEOMETRY_INCLUDE_DWA_HEADER_H_

// c++ lib
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/heap/binomial_heap.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/thread.hpp>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"


// other lib

//#include <tf/transform_broadcaster.h>
#include <bitset>
#include <deque>

#include "common/toolbox/geometry/include/site.h"

namespace acu {
namespace dwa {
using namespace std;



static float rad2Deg(float radian) {
  if (radian > 2 * M_PI) {
    radian -= 2 * M_PI;
  } else if (radian < 0.0) {
    radian += 2 * M_PI;
  }
  return radian * 180 / M_PI;
}

static float deg2Rad(float degree) {
  if (degree > 360.0) {
    degree -= 360.0;
  } else if (degree < 0.0) {
    degree += 360.0;
  }
  return M_PI * degree / 180.0;
}

static double decimal_process(float data_to_process, int n) {
  double t1;
  t1 = data_to_process - (int)data_to_process;
  t1 *= pow(10, n);
  if (fabs(t1 - (int)t1) > 0.5 - 1e-5) t1 += t1 / fabs(t1);
  t1 = (int)t1;
  t1 /= pow(10, n);
  t1 += (int)data_to_process;
  data_to_process = t1;
  return t1;
}

static double grid_process(double data_to_process, double t, int n) {
  int inv = t * pow(10, n + 1);
  int i_a = (int)(data_to_process * pow(10, n + 1));
  int i_b = i_a / inv;
  if (fabs(i_a % inv) > (inv / 2 - 1e-5)) {
    i_b += i_b / fabs(i_b);
  }
  data_to_process = double(i_b * inv) / pow(10, n + 1);
  return data_to_process;
}
/**
 * @brief 函数类
 * @brief 作为hash_map的比较函数
 * @brief 查找的时候不同的key往往可能会用到相同的hash值
 */
template <typename T>
struct hash_compare {
  bool operator()(const T &lhs, const T &rhs) const {
    return //(fabs(grid_process(lhs.heading,0.1,1)-grid_process(rhs.heading,0.1,1))<=1e-3) &&
        (fabs(grid_process(lhs.x, 0.2, 1) - grid_process(rhs.x, 0.2, 1)) <= 1e-3) &&
        (fabs(grid_process(lhs.y, 0.2, 1) - grid_process(rhs.y, 0.2, 1)) <= 1e-3);
  }
};

/**
 * @brief 函数类
 * @brief 作为hash_map的hash函数
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"
 */
template <typename T>
class hash_key {
public:
  size_t operator()(const T &str) const {
    std::hash<float> hash_fn;
    return size_t(hash_fn(str.x + str.y + str.heading));
  }
};

typedef struct KINEMATIC {
  // m/s
  float max_linear_velocity;
  // ras/s
  float max_angular_velocity;
  // m/s^2
  float max_linear_acceleration;
  // m/s^3
  float maximum_acceleration;
  // m/s^3
  float maximum_decceleration;
  // rad/s^2
  float max_angular_acceleration;
  // m/s^2
  float min_turn_radius;
  // m/s^2
  float centrifugal_acceleration;  // Multiply the gravitational acceleration
} sKinematic;

typedef struct VehicleElem {
  /*Unit: metre*/
  float length;
  float width;
  float halfwheeltrack;
  float frontoverhang;
  float backoverhang;
  float wheelbase;
  float headwheel;
  float tailwheel;

  /*turnangle, Unit: degree*/

  /*Left wheel steer angle*/
  float innerangle;
  /*Right wheel steer angle*/
  float outerangle;
  //>= 2.5 dm
  float outersafedis;
  float innersafedis;
  VehicleElem() {
    length = 0.0;
    width = 0.0;
    halfwheeltrack = 0.0;
    frontoverhang = 0.0;
    backoverhang = 0.0;
    wheelbase = 0.0;
    headwheel = 0.0;
    tailwheel = 0.0;
    innerangle = 0.0;
    outerangle = 0.0;
    outersafedis = 0.0;
    innersafedis = 0.0;
  }
} VehicleElem;

typedef struct ROADELEMENT {
  /*Unit: metre*/
  float roadwidthmin;
  float turnradiusmin;
  float turnradiusmax;
  float roadouterradius;
  float roadinnerradius;
} sRoadElem;



/**
 * @brief The boost::heap::binomial_heap implementaion of the priority_queue
 * @brief template pair and template priority_queue and template node
 */
template <typename T, typename priority_t>
struct PriorityNode {
  typedef std::pair<T, priority_t> PQElement;

  PQElement node;

  PriorityNode() {}

  PriorityNode(PQElement ref) : node(ref) {}

  PriorityNode(T a, priority_t b) {
    node.first = a;
    node.second = b;
  }

  /**
   * \brief The overload of the operator "()"
   *
   */
  struct CompareNode {
    bool operator()(const PQElement lhs, const PQElement rhs) const {
      return (lhs.second > rhs.second);
    }
  };

  /**
   * \brief The overload of the operator "<"
   * \make the big top heap
   */
  friend bool operator<(const PriorityNode lhs, const PriorityNode rhs) {
    return (lhs.node.second < rhs.node.second);
  }

  // The default priority queue is big top elements heap!!!
  typedef boost::heap::priority_queue<PriorityNode> PriorityQueue;
  typedef boost::heap::binomial_heap<PQElement,
          boost::heap::compare<CompareNode>>
          boostPriority;
  typedef std::priority_queue<PQElement, std::vector<PQElement>, CompareNode>
  stdPriority;

  boostPriority elements;
  boostPriority big_elements;
};

/**
 * @brief The boost::heap::priority_queue implementaion of the priority_queue
 */
template <typename T, typename priority_t>
class CostNode {
public:
  typedef boost::heap::priority_queue<CostNode> boostPriorityQueue;

  typedef std::pair<T, priority_t> PQElement;
  PQElement node;

  CostNode() {}

  CostNode(PQElement ref) : node(ref) {}

  CostNode(T a, priority_t b) {
    node.first = a;
    node.second = b;
  }

  ~CostNode() {}

  /**
   * \brief The overload of the operator "<"
   *
   */
  friend bool operator<(const CostNode lhs, const CostNode rhs) {
    return (lhs.node.second < rhs.node.second);
  }

  struct CompareNode {
    bool operator()(const CostNode lhs, const CostNode rhs) const {
      return (lhs.node.second < rhs.node.second);
    }
  };
};

/**
 * @brief The multeset obstacles tree
 */
template <typename T>
class PtsTree {
public:
  PtsTree() {}
  PtsTree(T ref) : elem(ref) {}
  ~PtsTree() {}
  T elem;
  struct CompareLeafX {
    bool operator()(const T lhs, const T rhs) const { return lhs.x < rhs.x; }
  };
  struct CompareLeafY {
    bool operator()(const T lhs, const T rhs) const { return lhs.y < rhs.y; }
  };
  typedef std::multiset<T, CompareLeafX> xtree;
  typedef std::multiset<T, CompareLeafY> ytree;
};


struct DwaPoint : public geometry::Site {
  // m/s
  float linear_velocity;
  // rad/s
  float angular_velocity;
  float heading;

  float halfwheelbias_left;
  float halfwheelbias_right;


  bool operator==(const DwaPoint &rhs) {
    return (fabs(heading - rhs.heading) <= 1e-3 && fabs(x - rhs.x) <= 1e-3 &&
            fabs(y - rhs.y) <= 1e-3 &&
            fabs(linear_velocity - rhs.linear_velocity) <= 1e-3 &&
            fabs(angular_velocity - rhs.angular_velocity) <= 1e-3);
  }
  DwaPoint(): Site() {
    linear_velocity = 0.0;
    angular_velocity = 0.0;
    heading = 0.0;
    halfwheelbias_left = 0.0;
    halfwheelbias_right = 0.0;
  }

  void Set(double x, double y, double heading, double linear_velocity = 0.0, double angular_velocity = 0.0) {
    this->x = x;
    this->y = y;
    this->heading = heading;
    this->linear_velocity = linear_velocity;
    this->angular_velocity = angular_velocity;
  }

  void SetBias(const float& bl, const float& br)
  {
    this->halfwheelbias_left = bl;
    this->halfwheelbias_right = br;
  }

  bool IsBiasEqual() const
  {
    return(fabs(this->halfwheelbias_left - this->halfwheelbias_right) < 1e-3);
  }

};

template <typename Val>
using UnorderedMapPoint =
    std::unordered_map<DwaPoint, Val, hash_key<DwaPoint>,
    hash_compare<DwaPoint>>;

using SortedTree =
    std::map<float, PtsTree<geometry_msgs::Point32>::ytree>;

}
}


#endif