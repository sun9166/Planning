#ifndef __POLYGON_H__
    #define __POLYGON_H__
#include <string>
#include <vector>
#include <map>


#include "Point32.h"

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Polygon_
{
  typedef Polygon_<ContainerAllocator> Type;

  Polygon_()
    : points()  {
    }
  Polygon_(const ContainerAllocator& _alloc)
    : points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Point32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point32_<ContainerAllocator> >::other >  _points_type;
  _points_type points;





  typedef boost::shared_ptr< ::geometry_msgs::Polygon_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Polygon_<ContainerAllocator> const> ConstPtr;

}; // struct Polygon_

typedef ::geometry_msgs::Polygon_<std::allocator<void> > Polygon;

typedef boost::shared_ptr< ::geometry_msgs::Polygon > PolygonPtr;
typedef boost::shared_ptr< ::geometry_msgs::Polygon const> PolygonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Polygon_<ContainerAllocator> & v)
{

    return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geometry_msgs::Polygon_<ContainerAllocator1> & lhs, const ::geometry_msgs::Polygon_<ContainerAllocator2> & rhs)
{
  return lhs.points == rhs.points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geometry_msgs::Polygon_<ContainerAllocator1> & lhs, const ::geometry_msgs::Polygon_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geometry_msgs




#endif