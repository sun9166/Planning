#ifndef __GEOMETRY_MSGS__
   #define  __GEOMETRY_MSGS__
#include <string>
#include <vector>
#include <map>
#include <boost/smart_ptr.hpp>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Point32_
{
  typedef Point32_<ContainerAllocator> Type;

  Point32_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  Point32_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::geometry_msgs::Point32_<ContainerAllocator> const> ConstPtr;

}; // struct Point32_

typedef ::geometry_msgs::Point32_<std::allocator<void> > Point32;

typedef boost::shared_ptr< ::geometry_msgs::Point32 > Point32Ptr;
typedef boost::shared_ptr< ::geometry_msgs::Point32 const> Point32ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::Point32_<ContainerAllocator> & v)
{

   return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::geometry_msgs::Point32_<ContainerAllocator1> & lhs, const ::geometry_msgs::Point32_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::geometry_msgs::Point32_<ContainerAllocator1> & lhs, const ::geometry_msgs::Point32_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace geometry_msgs


#endif
