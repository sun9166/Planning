#ifndef CALMCAR_DDS_COMMON_H
#define CALMCAR_DDS_COMMON_H

#ifdef _WIN32
#ifdef CALMCAR_DDS_BUILD_LIBRARY
#define CALMCAR_RCLCPP_EXPORT __declspec(dllexport)
#elif defined(CALMCAR_DDS_LIBRARY)
#define CALMCAR_DDS_EXPORT __declspec(dllimport)
#else  // not CALMCAR_DDS_BUILD_LIBRARY
#define CALMCAR_DDS_EXPORT
#endif  // CALMCAR_DDS_BUILD_LIBRARY
#else   // not _WIN32
#define CALMCAR_DDS_EXPORT
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <iterator>
#include <typeinfo>

#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/variant.hpp>
#include <boost/any.hpp>

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastrtps/utils/md5.h>
#include <fastcdr/exceptions/BadParamException.h>
#include <fastrtps/fastrtps_all.h>

#include <rclcpp/rclcpp.h>

#include "any.h"
#include "json_data.h"

#define MODULE_NAME_DDS  "message"

#endif // CALMCAR_DDS_COMMON_H
