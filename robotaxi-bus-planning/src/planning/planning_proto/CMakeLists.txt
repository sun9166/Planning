cmake_minimum_required(VERSION 2.8.3)
project(planning_proto)

set(CROSSCOMPILE $ENV{CROSSCOMPILE})


list(APPEND PROTO_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/../../../common/common_proto/)


list(APPEND PROTO_SRC_FILE motionplanning.proto)   
list(APPEND PROTO_SRC_FILE sl_boundary.proto)
list(APPEND PROTO_SRC_FILE waypoint_sampler_config.proto)
list(APPEND PROTO_SRC_FILE st_boundary_config.proto)
list(APPEND PROTO_SRC_FILE prediction_trajectory.proto)
list(APPEND PROTO_SRC_FILE dp_poly_path_config.proto)
list(APPEND PROTO_SRC_FILE dp_st_speed_config.proto)
list(APPEND PROTO_SRC_FILE poly_st_speed_config.proto)
list(APPEND PROTO_SRC_FILE qp_spline_path_config.proto)
list(APPEND PROTO_SRC_FILE qp_st_speed_config.proto)
list(APPEND PROTO_SRC_FILE planning_config.proto)
list(APPEND PROTO_SRC_FILE poly_vt_speed_config.proto) 
list(APPEND PROTO_SRC_FILE qp_problem.proto)
list(APPEND PROTO_SRC_FILE decision.proto) 
list(APPEND PROTO_SRC_FILE planning_internal_info.proto)
list(APPEND PROTO_SRC_FILE decision_config.proto) 
list(APPEND PROTO_SRC_FILE fem_pos_deviation_smoother_config.proto)
list(APPEND PROTO_SRC_FILE cos_theta_smoother_config.proto)
list(APPEND PROTO_SRC_FILE reference_line_smoother_config.proto)  

include(protobuf.cmake)         
          