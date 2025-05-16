
list(APPEND PROTO_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/build/proto_tmp/)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/build/proto_tmp/)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/build/proto_tmp/lib/)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/)

if(EXISTS /usr/bin/protoc)
   set(protoc_path "/usr/bin/protoc ")
#if(EXISTS /work/share/acu3_0/protobuf-3.5.1/src/.libs/protoc)
#   set(protoc_path "/work/share/acu3_0/protobuf-3.5.1/src/.libs/protoc ")
   message(STATUS ${protoc_path} " exists")
else()
   set(protoc_path "/usr/local/bin/protoc ")
   message(STATUS "use " ${protoc_path} )
   set(is_local "yes")
endif()
file(WRITE ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh "proto_name=$2; \n file_name=\${proto_name%.*}; \n pb_file_name=$file_name.pb.h \necho $pb_file_name \n if [ -f ${CMAKE_SOURCE_DIR}/build/proto_tmp/$pb_file_name ];\nthen \n\techo \" the file $pb_file_name is exist , not to need generate proto cc\" \n exit 1\n fi \n echo \"gennerating protobuf cc file\" \n\n")
file(APPEND ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh ${protoc_path} ) 
foreach(item IN LISTS PROTO_INCLUDE_DIRS)
  file(APPEND ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh "-I${item} ")
  set(PROTO_HEADER_PATH ${PROTO_HEADER_PATH} "-I${item} ")
endforeach(item)
string(REPLACE ";" "  " NEW_PATH ${PROTO_HEADER_PATH}) #by:guanqp, Xavier platform will add ";", so replace ";" with backspace symbol.
#message("after:${NEW_PATH}")
set(PROTO_HEADER_PATH ${NEW_PATH})

if(is_local)
  file(APPEND ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh "--cpp_out=${CMAKE_SOURCE_DIR}/build/proto_tmp/ -I$1 $2")
else()
  file(APPEND ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh "--cpp_out=${CMAKE_SOURCE_DIR}/build/proto_tmp/ -I$1 $1/$2")
endif()


foreach(src_item IN LISTS PROTO_SRC_FILE)
  execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto.sh ${PROJECT_SOURCE_DIR}/  ${src_item})
endforeach(src_item)

set(PROTO_COMPILE_CROSS /work/crosscompile/gcc-ubuntu-11.4.0-hobot-x86_64-aarch64-linux-gnu/usr/bin/aarch64-linux-gnu-g++)

set(PROTO_COMPILE_LOCAL g++)
if(DEFINED CROSSCOMPILE)
   set(PROTO_COMPILE_ACU ${PROTO_COMPILE_CROSS})
else()
   set(PROTO_COMPILE_ACU ${PROTO_COMPILE_LOCAL})
endif()

message(STATUS "PROTO_COMPILE_ACU = ${PROTO_COMPILE_ACU}")

set(PKG_INFO_CROSS "-pthread -I/work/crosscompile/usr/local/include -L/work/crosscompile/usr/local/lib -lprotobuf -pthread -lpthread")

set(PKG_INFO_LOCAL "`pkg-config --cflags --libs protobuf`")

if(DEFINED CROSSCOMPILE)
   set(PKG_INFO_ACU ${PKG_INFO_CROSS})
else()
    set(PKG_INFO_ACU ${PKG_INFO_LOCAL})
endif()

file(WRITE ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto_so.sh "if [ -f ${CMAKE_SOURCE_DIR}/build/proto_tmp/lib/lib${PROJECT_NAME}_proto.so ];\nthen \n\techo \"not to need generate proto so\" \n exit 1\n fi \n echo \"gennerating protobuf so file\" \n\n")
file(APPEND ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto_so.sh "${PROTO_COMPILE_ACU} ${CMAKE_SOURCE_DIR}/build/proto_tmp/*.cc ${PKG_INFO_ACU} -I${CMAKE_SOURCE_DIR}/build/proto_tmp/ -I${CMAKE_SOURCE_DIR}/../../../common/common_proto/ --std=c++11 -fPIC -shared -o ${CMAKE_SOURCE_DIR}/build/proto_tmp/lib/lib${PROJECT_NAME}.so")
execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/build/${PROJECT_NAME}/generate_proto_so.sh)

