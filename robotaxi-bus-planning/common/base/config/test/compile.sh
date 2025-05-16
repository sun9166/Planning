protoc config_schema.proto --cpp_out=./
g++ main.cpp config_schema.pb.cc ../src/param_config_manager.cpp -I.. -I. -I../include `pkg-config --cflags --libs protobuf` --std=c++11