if [ -f /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/lib/libplanning_proto_proto.so ];
then 
	echo "not to need generate proto so" 
 exit 1
 fi 
 echo "gennerating protobuf so file" 

/work/crosscompile/gcc-ubuntu-11.4.0-hobot-x86_64-aarch64-linux-gnu/usr/bin/aarch64-linux-gnu-g++ /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/*.cc -pthread -I/work/crosscompile/usr/local/include -L/work/crosscompile/usr/local/lib -lprotobuf -pthread -lpthread -I/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/ -I/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/../../../common/common_proto/ --std=c++11 -fPIC -shared -o /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/lib/libplanning_proto.so