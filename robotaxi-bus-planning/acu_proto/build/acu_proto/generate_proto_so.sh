if [ -f /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/acu_proto/build/proto_tmp/lib/libacu_proto_proto.so ];
then 
	echo "not to need generate proto so" 
 exit 1
 fi 
 echo "gennerating protobuf so file" 

g++ /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/acu_proto/build/proto_tmp/*.cc `pkg-config --cflags --libs protobuf` -I/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/acu_proto/build/proto_tmp/ --std=c++11 -fPIC -shared -o /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/acu_proto/build/proto_tmp/lib/libacu_proto.so