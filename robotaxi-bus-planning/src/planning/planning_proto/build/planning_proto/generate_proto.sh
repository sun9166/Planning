proto_name=$2; 
 file_name=${proto_name%.*}; 
 pb_file_name=$file_name.pb.h 
echo $pb_file_name 
 if [ -f /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/$pb_file_name ];
then 
	echo " the file $pb_file_name is exist , not to need generate proto cc" 
 exit 1
 fi 
 echo "gennerating protobuf cc file" 

/usr/bin/protoc -I/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/../../../common/common_proto/ -I/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/ --cpp_out=/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/src/planning/planning_proto/build/proto_tmp/ -I$1 $1/$2