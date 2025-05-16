RunBuild() {
    n=`nproc`
    cmake -S . -B build  -DCMAKE_BUILD_TYPE=Release  
}

RunBuild

cp ./build/proto_tmp/*.pb.h ../include

if [ $CROSSCOMPILE ]
then
    echo "crosscompile......"
    cp ./build/proto_tmp/lib/* ../lib/j6_lib
else
    echo "x86 compile......"
    cp ./build/proto_tmp/lib/* ../lib/x86_lib
fi
