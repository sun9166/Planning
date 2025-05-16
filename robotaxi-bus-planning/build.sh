#!/usr/bin/env bash

RunBuild() {
    n=`nproc`
    cmake -S . -B build  -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=${toolchain_file}  

    make -j $((n/2)) -C build
    if [ $? -ne 0 ];then
        exit 1
    fi
}

unset CROSSCOMPILE

# keyword="proto"
# search_path="."
# proto_path=`find "$search_path" -type d -maxdepth 1 -name "*$keyword*"`
# cd $proto_path
# bash protobuf_.sh 
# cd ..

# keyword="proto"
# cd ./src/planning
# search_path="."
# proto_path=`find "$search_path" -type d -maxdepth 1 -name "*$keyword*"`
# cd $proto_path
# bash protobuf_.sh 
# cd ..
# cd ../..


toolchain_file=./toolchain/Toolchain.cmake

RunBuild
