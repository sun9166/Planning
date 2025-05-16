# Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.

set(CMAKE_SYSTEM_NAME "Linux")
set(CMAKE_SYSTEM_VERSION 1)

set(CROSSCOMPILE $ENV{CROSSCOMPILE})

if(DEFINED CROSSCOMPILE)
   set(CMAKE_SYSTEM_PROCESSOR aarch64)
endif()

# need that one here, because this is a toolchain file and hence executed before
# default cmake settings are set
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a" ".so")

if(DEFINED CROSSCOMPILE)
   set(PROTOBUF_LIBRARIES_CROSS /work/crosscompile/usr/local/lib/libprotobuf.so)
   set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES_CROSS})
else()
   set(PROTOBUF_LIBRARIES_ /usr/local/lib/libprotobuf.so)
   set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES_})
endif()



if(DEFINED CROSSCOMPILE)
set(ARCH "aarch64")
set(VIBRANTE_PDK "/work")
set(TOOLCHAIN "${VIBRANTE_PDK}/crosscompile/gcc-ubuntu-11.4.0-hobot-x86_64-aarch64-linux-gnu/usr")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-g++")
set(CMAKE_C_COMPILER "${TOOLCHAIN}/bin/aarch64-linux-gnu-gcc")
set(CMAKE_LINKER "${TOOLCHAIN}/bin/aarch64-linux-gnu-ld")
set(GCC_COMPILER_VERSION "11.4.0" CACHE STRING "GCC Compiler version")
set(CMAKE_SYSROOT "${VIBRANTE_PDK}/crosscompile")
set(CMAKE_FIND_ROOT_PATH "${VIBRANTE_PDK}/crosscompile")

set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

endif()


# setup compiler for cross-compilation
set(CMAKE_CXX_FLAGS           "-fPIC"               CACHE STRING "c++ flags")
set(CMAKE_CXX_FLAGS           "${CMAKE_C_FLAGS_DEBUG} -pthread")
set(CMAKE_C_FLAGS             "-fPIC"               CACHE STRING "c flags")
set(CMAKE_SHARED_LINKER_FLAGS ""                    CACHE STRING "shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS ""                    CACHE STRING "module linker flags")
set(CMAKE_EXE_LINKER_FLAGS    ""                    CACHE STRING "executable linker flags")

if(DEFINED CROSSCOMPILE)
    set(LD_PATH ${VIBRANTE_PDK}/crosscompile/usr/lib)
    set(LD_PATH_EXTRA ${VIBRANTE_PDK}/crosscompile/gcc-ubuntu-11.4.0-hobot-x86_64-aarch64-linux-gnu/usr/aarch64-linux-gnu)
else()

endif()


# Please, be careful looks like "-Wl,-unresolved-symbols=ignore-in-shared-libs" can lead to silent "ld" problems
set(CMAKE_SHARED_LINKER_FLAGS   "-Wl,--stats -Wl,-rpath,${LD_PATH}  -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_SHARED_LINKER_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS   "-Wl,-rpath,${LD_PATH} -Wl,-rpath,${LD_PATH_EXTRA} ${CMAKE_MODULE_LINKER_FLAGS}")

# Set default library search path
set(CMAKE_LIBRARY_PATH ${LD_PATH})

# Set cmake root path. If there is no "/usr/local" in CMAKE_FIND_ROOT_PATH then FinCUDA.cmake doesn't work
if(DEFINED CROSSCOMPILE)
    set(CMAKE_FIND_ROOT_PATH ${VIBRANTE_PDK/crosscompile/} ${VIBRANTE_PDK}/crosscompile/usr/local/)
else()
# search for programs in the build host directories
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
endif()

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# TRT-5.0.3.2 has a bug for the linux toolchain. This will be fixed by adding stub libraries. Bug - 2468847
set(CMAKE_EXE_LINKER_FLAGS      "-Wl,-rpath,${LD_PATH_EXTRA} -Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,-rpath,${LD_PATH} ${CMAKE_EXE_LINKER_FLAGS}")

# set system default include dir
#link_directories(${VIBRANTE_PDK}/targetfs_a/usr/lib/aarch64-linux-gnu)
#link_directories(${VIBRANTE_PDK}/targetfs_a/usr/lib/python2.7/config-aarch64-linnux-gnu)

