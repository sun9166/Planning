# Install script for directory: /home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/out/libosqp.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osqp" TYPE FILE FILES
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/auxil.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/constants.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/cs.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/ctrlc.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/glob_opts.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/kkt.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/lin_alg.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/lin_sys.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/osqp.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/osqp_configure.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/types.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/polish.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/proj.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/scaling.h"
    "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/include/util.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/out/libosqp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake"
         "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/CMakeFiles/Export/04d30f429d9157da04e6b5103179a7bf/osqp-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/CMakeFiles/Export/04d30f429d9157da04e6b5103179a7bf/osqp-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/CMakeFiles/Export/04d30f429d9157da04e6b5103179a7bf/osqp-targets-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/osqp-config.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/lin_sys/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hc/J6_project/Regulatory_code/robotaxi-bus-planning/common/common_lib/osqp/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
