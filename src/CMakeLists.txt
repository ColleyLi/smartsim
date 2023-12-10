cmake_minimum_required(VERSION ${CMAKE_VERSION})

message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -pipe -02 -Wall -Wextra -fopenmp -fPIC -pthread")

add_link_options(-wl,--disable-new-dtags) # use rpath instead of runpath

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

add_compile_definitions(SIMUDRIVE_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/Lib:$\{ORIGIN\})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR})

find_package(Protobuf 3 EXACT REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

find_Library(GLOG_LIBRARIES glog)
find_path(GLOG_INCLUDE_DIRS "glog/logging.h")

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
Link_directories(${GTEST_LINK_DIRS})

if(CMAKE_BUILD_TYPE MATCHES Debug)
  add_compile_definitions(DEBUG)
endif()

if(CMAKE_MW MATCHES ROS)
  add_compile_definitions(MW_ROS)
  find_package(PCL REQUIRED)
  find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge)
  include_directories(${catkin_INCLUDE_DIRS})
  set(CM_NAME "ros")
  set(PLATFORM_NAME "ipc")
elseif(CMAKE_MW MATCHES AP)
  add_compile_definitions(MW_AP)
  set(AP_INC_PATH ${MDC_SDK}/sysroot/usr/include)
  set(PCL_INCLUDE_DIRS ${AP_INC_PATH}/pcl-1.11)
  set(ACL_HAL_INC_PATH ${AP_INC_PATH}/driver)
  set(ACL_INC_PATH/home/HwHiAiUser/Ascend/ascend-toolkit/5.0.mdc610/acllib_linux1.1.aarch64/runtime/include)
  include_directories(${ACL_HAL_INC_PATH})
  include_directories(${ACL_INC_PATH})
  include_directories(
    ${ara-com_INCLUDE_DIRS}
    ${ara-core_INCLUDE_DIRS}
    ${ara-exec_INCLUDE_DIRS}
    ${Log_INCLUDE_DIRS}
    ${MDC_SDK}/sysroot/usr/include/driver
    ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
    ${MDC_SDK}/sysroot/usr/include/opencv4
    ${MDC_ACLLIB_SDK}/include
    ${MDC_ACLLIB_SDK}/include/acl
    ${MDC_ACLLIB_SDK}/incLude/acl/ops
    ${MDC_ACLLIB_SDK}/include/acl/dvpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cm/ap/generated
  )
  set(0penCV_INC_PATH ${AP_INC_PATH}/opencv4)
  include_directories(${0penCV_INC_PATH})
  set(CM_NAME "ap")
  set(PLATFORM_NAME "mdc810")
elseif(CMAKE_MW MATCHES RT)
  add_definitions(-DMW_RT)
  find_package(PCL REQUIRED)
  set(CM_LIBRARIES ${GLOG_LIBRARIES})
  include_directories(${cyber_SOURCE_DIR})
  include_directories(${cyber_BINARY_DIR})
  find_package(Protobuf REQUIRED)
  if(PROTOBUF_FOUND)
    message(STATUS "Found Protobuf ${PROTOBUF_INCLUDE_DIR}${PROTOBUF_LIBRARIES}")
  else()
    message(FATAL-ERROR “protobuf Mot Foundm)
  endif(PROTOBUF_FOUND)

# SOME/IP
  find_package(CommonAPI-SomeIP REQUIRED)
  find_package (vsomeip3 REQUIRED)
  include_directories(
    ${COMMONAPI_INCLUDE_DIRS}
    ${COMMONAPI_SOMEIP_INCLUDE_DIRS}
    ${VSOMEIP_INCLUDE_DIRS}
  )

  link_directories(
    ${COMMONAPI_LIBDIR}
    ${COMMONAPI_SOMEIP_CMAKE_DIR}/build
  )

  add_compile_options(-02)
  set(CM_NAME "rt")
  set(PLATFORM_NAME "a1000")
endif()

include_directories(${PCL_INCLUDE_DIRS})


add_subdirectory(cm)
add_subdirectory(proto)
add_subdirectory(app)
add_subdirectory(common)