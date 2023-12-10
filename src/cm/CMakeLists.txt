file(GLOB LIB_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.c)

find_package(Boost REQUIRED COMPONENTS filesystem system iostreams thread)
include_directories(${Boost_INCLUDE_DIRS})

if(CMAKE_MW STREQUAL "ROS")

  set(CM_LIBRARIES ${catkin_LIBRARIES} ${GLOG_LIBRARIES})
  include_directories(${catkin_INCLUDE_DIRS})

elseif(CMAKE_MW STREQUAL "ROSM")

  set(CM_LIBRARIES ${catkin_LIBRARIES} ${GLOG_LIBRARIES})
  include_directories(include ${catkin_INCLUDE_DIRS})
  
  add_library(communication ${LIB_SRCS})

  target_link_libraries(communication ${CM_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(communication apollo)
  add_subdirectory(ros_automsg)

  install(TARGETS communication 
          LIBRARY 
          DESTINATION lib
  )
  return()

elseif(CMAKE_MW STREQUAL "CAP")
  
  add_library(communication ${LIB_SRCS})
  target_link_libraries(communication ${CM_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(communication apollo)
  add_subdirectory(${CM_NAME})
  install(TARGETS communication 
    LIBRARY 
    DESTINATION lib
  )
  return()

elseif(CMAKE_MW STREQUAL "AP")
  set(AP_INC_PATH ${MDC_SDK}/sysroot/usr/include)
  set(ACL_HAL_INC_PATH ${AP_INC_PATH}/driver)
  set(ACL_INC_PATH /home/HwHiAiUser/Ascend/ascend-toolkit/5.0.mdc610/acllib_linux1.1.aarch64/runtime/include)
  include_directories(${ACL_INC_PATH})
  include_directories(${ACL_HAL_INC_PATH})
  find_package(ara-com REQUIRED)
  find_package(ara-core REQUIRED)
  find_package(ara-exec REQUIRED)
  find_package(log REQUIRED)
  include_directories(${ara-com_INCLUDE_DIRS}
                      ${ara-core_INCLUDE_DIRS}
                      ${ara-exec_INCLUDE_DIRS}
                      ${log_INCLUDE_DIRS}
                      ${MDC_SDK}/sysroot/usr/include/driver
                      ${MDC_SDK}/sysroot/usr/include/adsfi/adb/include
                      ${MDC_SDK}/sysroot/usr/include/opencv4
                      ${MDC_ACLLIB_SDK}/include
                      ${MDC_ACLLIB_SDK}/include/acl
                      ${MDC_ACLLIB_SDK}/include/acl/ops
                      ${MDC_ACLLIB_SDK}/include/acl/dvpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/ap/generated)
  link_directories(${MDC_SDK}/sysroot/usr/lib
                   ${MDC_SDK}/sysroot/usr/lib/mdc_vector
                   ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
                   ${MDC_ACLLIB_SDK}/lib64/stub
                   ${MDC_ACLLIB_SDK}/lib64)
  set(CM_LIBRARIES ${ara-com_LIBRARIES}
                   ${ara-core_LIBRARIES}
                   ${ara-exec_LIBRARIES}
                   ${GLOG_LIBRARIES}
                   adb
                   ascendcl
                   acl_dvpp_mpi
                   mdc_aiv
                   ascend_hal
                   rm
                   pthread)
  # AscendCL for AI
  include_directories(${MDC_SDK}/sysroot/usr/local/Ascend/runtime/include
                      ${MDC_SDK}/sysroot/usr/include/driver)
  link_directories(${MDC_SDK}/sysroot/usr/local/Ascend/runtime/lib64
                   ${MDC_SDK}/sysroot/usr/local/Ascend/runtime/lib64/stub)

elseif(CMAKE_MW STREQUAL "RT")

  set(CM_LIBRARIES  ${GLOG_LIBRARIES})
  # set(CM_LIBRARIES ads_log autoplt autoplt_example_proto cyber cm_cyber_proto fastrtps protobuf pthread)

elseif(CMAKE_MW STREQUAL "TZ")

  set(CM_LIBRARIES ${GLOG_LIBRARIES})
  find_package(Threads REQUIRED)
  if(PLATFORM_NAME STREQUAL "J5")
    set(LIB_DIR /opt/link)
    link_directories(${LIB_DIR})
    add_library(communication ${LIB_SRCS})
    add_definitions(-DENABLE_DMA)
    add_subdirectory(tz)
    target_link_libraries(communication pthread mwcomm vsomeip3 
    vsomeip3-cfg vsomeip3-e2e vsomeip3-sd fastcdr 
    fastrtps hbmem hbpcie tzcjson slog ${Boost_LIBRARIES}
    /opt/link/libhbmem.so
                      /opt/link/libalog.so
                      /opt/link/libion.so)
    add_dependencies(communication apollo)

    install(TARGETS communication LIBRARY DESTINATION lib)
    return()

  elseif(PLATFORM_NAME STREQUAL "X9")
    set(LIB_DIR /opt/lib_x9u)
    link_directories(${LIB_DIR})
    set(CM_LIBRARIES mwcomm fastcdr pthread vsomeip3 
    fastrtps boost_system boost_thread boost_filesystem boost_chrono
    boost_atomic rt dl crypto ssl slog tzcjson)
  endif()

elseif(CMAKE_MW STREQUAL "ART")

  set(CM_LIBRARIES  ${GLOG_LIBRARIES})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/art)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/art)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/x86/fast-rtps/include
    )
  endif()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/aarch64/fast-rtps/include
    )
  endif()
endif()

#add_subdirectory(${CM_NAME})
add_subdirectory("art")
add_library(communication ${LIB_SRCS})

if(CMAKE_MW STREQUAL "ROS")
  add_dependencies(communication ros_custom_msgs_generate_messages_cpp)
endif()

if(CMAKE_MW STREQUAL "ART")
  add_dependencies(communication cm_art_core)
endif()

target_link_libraries(communication ${CM_LIBRARIES} ${Boost_LIBRARIES})

#安装核心库
install(TARGETS communication LIBRARY DESTINATION lib)

#安装配置文件
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/art/cyber/conf
  DESTINATION include/src/cm/art/cyber
  PATTERN "fastcdr" EXCLUDE
)

#安装依赖库
if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/x86/fast-rtps/lib/ 
    DESTINATION lib
    PATTERN "fastcdr" EXCLUDE
		PATTERN "fastrtps" EXCLUDE
	)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/x86/poco/lib/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/aarch64/fast-rtps/lib/ 
    DESTINATION lib
    PATTERN "fastcdr" EXCLUDE
		PATTERN "fastrtps" EXCLUDE
  )
  install(
      DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/art/third_party/aarch64/poco/lib/
      DESTINATION lib
  )
endif()
