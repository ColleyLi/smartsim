add_subdirectory(proto)

set(CMAKE_CXX_FLAGS "-fext-numeric-literals -pipe -O2 -Wall -Wextra -Wextra -fopenmp -fPIC -pthread")

add_definitions(-w)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    include_directories(
#        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/fast-rtps/include
#				${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/ncurses/include
    )
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    include_directories(
#        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/fast-rtps/include
    )
endif()

#1.0
add_subdirectory(common)
add_subdirectory(message)

#2.0
add_library(cm_art_base SHARED
    ./state.cc
    ./binary.cc
)

install(TARGETS cm_art_base
        LIBRARY DESTINATION lib 
)

#3.0
add_subdirectory(event)
add_subdirectory(croutine)
add_subdirectory(time)
add_subdirectory(timer)
add_subdirectory(transport)
add_subdirectory(service_discovery)
add_subdirectory(node)
add_subdirectory(blocker)
add_subdirectory(class_loader)
add_subdirectory(io)
add_subdirectory(logger)
add_subdirectory(parameter)
add_subdirectory(scheduler)
add_subdirectory(sysmo)
add_subdirectory(task)
add_subdirectory(component)
add_subdirectory(record)

#4.0
add_library(cm_art_core SHARED
    ./cyber.cc
    ./init.cc
)
target_link_libraries(cm_art_core
    cm_art_service_discovery
    cm_art_logger
    cm_art_event
    cm_art_common 
    cm_art_time 
    cm_art_croutine 
    cm_art_scheduler 
    cm_art_class_loader 
    cm_art_timer 
    cm_art_task
    cm_art_base
    cm_art_component
    cm_art_node
    cm_art_blocker
    cm_art_transport
    cm_art_sysmo
    cm_art_record
    
    ${GLOG_LIBRARIES}
    protobuf
    gflags
    
    cm_cyber_proto

    cm_proto
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    target_link_libraries(cm_art_core
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/libuuid.so
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/librt.so.1
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/fast-rtps/lib
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/x86/poco/lib
    )
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    target_link_libraries(cm_art_core
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/libuuid.so
    #    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aarch64/librt.so.1
    )
endif()

install(TARGETS cm_art_core
        LIBRARY DESTINATION lib 
)

#5.0
add_subdirectory(mainboard)

add_subdirectory(tools)
