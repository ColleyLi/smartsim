if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/include
    )
endif()
if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/include
    )
endif()

add_library(cm_art_service_discovery SHARED
    ./topology_manager.cc
    ./communication/participant_listener.cc 
    ./communication/subscriber_listener.cc 
    ./container/graph.cc 
    ./container/multi_value_warehouse.cc 
    ./container/single_value_warehouse.cc 
    ./role/role.cc 
    ./specific_manager/channel_manager.cc 
    ./specific_manager/manager.cc 
    ./specific_manager/node_manager.cc 
    ./specific_manager/service_manager.cc 
)

target_link_libraries(cm_art_service_discovery
    cm_art_time
    cm_art_message
    cm_art_transport
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    target_link_libraries(cm_art_service_discovery
        # ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/lib/libfastrtps.so
        # ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/lib/libfastcdr.so
        # ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/libuuid.so
        /lib/x86_64-linux-gnu/librt.so.1
    )
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    target_link_libraries(cm_art_service_discovery
        # ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/lib/libfastrtps.so
        # ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/lib/libfastcdr.so
		# ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/libuuid.so
        /usr/lib/aarch64-linux-gnu/librt.so.1
    )
endif()

install(TARGETS cm_art_service_discovery
        LIBRARY DESTINATION lib 
)
