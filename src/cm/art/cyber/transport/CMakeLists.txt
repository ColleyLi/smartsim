# include_directories(/usr/local/lib/fast-rtps/include)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/include)

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

add_library(cm_art_transport SHARED
    ./transport.cc
    ./common/endpoint.cc
    ./common/identity.cc
    ./dispatcher/dispatcher.cc
    ./dispatcher/intra_dispatcher.cc
    ./dispatcher/rtps_dispatcher.cc
    ./dispatcher/shm_dispatcher.cc 
    ./message/message_info.cc 
    ./qos/qos_profile_conf.cc 
    ./rtps/attributes_filler.cc
    ./rtps/participant.cc 
    ./rtps/sub_listener.cc
    ./rtps/underlay_message_type.cc
    ./rtps/underlay_message.cc
    ./shm/block.cc
    ./shm/condition_notifier.cc
    ./shm/multicast_notifier.cc
    ./shm/notifier_factory.cc
    ./shm/posix_segment.cc
    ./shm/readable_info.cc
    ./shm/segment_factory.cc
    ./shm/segment.cc
    ./shm/shm_conf.cc
    ./shm/state.cc
    ./shm/xsi_segment.cc
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    target_link_libraries(cm_art_transport
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/lib/libfastrtps.so
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/lib/libfastcdr.so
        /usr/lib/x86_64-linux-gnu/libuuid.so
    )
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    target_link_libraries(cm_art_transport
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/lib/libfastrtps.so
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/fast-rtps/lib/libfastcdr.so
		${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/libuuid.so

    )
endif()

install(TARGETS cm_art_transport
        LIBRARY DESTINATION lib 
)
