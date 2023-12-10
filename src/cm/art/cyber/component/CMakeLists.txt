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

add_library(cm_art_component SHARED
    ./timer_component.cc
)

target_link_libraries(cm_art_service_discovery
    cm_art_scheduler
    cm_art_common
    cm_art_croutine
)

install(TARGETS cm_art_component
        LIBRARY DESTINATION lib 
)