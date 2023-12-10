include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../proto)

add_library(cm_art_event SHARED
    ./perf_event_cache.cc
)

install(TARGETS cm_art_event
        LIBRARY DESTINATION lib 
)