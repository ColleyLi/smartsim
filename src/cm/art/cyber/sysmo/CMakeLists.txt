add_library(cm_art_sysmo SHARED
    ./sysmo.cc
)

target_link_libraries(cm_art_service_discovery
    cm_art_scheduler
    cm_art_common
)

install(TARGETS cm_art_sysmo
        LIBRARY DESTINATION lib 
)
