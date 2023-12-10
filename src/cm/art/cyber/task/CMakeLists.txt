add_library(cm_art_task SHARED
    ./task_manager.cc
)

target_link_libraries(cm_art_service_discovery
    cm_art_scheduler
    cm_art_common
    cm_art_croutine
)

install(TARGETS cm_art_task
        LIBRARY DESTINATION lib 
)
