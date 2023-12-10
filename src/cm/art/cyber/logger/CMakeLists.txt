find_package(gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

add_library(cm_art_logger SHARED
    ./async_logger.cc
    ./log_file_object.cc
    ./logger_util.cc
    ./logger.cc
)

target_link_libraries(cm_art_logger ${GFLAGS_LIBRARIES})

install(TARGETS cm_art_logger
        LIBRARY DESTINATION lib 
)