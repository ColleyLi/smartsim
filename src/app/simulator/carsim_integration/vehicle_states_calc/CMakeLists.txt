aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CARSIM_CALC_SRCS)

add_library(carsim_calc SHARED
            ${CARSIM_CALC_SRCS})

target_link_libraries(carsim_calc
        carsim_dependencies
        ${YAML_CPP_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        dl
        )

install(TARGETS carsim_calc
        LIBRARY DESTINATION lib
        )