file(GLOB_RECURSE CARSIM_DEPEND_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cc)

add_library(carsim_dependencies SHARED
        ${CARSIM_DEPEND_SRCS})

target_link_libraries(carsim_dependencies
        )

install(TARGETS carsim_dependencies
        LIBRARY DESTINATION lib
        )