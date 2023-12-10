add_library(core_common
        core_gflags.cc)

target_link_libraries(core_common
        ${catkin_LIBRARIES})

install(TARGETS core_common
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
