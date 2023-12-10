add_library(bridge_common SHARED
            bridge_gflags.cc)   

target_link_libraries(bridge_common
                      ${catkin_LIBRARIES})   

install(TARGETS bridge_common
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)