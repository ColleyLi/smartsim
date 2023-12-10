add_library(common_util
            proto_utils.cc)   

target_link_libraries(common_util
                      ${catkin_LIBRARIES})   

install(TARGETS common_util
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)