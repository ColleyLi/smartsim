add_library(common_adapters
                adapter_gflags.cc)   

target_link_libraries(common_adapters
                      ${catkin_LIBRARIES})   

install(TARGETS common_adapters
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)