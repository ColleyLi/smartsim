add_library(control_calibration SHARED
            control_calibration.cc
            interpolation_2d.cc
)
target_link_libraries(control_calibration 
                      ${catkin_LIBRARIES}
)

install(TARGETS control_calibration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)