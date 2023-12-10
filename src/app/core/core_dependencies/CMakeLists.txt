add_library(core_dependencies SHARED
            calibration/control_calibration.cc
            calibration/interpolation_2d.cc
            common/core_gflags.cc)

target_link_libraries(core_dependencies
        ${YAML_CPP_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        )

install(TARGETS core_dependencies
        LIBRARY DESTINATION lib
        )