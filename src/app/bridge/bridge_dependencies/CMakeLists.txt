add_library(bridge_dependencies SHARED
            common/bridge_gflags.cc)

target_link_libraries(bridge_dependencies
        ${YAML_CPP_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        )

install(TARGETS bridge_dependencies
        LIBRARY DESTINATION lib
        )