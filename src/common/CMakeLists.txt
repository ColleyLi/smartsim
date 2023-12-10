project(common)

add_library(common SHARED
		adapters/adapter_gflags.cc
        util/proto_util.cc
        util/yaml_util.cc
		time/timer.cpp
		math/math_utils.cc
		log/Logger.cpp)
target_link_libraries(common
		${YAML_CPP_LIBRARIES}
)
install(TARGETS common
	ARCHIVE DESTINATION lib)