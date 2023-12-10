if (CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
	include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/fast-rtps/include
	)
endif()
if (CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
	include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch/fast-rtps/include
	)
endif()

add_libraries(cm_art_blocker SHARED
	./blocker_manager.cc
)

install(TARGETS cm_art_blocker
	LIBRARY DESTINATION lib
)