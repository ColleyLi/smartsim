include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_libraries(cm_art_class_loader SHARED
	./utility/class_factory.cc
	./utility/class_loader_utility.cc
	./class_loader.cc
	./class_loader_manager.cc
)

if (CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
	target_link_libraries(
		${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/x86/poco/lib/libPocoFoundation.so
	)
endif()
if (CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
	target_link_libraries(
		${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/aarch64/poco/lib/libPocoFoundation.so
	)
endif()

install(TARGETS cm_art_class_loader
	LIBRARY DESTINATION lib
)