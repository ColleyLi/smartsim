add_libraries(cm_art_common SHARED
	./file.cc
	./global_data.cc
)

install(TARGETS cm_art_common
	LIBRARY DESTINATION lib
)
