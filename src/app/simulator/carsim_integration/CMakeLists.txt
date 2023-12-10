aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} CARSIM_SRCS)
add_executable(carsim_simulation ${CARSIM_SRCS})
target_link_libraries(carsim_simulation
                        carsim_calc
                        )

install(TARGETS carsim_simulation
        RUNTIME DESTINATION bin)

add_subdirectory(vehicle_states_calc)
add_subdirectory(carsim_dependencies)