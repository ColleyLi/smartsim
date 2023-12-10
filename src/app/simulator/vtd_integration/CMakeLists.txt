cmake_minimum_required(VERSION 3.20.5)

project(vtd_simulation
        DESCRIPTION "vtd simulation entry point")

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/perc_object_lists PERC_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/bev_detected BEV_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/query_ego_states VEHICLE_STATES_SRCS)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/trigger TRIGGER_SRCS)
#
#add_library(perc_libs SHARED
#        ${PERC_SRCS})
#target_link_libraries(perc_libs
#        vtd_dependencies)
add_library(bev_libs SHARED
        ${BEV_SRCS})
target_link_libraries(bev_libs
        vtd_dependencies)
add_library(vehicle_states_libs SHARED
        ${VEHICLE_STATES_SRCS})
target_link_libraries(vehicle_states_libs
        vtd_dependencies)
#add_library(trigger_libs SHARED
#        ${TRIGGER_SRCS})
#target_link_libraries(trigger_libs
#        vtd_dependencies)

add_executable(vtd_simulation vtd_simulation.cc)
target_link_libraries(vtd_simulation
#        perc_libs
        bev_libs
        vehicle_states_libs
        ${YAML_CPP_LIBRARIES})

add_executable(vtd_trigger ${TRIGGER_SRCS})
target_link_libraries(vtd_trigger
        vtd_dependencies
        ${YAML_CPP_LIBRARIES})

add_executable(vtd_ego ${VEHICLE_STATES_SRCS})
target_link_libraries(vtd_ego
        vtd_dependencies
        ${YAML_CPP_LIBRARIES})

add_subdirectory(vtd_dependencies)

#install(TARGETS perc_libs
#        LIBRARY
#        DESTINATION lib)
install(TARGETS bev_libs
        LIBRARY
        DESTINATION lib)
install(TARGETS vehicle_states_libs
        LIBRARY
        DESTINATION lib)
install(TARGETS vtd_trigger
        RUNTIME
        DESTINATION bin)
install(TARGETS vtd_ego
        RUNTIME
        DESTINATION bin)
install(TARGETS vtd_simulation
        RUNTIME
        DESTINATION bin)
