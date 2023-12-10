enable_language(ASM)

if(CMAKE_SYSTEM_PROCESSOR MATCHES x86_64)
    list(APPEND SRC_FILES "detail/swap_x86_64.S")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES arm64)
    list(APPEND SRC_FILES "detail/swap_aarch64.S")
endif()

add_library(cm_art_croutine SHARED
    ./croutine.cc
    ./detail/routine_context.cc
    ${SRC_FILES}
)

target_link_libraries(cm_art_croutine
    cm_art_common
    atomic
)

install(TARGETS cm_art_croutine
        LIBRARY DESTINATION lib 
)
