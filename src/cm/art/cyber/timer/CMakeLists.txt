add_library(cm_art_timer SHARED
    ./timing_wheel.cc
    ./timer.cc
)

install(TARGETS cm_art_timer
        LIBRARY DESTINATION lib 
)
