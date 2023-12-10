add_library(cm_art_time SHARED
    ./clock.cc
    ./duration.cc
    ./rate.cc
    ./time.cc
)

install(TARGETS cm_art_time
        LIBRARY DESTINATION lib 
)
