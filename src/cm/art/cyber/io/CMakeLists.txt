add_library(cm_art_io SHARED
    ./poll_handler.cc
    ./poller.cc
    ./session.cc
)

install(TARGETS cm_art_io
        LIBRARY DESTINATION lib 
)