add_library(cm_art_record SHARED
    ./record_writer.cc
    ./record_viewer.cc
    ./record_reader.cc
    ./header_builder.cc
    ./file/record_file_base.cc
    ./file/record_file_reader.cc
    ./file/record_file_writer.cc
)

install(TARGETS cm_art_record
        LIBRARY DESTINATION lib 
)
