library(log_format log_format.cxx)
target_link_libraries(
    log_format
    yaml
    utctime
    galileo
    glonass
    gps
)
add_gtest(log_format_test log_format_test.cxx)
target_link_libraries(log_format_test log_format)

library(serialize serialize.cxx)
target_link_libraries(
    serialize
    gps
    galileo
    glonass
    utctime
)
add_gtest(serialize_test serialize_test.cxx)
target_link_libraries(serialize_test serialize)

library(log_writer log_writer.cxx)
target_link_libraries(
    log_writer
    log_format
    yaml
    file
    serialize
)
add_gtest(log_writer_test log_writer_test.cxx)
target_link_libraries(log_writer_test log_writer)

library(log_reader log_reader.cxx)
target_link_libraries(
    log_reader
    yaml
    serialize
    print
    stdc++fs
)
add_gtest(log_reader_test log_reader_test.cxx)
target_link_libraries(log_reader_test log_reader log_writer)
