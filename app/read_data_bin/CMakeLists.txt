set(TARGET read_data_bin)

add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} rplidar fmt::fmt)
