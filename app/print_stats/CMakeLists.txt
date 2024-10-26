set(TARGET print_stats)

add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} rplidar fmt::fmt Threads::Threads)
