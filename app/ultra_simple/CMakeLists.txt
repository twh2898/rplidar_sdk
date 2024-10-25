set(TARGET ultra_simple)

add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} rplidar Threads::Threads)
