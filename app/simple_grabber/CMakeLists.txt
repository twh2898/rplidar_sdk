set(TARGET simple_grabber)

add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} rplidar)
