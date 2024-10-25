cmake_minimum_required(VERSION 3.5.0)
project(rplidar VERSION 0.1.0 LANGUAGES C CXX)

set (CMAKE_CXX_STANDARD 17)

find_package(fmt REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(sdk)
add_subdirectory(app)
