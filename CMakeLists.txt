cmake_minimum_required(VERSION 3.15..3.20)
project(rplidar VERSION 2.1.0 LANGUAGES C CXX)

set(RPLIDAR_IS_MASTER_PROJECT OFF)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    set(RPLIDAR_IS_MASTER_PROJECT ON)
endif()

option(RPLIDAR_BUILD_EXAMPLES "Builds the rplidar examples" ${RPLIDAR_IS_MASTER_PROJECT})

# Use for platform independent install dirs, despite being called GNU
include(GNUInstallDirs)

# Create config.h with project version numbers
configure_File(cmake/config.h.in include/config.h)
include_directories(PRIVATE ${CMAKE_BINARY_DIR}/include)

set(CMAKE_CXX_STANDARD 17)

find_package(fmt 11.0.2 REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(sdk)

if(RPLIDAR_BUILD_EXAMPLES)
    add_subdirectory(app)
endif()

# Create Targets file
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

# Generate ConfigVersion file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion)

# Copy the Config file to the build dir
configure_file(cmake/ProjectConfig.cmake ${PROJECT_NAME}Config.cmake @ONLY)

# Add Config and ConfigVersion files to install
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
