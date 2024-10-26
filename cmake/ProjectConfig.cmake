set(RPLIDAR_VERSION @PROJECT_VERSION@)

@PACKAGE_INIT@

find_dependency(Threads REQUIRED)
find_dependency(fmt 11.0 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake")

check_required_components(@PROJECT_NAME@)