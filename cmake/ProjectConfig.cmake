set(RPLIDAR_VERSION @PROJECT_VERSION@)

@PACKAGE_INIT@

find_dependency(Threads REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake")

check_required_components(@PROJECT_NAME@)
