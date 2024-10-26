set(RPLIDAR_VERSION @PROJECT_VERSION@)

@PACKAGE_INIT@

find_dependency(fmt)

check_required_components(@PROJECT_NAME@)