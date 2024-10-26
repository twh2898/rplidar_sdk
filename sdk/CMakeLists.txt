set(TARGET rplidar)

file(GLOB SOURCE_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataunpacker/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataunpacker/unpacker/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arch/linux/*.cpp)
file(GLOB HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataunpacker/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dataunpacker/unpacker/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arch/linux/*.h)

add_library(${TARGET} ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(${TARGET}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${TARGET} PUBLIC Threads::Threads)

# Add ALIAS target for use in add_subdirectory or build directory
add_library(${PROJECT_NAME}::${TARGET} ALIAS ${TARGET})

# Add target to Project Targets for install
install(TARGETS ${TARGET}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Include the headers in install
install(DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}/sdk/include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
