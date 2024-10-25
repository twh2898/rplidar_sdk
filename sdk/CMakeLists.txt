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
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(${TARGET} PUBLIC Threads::Threads)

install(TARGETS ${TARGET}
    EXPORT ${TARGET}Targets
    DESTINATION lib
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include)
