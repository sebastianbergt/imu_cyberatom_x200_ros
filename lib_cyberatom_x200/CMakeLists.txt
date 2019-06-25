cmake_minimum_required(VERSION 3.10)
project("lib_cyberatom_x200")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set (CMAKE_CXX_STANDARD 11)

add_subdirectory(extern/googletest)

option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

add_subdirectory(src)
add_subdirectory(example)

