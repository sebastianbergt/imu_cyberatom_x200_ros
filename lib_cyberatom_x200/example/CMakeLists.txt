cmake_minimum_required(VERSION 3.10)
project("example_lib_cyberatom_x200")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set (CMAKE_CXX_STANDARD 11)

add_executable(example main.cc)
target_link_libraries(example cyberatom_x200_lib)