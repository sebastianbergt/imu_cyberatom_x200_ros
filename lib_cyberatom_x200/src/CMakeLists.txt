cmake_minimum_required(VERSION 3.10)
project("lib_cyberatom_x200")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set (CMAKE_CXX_STANDARD 11)

add_library(cyberatom_x200_lib STATIC
    CyberAtomUsb.cc X200DataConverter.cc
)

target_link_libraries(cyberatom_x200_lib usb-1.0)
target_include_directories(cyberatom_x200_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})