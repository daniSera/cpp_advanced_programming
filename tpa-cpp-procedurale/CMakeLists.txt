cmake_minimum_required(VERSION 3.10)
project(es01)

set(CMAKE_CXX_STANDARD 14)

add_library(es01_library STATIC
        src/siso.cpp
        src/mimo.cpp
        src/mimo_struct.cpp)
target_include_directories(es01_library PRIVATE include)

add_executable(es01 main.cpp)
target_link_libraries(es01 es01_library)
target_include_directories(es01 PRIVATE include)

add_subdirectory(test)

