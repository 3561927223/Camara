cmake_minimum_required(VERSION 3.16)

project(double_s_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ds_planner
    src/ds_planner.cpp
)
target_include_directories(ds_planner PUBLIC ${CMAKE_SOURCE_DIR}/src)

add_executable(ds_preview src/main.cpp)
target_link_libraries(ds_preview PRIVATE ds_planner)

