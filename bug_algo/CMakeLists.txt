cmake_minimum_required(VERSION 3.20)
project(Bug_Algorithm
DESCRIPTION "Implementation of bug algorithm with OpenCV"
LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_PARALLEL_LEVEL 10)

add_subdirectory(src)

