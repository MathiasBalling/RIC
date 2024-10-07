cmake_minimum_required(VERSION 3.20)
project(Bug_Algorithm
DESCRIPTION "Implementation of bug algorithm with OpenCV"
LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src)
