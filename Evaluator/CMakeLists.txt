cmake_minimum_required(VERSION 2.6)
project(evaluator)
set(CMAKE_CXX_FLAGS  "-std=c++11")
SET(CMAKE_BUILD_TYPE Release)

include_directories( "/usr/include/eigen3" )

add_library(${PROJECT_NAME} SHARED evaluator.cpp)

