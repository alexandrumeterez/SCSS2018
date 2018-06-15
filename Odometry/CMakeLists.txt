cmake_minimum_required(VERSION 3.10)
project(Odometry)

set(CMAKE_CXX_STANDARD 11)

add_executable(Odometry main.cpp AccelerometerData.cpp AccelerometerData.h UDPServer.cpp)