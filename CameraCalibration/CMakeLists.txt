cmake_minimum_required(VERSION 3.10)
project(CameraCalibration)

set(CMAKE_CXX_STANDARD 11)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(CameraCalibration main.cpp)
target_link_libraries(CameraCalibration ${OpenCV_LIBS})