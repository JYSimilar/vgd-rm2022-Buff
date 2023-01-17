cmake_minimum_required(VERSION 3.23)
project(rm2023_bigDickEnergy)

set(CMAKE_CXX_STANDARD 20)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include)

add_executable(rm2023_bigDickEnergy main.cpp
        big_dick_energy.cpp
        big_dick_energy.h
        define.h predictor.cpp predictor.h)

target_link_libraries(rm2023_bigDickEnergy ${OpenCV_LIBS})
