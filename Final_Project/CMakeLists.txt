cmake_minimum_required(VERSION 3.15)
project(Final_Project)

set(CMAKE_CXX_STANDARD 14)

add_executable(Final_Project main.cpp src/LandBasedRobot/landbasedrobot.cpp src/LandBasedRobot/landbasedrobot.h src/Maze/maze.cpp src/Maze/maze.h src/Algorithm/algorithm.cpp src/Algorithm/algorithm.h src/API/api.cpp src/API/api.h src/LandBasedTracked/landbasedtracked.cpp src/LandBasedTracked/landbasedtracked.h src/LandBasedWheeled/landbasedwheeled.cpp src/LandBasedWheeled/landbasedwheeled.h)