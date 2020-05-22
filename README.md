# ENPM809Y-FinalProject
Group 4 Final Project - A robot navigates through a maze to reach the center of the maze

This is a program to drive a robot through a maze using Depth First Search as it's path-planning algorithm. The program is interfaced with Micromouse simulator for visualizing the maze and the robot in real-time.The concepts of object-oriented programming, inheritance, and dynamic polymorphism were used to develop this project.

It is currently hosted on GitHub at https://github.com/Karansutradhar/ENPM809Y-FinalProject

# Overview
Steps to run this program:-

1. Visit this Github link https://github.com/mackorone/mms#building-from-source to install Micromouse simulator.
2. Build the simulator using the following command: /user# cd mms/src /user# qmake && make
3. Run the simulator using the following command: ../bin/mms
4. Parameters to be changed in the simulator:-
  Name:- Input your desired name.
  Directory:- ../../Final-Project-Group4/Final_Project/src
  Build Command:- g++ -std=c++14 API/api.cpp LandBasedRobot/landbasedrobot.cpp LandBasedTracked/landbasedtracked.cpp LandBasedWheeled/landbasedwheeled.cpp Maze/maze.cpp Algorithm/algorithm.cpp main.cpp
  Run Command:- ./a.out
5. Run the Simulator.
