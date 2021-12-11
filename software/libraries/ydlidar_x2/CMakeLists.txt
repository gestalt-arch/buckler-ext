cmake_minimum_required(VERSION 3.10)

# set the project name
project(lidar_test)

# add the executable
add_executable(lidar ydlidar_x2_test.c ydlidar_x2.c ydlidar_x2.h)

