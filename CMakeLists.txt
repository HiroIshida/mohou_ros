cmake_minimum_required(VERSION 3.0.2)
project(mohou_ros)
find_package(catkin REQUIRED)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(rostest/pipeline.test)
endif()
