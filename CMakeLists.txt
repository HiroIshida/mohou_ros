cmake_minimum_required(VERSION 3.0.2)
project(mohou_ros)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED
    std_msgs
    message_generation
)

add_message_files(
    FILES
    ControlCommand.msg
)

add_service_files(
  FILES
  EuslispDirectCommand.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(rostest/data_collection.test)
  add_rostest(rostest/train_pipeline.test)
  add_rostest(rostest/executor.test)
  add_rostest(rostest/euslisp_core/test_pr2eus_commander.test)
endif()
