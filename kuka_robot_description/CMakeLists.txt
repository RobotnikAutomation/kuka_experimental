cmake_minimum_required(VERSION 3.0.2)
project(kuka_robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY robot
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
