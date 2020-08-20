cmake_minimum_required(VERSION 3.0.2)
project(iiwa_workcell_sensor)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs tf
)

catkin_install_python(PROGRAMS
  scripts/range_faker.py
  scripts/range_to_pointcloud2_converter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
