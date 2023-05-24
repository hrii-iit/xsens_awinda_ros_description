cmake_minimum_required(VERSION 3.0.2)
project(xsens_awinda_ros_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xacro
  urdf
)

catkin_package()

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Instructions to install urdf files
install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
