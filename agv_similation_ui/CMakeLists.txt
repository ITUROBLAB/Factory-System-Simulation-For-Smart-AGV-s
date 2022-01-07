cmake_minimum_required(VERSION 3.0.2)
project(agv_similation_ui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(MESSAGE_DEPENDENCIES
std_msgs
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  message_generation
  ${MESSAGE_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# add_service_files(
#   FILES
#   std_srvs.srv
#   )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()



## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   ${MESSAGE_DEPENDENCIES}
# )

catkin_package( 
#  INCLUDE_DIRS include
#  LIBRARIES elr3delta_ethercat_ui
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
