cmake_minimum_required(VERSION 3.0.2)
project(manipulate_topics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
              roscpp
              dynamic_reconfigure
              tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/mean.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES manipulate_topics
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(message_operators
  include/${PROJECT_NAME}/msg_operators.hpp
  include/${PROJECT_NAME}/operators/vector3.hpp
  include/${PROJECT_NAME}/operators/joint_state.hpp
  include/${PROJECT_NAME}/operators/twist.hpp
  include/${PROJECT_NAME}/operators/twist_stamped.hpp 
  include/${PROJECT_NAME}/operators/wrench.hpp
  include/${PROJECT_NAME}/operators/wrench_stamped.hpp
)
set_target_properties(message_operators PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(message_operators
  ${catkin_LIBRARIES}
)

add_library(message_convolution_filters
  include/${PROJECT_NAME}/msg_filters.hpp

  include/${PROJECT_NAME}/msg_diff_filter.hpp
  
  include/${PROJECT_NAME}/msg_filter_base.hpp

  include/${PROJECT_NAME}/msg_mean_filter.hpp
)
set_target_properties(message_convolution_filters PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(message_convolution_filters
  message_operators
  ${catkin_LIBRARIES}
)
add_dependencies(message_convolution_filters ${PROJECT_NAME}_gencfg)


add_library(msg_transformations
  include/${PROJECT_NAME}/msg_static_transformer.hpp
)
set_target_properties(msg_transformations PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(msg_transformations
  message_operators
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

function(generate_mean_filter message_type)
  add_executable(${message_type}_mean_filter nodes/mean_filter_${message_type}.cpp)
    
  target_link_libraries(${message_type}_mean_filter
    message_convolution_filters
    ${catkin_LIBRARIES}
  )
endfunction()


function(generate_diff_filter message_type)
  add_executable(${message_type}_diff_filter nodes/diff_filter_${message_type}.cpp)
    
  target_link_libraries(${message_type}_diff_filter
    message_convolution_filters
    ${catkin_LIBRARIES}
  )
endfunction()


function(generate_static_transformer message_type)
  add_executable(${message_type}_static_transformer nodes/static_transformer_${message_type}.cpp)
    
  target_link_libraries(${message_type}_static_transformer
    msg_transformations
    ${catkin_LIBRARIES}
  )
endfunction()


generate_mean_filter(wrench_stamped)
generate_mean_filter(joint_state)
generate_mean_filter(twist_stamped)
generate_diff_filter(twist_stamped)

generate_static_transformer(wrench_stamped)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/remove_pose_tf_prefix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulate_topics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
