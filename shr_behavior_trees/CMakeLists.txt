cmake_minimum_required(VERSION 3.0.2)
project(shr_behavior_trees)

set(ROS_DEPENDENCIES
roscpp 
std_msgs
behaviortree_cpp_v3
actionlib_msgs
actionlib
shr_interfaces
tf2
tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

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
    # INCLUDE_DIRS include
    # LIBRARIES
    # CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(test_bt src/test_bt.cpp 
src/nodes/pick_node.cpp 
src/nodes/move_to_target_node.cpp
src/nodes/move_gripper_to_target_node.cpp
src/nodes/move_to_pose_node.cpp
src/nodes/move_gripper_node.cpp
src/nodes/place_node.cpp
src/nodes/remove_object_node.cpp
src/nodes/detach_object_node.cpp
src/nodes/reset_node.cpp
src/nodes/clear_octomap_node.cpp
src/nodes/enable_camera_node.cpp
src/nodes/wait_node.cpp
src/nodes/add_object_node.cpp
)
add_dependencies(test_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bt ${catkin_LIBRARIES} )

add_executable(behavior_tree_server src/behavior_tree_server.cpp 
src/nodes/pick_node.cpp 
src/nodes/move_to_target_node.cpp
src/nodes/move_gripper_to_target_node.cpp
src/nodes/move_to_pose_node.cpp
src/nodes/move_gripper_node.cpp
src/nodes/place_node.cpp
src/nodes/remove_object_node.cpp
src/nodes/detach_object_node.cpp
src/nodes/reset_node.cpp
src/nodes/clear_octomap_node.cpp
src/nodes/enable_camera_node.cpp
src/nodes/wait_node.cpp
src/nodes/add_object_node.cpp
)
add_dependencies(behavior_tree_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_tree_server ${catkin_LIBRARIES} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_shr_behavior_trees.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
