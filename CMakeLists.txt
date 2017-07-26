cmake_minimum_required(VERSION 2.8.3)


project(test_polishing)

# Add support for c++11
add_definitions(-std=c++11)




# Find required packages
find_package(catkin REQUIRED COMPONENTS
  lwr_ros_client
  ds_motion_generator
  dynamic_reconfigure
  roscpp
  rospy
  serial
  std_msgs
)


# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/object_params.cfg)


# find_package(Eigen REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

 catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS lwr_ros_client ds_motion_generator roscpp rospy serial std_msgs dynamic_reconfigure
    DEPENDS Eigen
 )




include_directories(include ${catkin_INCLUDE_DIRS} ${ds_motion_generator_INCLUDE_DIRS})

add_executable(talkerKuka src/talkerKuka.cpp)
target_link_libraries(talkerKuka ${catkin_LIBRARIES})

add_executable(listenerKuka src/listenerKuka.cpp)
target_link_libraries(listenerKuka ${catkin_LIBRARIES})

add_executable(moveToDesiredJoints src/moveToDesiredJoints_node.cpp src/MoveToDesiredJoints.cpp)
target_link_libraries(moveToDesiredJoints ${catkin_LIBRARIES})

add_executable(polishingMotionGeneration src/polishingMotionGeneration_node.cpp)
target_link_libraries(polishingMotionGeneration ds_motion_generator ${catkin_LIBRARIES})
add_dependencies(polishingMotionGeneration ${PROJECT_NAME}_gencfg)


add_executable(movingObject src/movingObject_node.cpp src/MovingObject.cpp)
target_link_libraries(movingObject ${catkin_LIBRARIES})
add_dependencies(movingObject ${PROJECT_NAME}_gencfg)

add_executable(convoyer src/convoyer_node.cpp)
target_link_libraries(convoyer ${catkin_LIBRARIES})
