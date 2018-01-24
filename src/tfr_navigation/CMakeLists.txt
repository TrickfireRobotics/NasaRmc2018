cmake_minimum_required(VERSION 2.8.3)
project(tfr_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfr_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  actionlib
)

find_package(GTest REQUIRED)

catkin_package(
)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_library(navigation_goal_manager src/navigation_goal_manager.cpp)
target_link_libraries(navigation_goal_manager ${catkin_LIBRARIES}) 

add_executable(navigation_action_server 
    src/navigation_action_server.cpp
    src/navigator.cpp
)
target_link_libraries(navigation_action_server ${catkin_LIBRARIES} navigation_goal_manager)
add_dependencies(navigation_action_server ${catkin_EXPORTED_TARGETS})

add_executable(navigation_action_client 
    src/navigation_action_client.cpp
    src/navigation_client.cpp
)
target_link_libraries(navigation_action_client ${catkin_LIBRARIES})
add_dependencies(navigation_action_client ${catkin_EXPORTED_TARGETS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/test_goal_setting.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} navigation_goal_manager)
endif()



