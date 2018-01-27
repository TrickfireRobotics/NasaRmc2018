cmake_minimum_required(VERSION 2.8.3)
project(tfr_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfr_msgs
  geometry_msgs
  nav_msgs
  actionlib
)

find_package(GTest REQUIRED)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

catkin_package(
)


add_library( navigation_server
    src/navigation_goal_manager.cpp
    src/navigator.cpp
)
add_dependencies(navigation_server ${catkin_EXPORTED_TARGETS})

add_library( navigation_client 
    src/navigation_client.cpp
)
add_dependencies(navigation_client ${catkin_EXPORTED_TARGETS})


add_executable(navigation_action_server 
    src/navigation_action_server.cpp
)
add_dependencies(navigation_action_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_action_server navigation_server ${catkin_LIBRARIES})

add_executable(navigation_action_client 
    src/navigation_action_client.cpp
)
add_dependencies(navigation_action_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_action_client navigation_client ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/test_goal_setting.cpp)
    target_link_libraries(${PROJECT_NAME}-test navigation_server ${catkin_LIBRARIES})
endif()



