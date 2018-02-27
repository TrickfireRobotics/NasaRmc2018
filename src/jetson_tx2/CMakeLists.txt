cmake_minimum_required(VERSION 2.8.3)
project(jetson_tx2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}
    LIBRARIES gpio
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)

add_library(gpio
    src/gpio.c
)


## Mark executables and/or libraries for installation
install(TARGETS gpio
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
